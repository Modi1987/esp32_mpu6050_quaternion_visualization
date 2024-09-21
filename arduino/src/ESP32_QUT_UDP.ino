// Copyright: Mohammad Safeea, 2021-April-01
// ---------------------------------------
// ESP32 MPU QUAT 2 UDP program,
// It is used to:
// 1- Read data from MPU6050 over I2C,
// 2- Calculates quaternion using Mahony filter
// 3- It streams quaternion + gyro measurments over UDP WiFi.

// Tests:
// ------
// Tested successfully on {ESP32_DevKitc_v4}

// Connections:
// ===============
// Connect GPIO22 of ESP32 to SCL of MPU6050
// Connect GPIO21 of ESP32 to SDA of MPU6050

// IN A NUTSHELL:
// ===============
// This program receives MPU6050 data over I2C, then it stream it over UDP WiFi

//-- Libraries Included --------------------------------------------------------------
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
//------------------------------------------------------------------------------------
// UDP network
char*       net_ssid = "MPU2UDP";              // WIFI NAME
char*       net_pass = "";                                  // PASSWORD
// UDP parameters & buffer
WiFiUDP Udp;
unsigned int localPort = 8888;      // local port to listen on
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,

//  Bytes of the Static IP for ESP
#define IP01 192
#define IP02 167
#define IP03 4
#define IP04 1
#define IP04GateWay 2
// Distination IP and distination port
IPAddress  DestinationIP;
uint16_t  PCudpPort;
// UDP measurment message buffer
const unsigned int udpMessageLen=7*4;
byte udpMessage[udpMessageLen];
boolean IpREceived = false;
//====================================================================================
// MPU Variables
#define MPUaddress (0b1101000)
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain
//---------------------------------------------------------------------------------------------------
// Mahony Variable definitions
float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Measurments
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ; // G force

long gyroX, gyroY, gyroZ;
long gyroX_bias, gyroY_bias, gyroZ_bias;
long gyroXCalibrated, gyroYCalibrated, gyroZCalibrated;
float rotX, rotY, rotZ; // Degree per Second
//====================================================================================
// Gyro bias variables
long t0=0;
long counter =0;
float temp;
//====================================================================================
// dt Variables
boolean firstExecution =  true;
unsigned long tPrevious;
unsigned long tNow;
float dt;
//====================================================================================
// SDA, SCL GPIO pins for the ESP32
#define I2C_SDA 21
#define I2C_SCL 22
//====================================================================================    
void setup() {
    Serial.begin(115200);
    // Setting Wifi Access Point
    SetWifi(net_ssid, net_pass);
    // Setup MPU
    Wire.begin(I2C_SDA,I2C_SCL,400000);
    setupMPU();
    delay(500);
    
    t0=millis();
    gyroX_bias=0;gyroY_bias=0;gyroZ_bias=0;
    // calculate the bias in the gyro measurments
    delay(1000);
    calibrateGyroForBias();
}

void calibrateGyroForBias()
{
  long gyroX_sum,gyroY_sum,gyroZ_sum;
  gyroX_sum=0;gyroY_sum=0;gyroZ_sum=0;
  long daCount=0;
  long tStart=millis();
  while((millis()-tStart)<1000)
  {
    recordAccelRegisters();
    recordGyroRegisters();
    daCount++;
    gyroX_sum=gyroX_sum+gyroX;
    gyroY_sum=gyroY_sum+gyroY;
    gyroZ_sum=gyroZ_sum+gyroZ;
  }
  gyroX_bias=gyroX_sum/daCount;
  gyroY_bias=gyroY_sum/daCount;
  gyroZ_bias=gyroZ_sum/daCount;
  Serial.print("gyroX_bias:  ");
  Serial.println(gyroX_bias);
  Serial.print("gyroY_bias:  ");
  Serial.println(gyroY_bias);
  Serial.print("gyroZ_bias:  ");
  Serial.println(gyroZ_bias);
}

void calibrateBias()
{
  gyroXCalibrated=gyroX - gyroX_bias;
  gyroYCalibrated=gyroY - gyroY_bias;
  gyroZCalibrated=gyroZ - gyroZ_bias;
}
//====================================================================================
void dispAuthorsMessage()
{
  Serial.println("");
  Serial.println("Copyright: Mohammad Safeea, 2020-Dec-04");
  Serial.println("This is a program used to relay data received over I2C into UDP WiFi");
  Serial.println("");
  Serial.println("You sahll connect over wifi from PC to the network:");
  Serial.println("<< MPU2UDP >>");
  Serial.println("....................");
}
//====================================================================================

void SetWifi(char* Name, char* Password)
{
  dispAuthorsMessage();
  // Stop Any Previous WIFI
  WiFi.disconnect();

  // Setting The Wifi Mode
  WiFi.mode(WIFI_AP_STA);
  Serial.println("WIFI Mode : AccessPoint Station");
  
  // Setting The AccessPoint Name & Password
  net_ssid      = Name;
  net_pass  = Password;

  // Set fixed IP
  /*
  IPAddress ip(192,168,4,1);   
  IPAddress gateway(192,168,4,2);
  */
  IPAddress ip(IP01,IP02,IP03,IP04);   
  IPAddress gateway(IP01,IP02,IP03,IP04GateWay);   
  IPAddress subnet(255,255,255,0);   
  WiFi.softAPConfig(ip, gateway, subnet);
  
  // Starting The Access Point
  WiFi.softAP(net_ssid, net_pass);
  Serial.println("WIFI << " + String(net_ssid) + " >> has Started");
  
  // Wait For Few Seconds
  delay(2000);
  
  // Printing The Server IP Address
  Serial.print("Server IP : ");
  Serial.println(ip);
  Serial.print("Recieving port : ");
  Serial.println(localPort);
  
  // Begin UDP
  Udp.begin(localPort);

  Serial.println("Setup complete.");
}
//====================================================================================

void setupMPU(){
  Wire.beginTransmission(MPUaddress); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}
//====================================================================================
  
void loop() {
   while(1)
   {
      calculate_dt();
      recordAccelRegisters();
      recordGyroRegisters();
      calibrateBias();
      float wx,wy,wz;
      wx=rotX*3.14/180;
      wy=rotY*3.14/180;
      wz=rotZ*3.14/180;
      MahonyAHRSupdateIMU(wx, wy, wz, gForceX, gForceY, gForceZ);
      // printData();
      //streamOverUDP();
      // Check of client IP message
      if(IpREceived == false)
      {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
          DestinationIP = Udp.remoteIP();
          PCudpPort = Udp.remotePort();
          IpREceived = true;
          Serial.println("Remote IP is:");
          Serial.println(Udp.remoteIP().toString().c_str());
          Serial.println("Remote Port:");
          Serial.println(Udp.remotePort());
         }
      }
      else
      {
        printQuat();
        streamQuatGyroOverUDP();
      }     
      delay(5);
   }
}

void printQuat()
{
  Serial.print(q0);
  Serial.print("_");
  Serial.print(q1);
  Serial.print("_");
  Serial.print(q2);
  Serial.print("_");
  Serial.println(q3);
}

void streamQuatGyroOverUDP()
{
  int byteCount = 0;
  byteCount = serializeData2udpMessage(byteCount,q0);
  byteCount = serializeData2udpMessage(byteCount,q1);
  byteCount = serializeData2udpMessage(byteCount,q2);
  byteCount = serializeData2udpMessage(byteCount,q3);
  byteCount = serializeData2udpMessage(byteCount,rotX);
  byteCount = serializeData2udpMessage(byteCount,rotY);
  byteCount = serializeData2udpMessage(byteCount,rotZ);
  
  Udp.beginPacket(DestinationIP,PCudpPort);
  Udp.write(udpMessage,udpMessageLen);
  Udp.endPacket();

}

void streamOverUDP()
{
  int byteCount = 0;
  byteCount = serializeData2udpMessage(byteCount,gForceX);
  byteCount = serializeData2udpMessage(byteCount,gForceY);
  byteCount = serializeData2udpMessage(byteCount,gForceZ);
  byteCount = serializeData2udpMessage(byteCount,rotX);
  byteCount = serializeData2udpMessage(byteCount,rotY);
  byteCount = serializeData2udpMessage(byteCount,rotZ);
  byteCount = serializeData2udpMessage(byteCount,dt);
  
  Udp.beginPacket(DestinationIP,PCudpPort);
  Udp.write(udpMessage,udpMessageLen);
  Udp.endPacket();

}
//====================================================================================
int serializeData2udpMessage(int index,float x)
{
  byte* pointer;
  pointer = (byte*) &x;
  for(int i=3;i>-1;i--)
  {
    udpMessage[index] = pointer[i];
    index = index + 1;
  }
  return index;
}
//====================================================================================
void recordAccelRegisters() {
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(MPUaddress,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelX=checkOverFlow(accelX);
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelY=checkOverFlow(accelY);
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  accelZ=checkOverFlow(accelZ);
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(MPUaddress); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(MPUaddress,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroX=checkOverFlow(gyroX);
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroY=checkOverFlow(gyroY);
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  gyroZ=checkOverFlow(gyroZ);
  // The following call does not have an effect during the Gyro bias calculation phase
  processGyroData();
}

long checkOverFlow(long x)
{
  long y;
  if(x>32767)
  {
    y=x-65536;
  }
  else
  {
    y=x;
  }
  return y;
}

void processGyroData() {
  rotX = gyroXCalibrated / 131.0;
  rotY = gyroYCalibrated / 131.0; 
  rotZ = gyroZCalibrated / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.print(" dt=");
  Serial.println(dt);
}

//====================================================================================
// Calculate dt
float calculate_dt()
{
  if(firstExecution==true)
  {
    tPrevious=micros();
    tNow=tPrevious;
    firstExecution=false;
  }
  else
  {
    tNow=micros();
  }
  dt= (tNow-tPrevious)/1000000.0;
  tPrevious=tNow;
}
//====================================================================================
// Filter
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    temp=ax * ax + ay * ay + az * az;
    recipNorm = invSqrt(temp);
    ax = ax * recipNorm;
    ay = ay * recipNorm;
    az = az * recipNorm;        

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx = integralFBx + twoKi * halfex * (dt);  // integral error scaled by Ki
      integralFBy = integralFBy + twoKi * halfey * (dt);
      integralFBz = integralFBz + twoKi * halfez * (dt);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx = gx + twoKp * halfex;
    gy = gy + twoKp * halfey;
    gz = gz + twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx =  gx * (0.5f * dt);   // pre-multiply common factors
  gy = gy * (0.5f * dt);
  gz = gz * (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 = q0 + (-qb * gx - qc * gy - q3 * gz);
  q1 = q1 + (qa * gx + qc * gz - q3 * gy);
  q2 = q2 + (qa * gy - qb * gz + q3 * gx);
  q3 = q3 + (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  temp=q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  recipNorm = invSqrt(temp);
  q0 = q0 * recipNorm;
  q1 = q1 * recipNorm;
  q2 = q2 * recipNorm;
  q3 = q3 * recipNorm;
}


float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
