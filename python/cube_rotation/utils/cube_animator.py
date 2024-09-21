import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import math

def quaternion2matrix(q):
    norm = np.linalg.norm(q)
    quaternion = np.array(q) / norm
    qw, qx, qy, qz = quaternion
    mat = [
        [1 - 2*(qy*qy + qz*qz), 2*qx*qy - 2*qw*qz, 2*qw*qy + 2*qx*qz],
        [2*qx*qy + 2*qw*qz, 1 - 2*(qx*qx + qz*qz), 2*qy*qz - 2*qw*qx],
        [2*qx*qz - 2*qw*qy, 2*qw*qx + 2*qy*qz, 1 - 2*(qx*qx + qy*qy)]
    ]
    return np.array(mat, dtype=np.float64)

def rotz(angle_degree):
    theta = np.pi * (angle_degree / 180.0) / 2.0
    q = [math.cos(theta), 0., 0., math.sin(theta)]  # Quaternion for Z-axis rotation
    return q

class Cube():
    def __init__(self, render_rate_hz):
        # Define cube vertices (8 points)
        self.cube_vertices = np.array([[-1, -1, -1],
                                       [ 1, -1, -1],
                                       [ 1,  1, -1],
                                       [-1,  1, -1],
                                       [-1, -1,  1],
                                       [ 1, -1,  1],
                                       [ 1,  1,  1],
                                       [-1,  1,  1]])
        # Define the 6 faces of the cube (each face is a list of 4 vertices)
        self.cube_faces = [[0, 1, 2, 3],
                           [4, 5, 6, 7],
                           [0, 1, 5, 4],
                           [2, 3, 7, 6],
                           [0, 3, 7, 4],
                           [1, 2, 6, 5]]
        # Initialize the plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.last_time = None
        self.render_rate_hz = render_rate_hz
        self.adaptive_render_time_sec = 1 / render_rate_hz

    def get_rotate_cube(self, R):
        # Rotate vertices
        rotated_vertices = np.dot(self.cube_vertices, R.T)  # Rotate each vertex by R
        # Create a 3D polyhedron for the cube using the rotated vertices and faces
        poly3d = [[rotated_vertices[vert] for vert in face] for face in self.cube_faces]
        return poly3d

    def render_cube(self, R):
        self.ax.clear()  # Clear previous frame
        poly3d = self.get_rotate_cube(R)
        # Plot the cube
        self.ax.add_collection3d(Poly3DCollection(poly3d, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))
        # Set plot limits for better visualization
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        # Set labels for axes
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.draw()
        plt.pause(self.adaptive_render_time_sec)
        if self.last_time == None:
            self.last_time = time.time()
        else:
            dt = time.time() - self.last_time
            hz = 1 / dt
            print("Refresh rate is: ", hz)
            self.last_time = time.time()
