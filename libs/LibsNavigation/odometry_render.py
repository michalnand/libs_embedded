from OpenGL.GL import *
from OpenGL.GLU import *

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy

import glfw


class RenderPointCloud:

    def __init__(self, height = 256, width = 512):

        self.height = height
        self.width  = width

        glfw.init()
        self.window = glfw.create_window(self.width, self.height, "pointcloud", None,None)
        glfw.make_context_current(self.window)   


    def render(self, points):
        
        p_min = 10**9
        p_max = -p_min

        for j in range(len(points)):
            p_min = min(p_min, numpy.min(points[j]))
            p_max = max(p_max, numpy.max(points[j]))

        r_min = -2.0
        r_max = 2.0

        k = (r_max - r_min)/(p_max - p_min)
        q = r_max - k*p_max

        

        glLoadIdentity()
        gluPerspective(35, (self.width / self.height), 0.1, 50.0)


        glTranslatef(0.0, 0.0, -5)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    

        glColor3f(1.0, 1.0, 1.0)
        glBegin(GL_POINTS)


        
        for j in range(len(points)):
            
            '''
            phi = 2.0*numpy.pi*j/len(points)
            r = (numpy.cos(phi + 0.0*2.0*numpy.pi/3.0) + 1.0)/2.0
            g = (numpy.cos(phi + 1.0*2.0*numpy.pi/3.0) + 1.0)/2.0
            b = (numpy.cos(phi + 2.0*2.0*numpy.pi/3.0) + 1.0)/2.0
            '''
            

            for i in range(len(points[j])):
                x = k*points[j][i][0] + q
                y = k*points[j][i][1] + q
                z = k*points[j][i][2] + q


                r = (x - r_min)/(r_max - r_min)
                g = (y - r_min)/(r_max - r_min)
                b = (z - r_min)/(r_max - r_min)
                glColor3f(r, g, b)
                glVertex3f(x, y, -z)
        

        glEnd ()

        glfw.swap_buffers(self.window)
        glfw.poll_events()


