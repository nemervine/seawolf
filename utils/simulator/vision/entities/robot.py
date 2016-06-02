
from __future__ import division
from math import radians, degrees, sin, cos, pi, atan2, tan
import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
#from utils import matrix44, vector3
import math

import seawolf

from base import Entity
from doublepath import DoublePathEntity
import model

"""
def mix_rotate(ent, prev_rot, add_frc, sub_frc, var_name):
    new_rot = prev_rot + (add_frc-sub_frc) * 
    yaw = yaw + (port - star) * self.YAW_CONSTANT * dt
    yaw = (yaw + 180) % 360 - 180  # Range -180 to 180
    seawolf.var.set("SEA.Yaw", yaw)
    seawolf.notify.send("UPDATED", "IMU")
"""

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    theta = np.asarray(theta)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def get_rpy_rotation_matrix(norm_x, norm_y, roll, pitch, yaw):
    # definitions
    local_z_axis = np.cross(norm_x,norm_y)
    print("local z = %s" % local_z_axis)
    y_axis = [0,1,0]
    x_axis = [1,0,0]
    # rotation vectors
    roll *= math.pi/180.0
    pitch *= math.pi/180.0
    yaw *= math.pi/180.0
    print("")
    print("Roll %.2f, Pitch %.2f, Yaw %.2f input" % (roll,pitch,yaw) )
    
    # heading
    heading = np.array(norm_x).transpose()
    print("Defaut heading %s" % heading)
    
    # GET RZ
    rz = rotation_matrix(local_z_axis, yaw)
    print("yaw rotation mat applied to heading %s" % rz)
    
    # [heading] * [r_local_z]
    heading = heading.dot(rz)
    print ("heading after yaw = %s" % heading)
    
    # [heading] * [old r_local_z] * [new r_local_y]
    local_y_axis = np.cross(local_z_axis,heading)
    ry = rotation_matrix( local_y_axis , pitch)
    heading = heading.dot(ry)
    print ("heading after pitch = %s" % heading)
    
    # [heading] * [old r_local_z] * [old l_local_y] * [new r_local_x]
    local_x_axis = heading
    rx = rotation_matrix( local_x_axis, roll )
    heading = heading.dot(rx)
    print ("heading after roll = %s" % heading)
    
    
    # multiply
    prod1 = rz.dot(ry)
    final_rotation = prod1.dot(rx)
    
    return final_rotation
    
    
def rotation_2_euler(rot_mat):
    r11, r12, r13 = rot_mat[0,:]
    r21, r22, r23 = rot_mat[1,:]
    r31, r32, r33 = rot_mat[2,:]
    
    eulerZ = math.atan2(r21,r11)    * 180/np.pi
    eulerY = math.atan2(-r31, math.sqrt(r32**2 + r33**2))   * 180/np.pi
    eulerX = math.atan2(r32, r33)   * 180/np.pi
    
    return (eulerX, eulerY, eulerZ)
    

class RobotEntity(Entity):

    DEPTH_CONSTANT = 1.5
    VELOCITY_CONSTANT = 1
    YAW_CONSTANT = 40

    def __init__(self, *args, **kwargs):
        super(RobotEntity, self).__init__(*args, **kwargs)

        self.yaw_offset = 90

        #self.model = model.ObjModel(file("models/seawolf5.obj"))
        self.model = model.StlModel("models/seawolf6.stl", ambient=(1, 0, 0, 0), diffuse=(1, 0, 0, 0))

        self.depth = -1 * self.pos[2]
        self.tracked_vars = {}

        seawolf.var.subscribe("Port")
        seawolf.var.subscribe("Star")
        seawolf.var.subscribe("Bow")
        seawolf.var.subscribe("Stern")
        seawolf.var.subscribe("StrafeT")
        seawolf.var.subscribe("StrafeB")
        seawolf.var.subscribe("SEA.Roll")
        seawolf.var.subscribe("SEA.Pitch")
        seawolf.var.subscribe("SEA.Yaw")
        
        self.normal_x = np.array([1,0,0])
        self.normal_y = np.array([0,1,0])
        
        #self.rot_delta = vector3.Vector3()
        #self.trans_delta = vector3.Vector3()
        
        #self.rotation = matrix44.Matrix44()
        #self.translation = matrix44.Matrix44()

    def get_var(self, name):
        if seawolf.var.stale(name) or name not in self.tracked_vars:
            value = seawolf.var.get(name)
            self.tracked_vars[name] = value
            return value
        else:
            return self.tracked_vars[name]

    def set_rotation(self, velocity_roll, velocity_pitch, velocity_yaw):
        normals = ()
        Rotation_Mat_dt = get_rpy_rotation_matrix( self.normal_x, self.normal_y, 
                                              velocity_roll, velocity_pitch, velocity_yaw)
                                              
        # gener
        (Rx, Ry, Rz) = rotation_2_euler( Rotation_Mat_dt )
        
        # save new normal values
        
        self.normal_x = self.normal_x.dot(Rotation_Mat_dt)
        self.normal_y = self.normal_y.dot(Rotation_Mat_dt)
        self.roll += Rx
        self.pitch += -Ry
        self.yaw += Rz
    
    def step(self, dt):
    
        # Capture robot's current position and rotation
        #self.translation.set(self.pos[0], self.pos[1] , self.pos[2], 1.0)
        #self.rotation.set(self.roll, -self.pitch, -self.yaw, 0.0)

        # Thrusters
        port = self.get_var("Port")
        star = self.get_var("Star")
        bow = self.get_var("Bow")
        stern = self.get_var("Stern")
        strafeT = self.get_var("StrafeT")
        strafeB = self.get_var("StrafeB")
        
        # Orthongonal Velocity
        velocity_fw     = (port + star) * self.VELOCITY_CONSTANT * dt
        velocity_stf    = (strafeT - strafeB) * self.VELOCITY_CONSTANT * dt
        velocity_dph    = (bow + stern) * self.DEPTH_CONSTANT * dt
        
        # Angular Velocity
        velocity_yaw = (port - star) * self.YAW_CONSTANT * dt
        velocity_pitch = (stern - bow) * self.YAW_CONSTANT * dt
        velocity_roll = (strafeT + strafeB) * self.YAW_CONSTANT * dt
        
        """
        yaw = (yaw + 180) % 360 - 180  # Range -180 to 180
        seawolf.var.set("SEA.Yaw", yaw)
        seawolf.notify.send("UPDATED", "IMU")
        """
        """
        # apply to vectors
        self.trans_delta.set(velocity_fw, 
                            -velocity_stf, 
                            -velocity_dph)
                            
        self.rot_delta.set(velocity_roll,
                            -velocity_pitch,
                            -velocity_yaw)
                            
        # apply transformations
        self.translation.translate()
        """
        
        # show forward unit vect
        # generate rotation matrix for yaw
          # have rotation matrix for new forward direction
        # generate rotation matrix for pitch
          # have a rotation I can mult with new forward direction to show new downward heading
        # generate a z unit vector.
        # apply yaw and yawed-pitch rotations to the z unit vector
          # have a 
          
        # mult ti
        # multiply fuv with yaw angle
        # multiply fuv with pitch angle
        
        # reflect that you not have a vector for
        
        self.set_rotation(velocity_roll, velocity_pitch, velocity_yaw)
        
        
        
        # IMU
        yaw = -self.yaw
        pitch = self.pitch #IMU convention: pitch --- Simulator convention: self.pitch
        
        

        # Depth
        self.depth      = -self.pos[2]
        self.depth      += dt * velocity_dph
        if self.depth < 0:
            self.depth  += 1.0 * dt
        self.pos[2]     = -1 * self.depth
        seawolf.var.set("Depth", self.depth)

        # Position
        self.pos[0] += dt * (cos(radians(-self.yaw)) * sin(radians(pitch + 90)) * velocity_fw)
                           #+ sin(radians(-self.yaw)) * sin(radians(self.roll  + 90)) * velocity_stf
                           #+ cos(radians(-self.yaw)) * sin(radians(pitch))      * velocity_dph)
                           
        self.pos[1] += dt * (sin(radians(-self.yaw)) * sin(radians(pitch + 90)) * velocity_fw)
                           #+ -cos(radians(-self.yaw)) * sin(radians(self.roll  + 90)) * velocity_stf
                           #+ sin(radians(-self.yaw)) * sin(radians(pitch))      * velocity_dph)


        # Yaw
        
        
        # transformations
        #self.yaw = -yaw
        
        """
        # Pitch
        pitch = pitch + (stern - bow) * self.YAW_CONSTANT * dt
        pitch = (pitch + 180) % 360 - 180  # Range -180 to 180
        seawolf.var.set("SEA.Pitch", pitch)
        self.pitch = -pitch
        seawolf.notify.send("UPDATED", "IMU")
        
        # Roll
        self.roll = self.roll + (strafeT + strafeB) * self.YAW_CONSTANT * dt
        self.roll = (self.roll + 180) % 360 - 180  # Range -180 to 180
        seawolf.var.set("SEA.Roll", self.roll)
        seawolf.notify.send("UPDATED", "IMU")
        """

        
    def draw(self):
        self.pre_draw()

        # Account for model offsets and draw the model
        glPushMatrix()
        glRotate(self.yaw_offset, 0, 0, -1)
        self.model.draw()
        glPopMatrix()

        # Down camera guides
        glColor(1, 1, 0, 0.2)
        glPushMatrix()
        glTranslate(0.0, 0, -0.60)
        glRotate(90, 0, 1, 0)
        self.draw_camera_guides(
            self.get_camera_fov("down", vertical=False),
            self.get_camera_fov("down", vertical=True)
        )
        glPopMatrix()

        # Forward camera guides
        glColor(0, 1, 0, 0.2)
        glTranslate(1.1, 0, 0)
        self.draw_camera_guides(
            self.get_camera_fov("forward", vertical=False),
            self.get_camera_fov("forward", vertical=True)
        )

        self.post_draw()

    def draw_camera_guides(self, horizontal_fov, vertical_fov, box_dist=4):

        half_horizontal_fov = radians(horizontal_fov) / 2
        half_vertical_fov = radians(vertical_fov) / 2

        right = tan(half_horizontal_fov) * box_dist
        left = -right
        top = tan(half_vertical_fov) * box_dist
        bottom = -top

        # Draw box
        glBegin(GL_LINE_LOOP)
        glVertex(box_dist, left, top)
        glVertex(box_dist, right, top)
        glVertex(box_dist, right, bottom)
        glVertex(box_dist, left, bottom)
        glEnd()

        glBegin(GL_LINES)

        # Middle line
        glVertex(0, 0, 0)
        glVertex(box_dist, 0, 0)

        # Lines from box to camera
        glVertex(0, 0, 0)
        glVertex(box_dist, left, top)
        glVertex(0, 0, 0)
        glVertex(box_dist, right, top)
        glVertex(0, 0, 0)
        glVertex(box_dist, right, bottom)
        glVertex(0, 0, 0)
        glVertex(box_dist, left, bottom)

        glEnd()

        # Fill in the fruscum
        # Enable blending for transparency effect.  Disable writing to depth
        # buffer with glDepthMask so objects that are behind this will still be
        # drawn.
        blend_setting = glGetBooleanv(GL_BLEND)
        glEnable(GL_BLEND)
        glDepthMask(False)
        glBegin(GL_TRIANGLE_FAN)
        glVertex(0, 0, 0)
        glVertex(box_dist, left, top)
        glVertex(box_dist, right, top)
        glVertex(box_dist, right, bottom)
        glVertex(box_dist, left, bottom)
        glVertex(box_dist, left, top)
        glEnd()
        glDepthMask(True)
        if not blend_setting:
            glDisable(GL_BLEND)

    def find_entity(self, entity_cls):

        if entity_cls == DoublePathEntity:
            return DoublePathEntity.find_paths(self, self.simulator.entities)

        data = None
        entity_class_found = False
        for entity in self.simulator.entities:
            if entity.__class__.__name__ == entity_cls.__name__:

                entity_class_found = True
                found, data = entity.find(self)
                if found:
                    break

        if not entity_class_found:
            print "Warning: Entity %s is being searched for, but none exists in the simulator." % entity_cls

        return data

    def find_point(self, camera, point):
        '''Finds a point viewed from the given camera.

        All angles that are returned are expressed as a number from -1 to 1.  0
        is straight ahead, -1 is maximum fov to the left or down, 1 is maximum
        fov to the right or up.

        '''

        # Convert to homogeneous coordinates (column matrix)
        if len(point) == 3:
            dimmensions = 3
            point = np.matrix([point[0], point[1], point[2], 1]).transpose()
        elif len(point) == 2:
            dimmensions = 2
            point = np.matrix([point[0], point[1], 0, 1]).transpose()
        else:
            raise ValueError("point must be length 2 or 3.")

        camera_transform = self.get_camera_matrix(camera)
        new_point = np.dot(camera_transform, point)

        # Convert from a column matrix to array
        new_point = np.array(new_point.transpose())[0]

        # OpenGL camera is at the origin pointing down the -Z axis.  Using
        # arctargent we can get the spherical angles of the point.
        theta = degrees(atan2(new_point[0], -new_point[2]))
        phi = degrees(atan2(new_point[1], -new_point[2]))

        # Scale from -fov to fov
        half_horizontal_fov = self.get_camera_fov(camera, vertical=False) / 2
        half_vertical_fov = self.get_camera_fov(camera, vertical=True) / 2
        theta = theta / half_horizontal_fov
        phi = phi / half_vertical_fov

        # Make None if outside fov
        if abs(theta) > 1:
            theta = None
        if abs(phi) > 1:
            phi = None

        if dimmensions == 2:
            return theta
        else:
            return theta, phi

    def get_camera_fov(self, camera, vertical=False):
        # All cameras are currently the same
        if vertical:
            return 38  # TODO: We've never measured this, so this is a guess
        else:
            return 42

    def get_camera_matrix(self, camera):
        '''Gets the modelview matrix for the given camera.'''
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        self.camera_transform(camera)
        # Transpose to format into row major order
        modelview = glGetDouble(GL_MODELVIEW_MATRIX).transpose()
        glPopMatrix()

        return modelview

    def camera_transform(self, camera):

        if camera == "forward":
            ref_point = self.absolute_point((1.6, 0, 0))
            cam_point = self.absolute_point((1.1, 0, 0))
            up = self.absolute_point((0, 0, 1)) - self.pos

        elif camera == "down":
            cam_point = self.absolute_point((0, 0, -0.6))
            ref_point = self.absolute_point((0, 0, -1.6))
            up = self.absolute_point((1, 0, 0)) - self.pos

        else:
            raise ValueError('Unknown camera: "%s"' % camera)

        gluLookAt(
            cam_point[0], cam_point[1], cam_point[2],
            ref_point[0], ref_point[1], ref_point[2],
            up[0], up[1], up[2])
