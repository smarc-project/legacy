#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright 2018 Ozer Ozkahraman (ozkahramanozer@gmail.com)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np

import time
import sys
import rospy

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from utils import geometry as G
from utils import Pid

import config


class LoloPublisher:
    def __init__(self, frame_id='lolo_auv_1_map'):
        """
        a simple class to keep information about lolos fins.
        publishes coordinated fin movements when move_xxx methods are called
        # fin 0 -> vertical top right, + = right
        # fin 1 -> vertback top left,  + = right
        # fin 2 -> vertback bottom, right, + = left
        # fin 3 -> vertback bottom, left, + = left
        # fin 4,5 -> dont do anything
        # back_fins/0 -> horizontal, + = down
        """
        self.fin0pub = rospy.Publisher(config.LOLO_FIN0_INPUT, FloatStamped, queue_size=1)
        self.fin1pub = rospy.Publisher(config.LOLO_FIN1_INPUT, FloatStamped, queue_size=1)
        self.fin2pub = rospy.Publisher(config.LOLO_FIN2_INPUT, FloatStamped, queue_size=1)
        self.fin3pub = rospy.Publisher(config.LOLO_FIN3_INPUT, FloatStamped, queue_size=1)
        self.backfinspub = rospy.Publisher(config.LOLO_BACKFIN_INPUT, FloatStamped, queue_size=1)

        self.frame_id = frame_id



    def yaw(self, direction, frame_id='lolo_auv_1_map'):
        """
        + = move left
        """
        #  direction = np.sign(direction)

        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = 1*direction

        self.fin0pub.publish(out)
        self.fin1pub.publish(out)

        # the control for these fins are inverted for some reason
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction * -1

        self.fin2pub.publish(out)
        self.fin3pub.publish(out)

        #  if np.sign(out.data)== -1:
            #  config.pprint('>>>',out.data)
        #  elif np.sign(out.data)== 1:
            #  config.pprint('<<<',out.data)
        #  else:
            #  config.pprint('---',out.data)

    def pitch(self,direction, frame_id='lolo_auv_1_map'):
        """
        + = move up
        """
        #  direction = np.sign(direction)
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = -1 * direction

        self.backfinspub.publish(out)
        #  if np.sign(out.data)==-1:
            #  config.pprint('^^^',out.data)
        #  elif np.sign(out.data)==1:
            #  config.pprint('vvv',out.data)
        #  else:
            #  config.pprint('---',out.data)


class LineController:
    def __init__(self, line_topic, pose_topic, following_curve = True, no_pitch = False):

        # debugging 'current line' publisher
        self.debug_line_pub = rospy.Publisher(config.DEBUG_LINE_TOPIC, Path, queue_size=1)

        if following_curve:
            rospy.Subscriber(line_topic, Path, self.update_curve)
        else:
            # receive the line requests from ros
            rospy.Subscriber(line_topic, Path, self.update_line)

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        self._current_line = None
        self._frame_id = 'lolo_auv_1_map'

        self._yaw_pid = Pid.PID(*config.LOLO_YAW_PID)
        self._pitch_pid = Pid.PID(*config.LOLO_PITCH_PID)

        # create a publisher for LOLO
        self._lolopub = LoloPublisher()

        # we need to change from yz to xz if the two points have the same 
        # y,z values, this leads to nans in distance calculations!
        self._using_yz_for_pitch=True

        # disable pitch control
        self._no_pitch = no_pitch

        # distance from seabed to keep
        self._target_z = None

    def update_pose(self, data):
        datapos = data.pose.pose.position
        x = datapos.x
        y = datapos.y
        z = datapos.z

        self.pos = [x,y,z]

        dataori = data.pose.pose.orientation
        x = dataori.x
        y = dataori.y
        z = dataori.z
        w = dataori.w

        self.ori = [x,y,z,w]

        # self._frame_id = data.header.frame_id

        if self._target_z is None:
            self._target_z = z + config.Z_BUFFER
            # config.pprint('set target z to:',self._target_z)


    def update_line(self, data):
        # data should contain Path. which has a 'poses' field
        # which contains a list of "PoseStamped"
        try:
            p1 = data.poses[0].pose.position
            p2 = data.poses[1].pose.position
        except IndexError:
            config.pprint('No line')
            return

        # extract the  x,y,z values from the PoseStamped things
        l1 = (p1.x, p1.y, p1.z)
        l2 = (p2.x, p2.y, p2.z)

        # we finally have a line
        self._current_line = (l1, l2)

        # also extract the frame id, since we will need it when publishing control signals
        # self._frame_id = data.header.frame_id

    def update_curve(self, data):
        # data should contain Path, with multiple,  that represent a discretized curve
        # we will only use the x,y component for now

        points = []
        for p in data.poses:
            points.append(p.pose.position)

        if len(points) < 1:
            print('No curve received')
            return

        line = None
        # the segment intersects a circle of radius r if
        # the first point is closer than r and the second is further
        # we also want the 'last' one that intersects, not the first
        # that particular segment is 'forward'.
        # p1 inside, p2 outside should not happen for more than 1 point
        selfpos = self.pos[:2]
        for i in range(len(points)-1, 0, -1):
            p1 = (points[i-1].x,points[i-1].y,points[i-1].z)
            p2 = (points[i].x,points[i].y,points[i].z)

            p1d = G.euclid_distance(selfpos, p1[:2])
            p2d = G.euclid_distance(selfpos, p2[:2])
            #print(p1d, p2d)
            if p1d > config.LOOK_AHEAD_R:
                print('p1d:',p1d,'p2d:',p2d)
                # the first point is inside, check the second one
                if p2d < config.LOOK_AHEAD_R:
                    # we are intersecting!
                    line = (p1,p2)

        if line is None:
            print('No segment in range, using first segment')
            p1 = (points[0].x,points[0].y,points[0].z)
            p2 = (points[1].x,points[1].y,points[1].z)
            line = (p1,p2)

        p1,p2 = line
        p1 = list(p1)
        #p1[0] += 3
        #p1[1] += 3
        line = [p1,p2]

        # set these to be used later
        self._current_line = line
        # self._frame_id = data.header.frame_id



        # elongate the line for visualization purposes
        x1,y1,z1 = line[0]
        x2,y2,z2 = line[1]
        slope = (y2-y1)/(x2-x1)
        d = -5
        x2 += d
        y2 += d*slope
        x1 -= d
        y1 -= d*slope

        # publish the current line
        pose1 = Pose()
        pose1.position.x = x1
        pose1.position.y = y1
        pose1.position.z = z1
        stamped1 = PoseStamped()
        stamped1.pose = pose1

        pose2 = Pose()
        pose2.position.x = x2
        pose2.position.y = y2
        pose2.position.z = z2
        stamped2 = PoseStamped()
        stamped2.pose = pose2

        path = Path()
        path.poses = [stamped1,stamped2]
        path.header.frame_id = self._frame_id

        self.debug_line_pub.publish(path)



    def update(self, dt):
        if self._current_line is None:
            return
        # use a pid for yaw and another for pitch.

        # first the yaw, find the yaw error
        # just project the 3D positions to z=0 plane for the yaw control

        # cant do the same checks we did for pitch since yaw HAS to use xy plane.
        # pitch can use either xz OR yz.
        yaw_pos = np.array(self.pos[:2])
        yaw_line_p1 = np.array(self._current_line[0][:2])

        current_yaw = G.quat_to_yaw(self.ori) #% (2*np.pi)
        yaw_error = -G.directed_angle([np.cos(current_yaw), np.sin(current_yaw)], yaw_line_p1-yaw_pos)# % (2*np.pi)

        if not np.isnan(yaw_error):
            yaw_correction = self._yaw_pid.update(yaw_error, dt)
            self._lolopub.yaw(yaw_correction, self._frame_id)

        if not self._no_pitch:
            x0,y0,z0 = self.pos
            #  x1,y1,z1 = self._current_line[0]
            #  x2,y2,z2 = self._current_line[1]
#
            #  # create a plane from the current line.
            #  pa = np.array([x1,y1,z1])
            #  pb = np.array([x2,y2,z2])
            #  # put a point near the middle somewhere
            #  pc = np.array([x1+10,y1+10,(z1+z2)/2])
            #  # make a plane out of these 3 points
            #  ab = pa-pb
            #  ac = pa-pc
            #  xx = np.cross(ab,ac)
            #  d = xx[0]*pa[0] + xx[1]*pa[1] + xx[2]*pa[2]
            #  # this function returns +1 if the point is above the plane
            #  above = lambda pp: -np.sign(xx[0]*pp[0]+xx[1]*pp[1]+xx[2]*pp[2]-d)
#
            #  # this gives the magnitude of the error
            #  pitch_error = np.abs((xx[0]*x0+xx[1]*y0+xx[2]*z0+d)/np.sqrt(xx[0]**2+xx[1]**2+xx[2]**2))


            if self._target_z is not None:
                pitch_error = self._target_z - z0
                # config.pprint('current z:',z0,'_target_z:',self._target_z,'pitch_error:',pitch_error)
                # this only gives the magnitude of the error, not the 'side' of it
                pitch_correction = self._pitch_pid.update(pitch_error, dt)

                # combine side with magnitude for control
                #  control = above(self.pos)*pitch_correction
                self._lolopub.pitch(pitch_correction, self._frame_id)




if __name__=='__main__':
    import config
    import rospy
    import time
    import sys

    rospy.init_node('line_controller', anonymous=True)

    no_pitch = True
    args = sys.argv
    if len(args) > 1:
        if args[1] == 'nopitch':
            no_pitch = True

    pose_topic = config.POSE_TOPIC
    line_topic = config.LINE_TOPIC

    # controller subs to a line topic and follows that line using PID for pitch/yaw
    # LoLo rolls when yaw'ing and we can do nothing about it now. (16/02)
    lc = LineController(line_topic = line_topic,
                        pose_topic = pose_topic,
                        no_pitch = no_pitch)

    t1 = time.time()
    rate = rospy.Rate(config.UPDATE_FREQ)
    while not rospy.is_shutdown():
        dt = time.time()-t1
        lc.update(dt)
        t1 = time.time()
        rate.sleep()
