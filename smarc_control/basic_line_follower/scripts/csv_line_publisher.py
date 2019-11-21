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

import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from utils import geometry as G

import config

class LinePublisher:
    def __init__(self,
                 waypoints,
                 pose_topic,
                 line_topic):
        """
        a very simple line follower 'planner'.
        here just as a scaffolding that can publish Path messages we agreed on before
        """

        self._wps = waypoints
        self._current_line = [self._wps[0], self._wps[1]]

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        # give out a line for the controller to follow
        self.publisher = rospy.Publisher(line_topic, Path, queue_size=10)

        self._frame_id = 'world'


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

        #  self._frame_id = data.header.frame_id

    def update(self):
        xy_dist = G.euclid_distance(self.pos[:2], self._current_line[1][:2])
        z_dist = np.abs(self.pos[2] - self._current_line[1][2])
        if xy_dist <= config.LINE_END_XY_THRESHOLD and z_dist <= config.LINE_END_Z_THRESHOLD:
            config.pprint('LINE CHANGED')
            # reached the target, request to follow the next line
            self._wps = self._wps[1:]
            try:
                self._current_line = [self._wps[0], self._wps[1]]
            except IndexError:
                config.pprint('Out of waypoints!')
                sys.exit()

        # create a Path message with whatever frame we received the localisation in
        path = Path()
        path.header.frame_id = self._frame_id

        # Path.poses is a PoseStamped list
        # so we have to create these objects
        ps1 = PoseStamped()
        ps1.header.frame_id = self._frame_id
        ps1.pose.position.x = self._current_line[0][0]
        ps1.pose.position.y = self._current_line[0][1]
        ps1.pose.position.z = self._current_line[0][2]
        path.poses.append(ps1)

        ps2 = PoseStamped()
        ps2.header.frame_id = self._frame_id
        ps2.pose.position.x = self._current_line[1][0]
        ps2.pose.position.y = self._current_line[1][1]
        ps2.pose.position.z = self._current_line[1][2]
        path.poses.append(ps2)

        self.publisher.publish(path)
