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

import config
import rospy
import time

from csv_line_publisher import LinePublisher
from line_controller import LineController

if __name__=='__main__':
    rospy.init_node('example_line_follower', anonymous=True)

    rate = rospy.Rate(config.UPDATE_FREQ)
    waypoint_file = config.WAYPOINTS_FILE
    pose_topic = config.POSE_TOPIC
    line_topic = config.LINE_TOPIC


    # parse the csv file to a list of tuples
    with open(waypoint_file, 'r') as fin:
        wps = [tuple(map(lambda a: float(a.strip()), line.split(','))) for line in fin.readlines()]

    # line planner acts like an external planning node and publishes a line
    # for the control to follow
    lp = LinePublisher(waypoints = wps,
                       pose_topic = pose_topic,
                       line_topic = line_topic)

    # controller subs to a line topic and follows that line using PID for pitch/yaw
    # LoLo rolls when yaw'ing and we can do nothing about it now. (16/02)
    lc = LineController(line_topic = line_topic,
                        pose_topic = pose_topic,
                        no_pitch=True)

    t1 = time.time()
    while not rospy.is_shutdown():
        dt = time.time()-t1
        lp.update()
        lc.update(dt)
        t1 = time.time()
        rate.sleep()
