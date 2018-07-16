#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


DIST_CORR  = 52

import rospy
import serial
import time
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('sensor_readings', Float64, queue_size=1)
    rospy.init_node('wenglor_node', anonymous=True)
    rate = rospy.Rate(400) # 10hz
    ser  =serial.Serial('/dev/ttyUSB0', 38400)
    print(ser.name)
    ser.write('/000R4D.')
    time.sleep(0.5)
    ser.write('/020D0059.')
    seq = []
    count = 1

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()

        for c in ser.read():
	    #print(c)
            seq.append(c) #convert from ANSII
            joined_seq = ''.join(str(v) for v in seq) #Make a string from array

            if c == '.':
                #print("Line " + str(count) + ': ' + joined_seq[8:14] + "Time : " + str(rospy.get_time()))
                seq = []
                ser.write('/020D0059.')
                time.sleep(0.005)
                count += 1
                dist_str = joined_seq[8:11]
                if(dist_str.isdigit()):
                    dist = float( int(dist_str) + DIST_CORR )/1000
                    rospy.loginfo(dist)
                    pub.publish(dist)
                break

	rate.sleep()

    ser.write('/020D0a08.')
    time.sleep(3)
    ser.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
