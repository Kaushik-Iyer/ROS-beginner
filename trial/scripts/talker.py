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

from ctypes.wintypes import MSG
from lib2to3.pgen2.token import NUMBER
import numbers

from tokenize import Number
import rospy
import numpy
import matplotlib.pyplot as plt
import random

import json
from typing import List, Any
from std_msgs.msg import String
from trial.msg import value


def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', value, queue_size=10)
    f = open('/home/kaushik/catkin_ws/src/trial/scripts/data_2.json','r+')
    file = json.load(f)
    # a_list: list[Any] = []
    # b_list = []
    # a_list2 = []
    # b_list2 = []
    for i in file['cones']:
        Value=value()
        Value.x= int(i['x'])
        Value.y=int(i['y'])
        Value.color.data=i['color']
        rospy.loginfo(Value)
        pub.publish(Value)
        # number = random.randint(-1, 1)
        # i['x'] = i['x'] + number
        # if (k % 2 == 0):
        #     a_list.append(i['x'])
        # else:

        #     a_list2.append(i['x'])

        # number2 = random.randint(-1, 1)
        # i['y'] = i['y'] + number2
        # if (k % 2 == 0):
        #     b_list.append(i['y'])
        # else:
        #     b_list2.append(i['y'])
  
    # plt.plot(a_list, b_list, 'o')
    # plt.plot(a_list2, b_list2, 'o')
    # plt.show()
    
    
    f.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
