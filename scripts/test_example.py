#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from dynamixel_x_mcu_controller.srv import EnableActuators, EnableActuatorsResponse, EnableActuatorsRequest
from dynamixel_x_mcu_controller.srv import InitializeActuators, InitializeActuatorsResponse, InitializeActuatorsRequest
from dynamixel_x_mcu_controller.srv import DriveActuators, DriveActuatorsResponse, DriveActuatorsRequest

def InitializeActuatorsFn(req: InitializeActuatorsRequest):
    print("initialized")
    resp = InitializeActuatorsResponse()
    resp.status = 0
    return resp

def EnableActuatorsFn(req: EnableActuatorsRequest):
    if (req.enable == True):
        print("enabled")
    else:
        print("disabled")
    resp = EnableActuatorsResponse()
    resp.status = 0
    return resp


def DriveActuatorsFn(req: DriveActuatorsRequest):
    resp = DriveActuatorsResponse()
    resp.position = 0
    resp.speed = 0
    resp.effort = 0
    resp.error = 0
    return resp

if __name__ == "__main__":
    rospy.init_node('test_example_server')
    initialize = rospy.Service('InitializeActuators', InitializeActuators, InitializeActuatorsFn)
    enable = rospy.Service('EnableActuators', EnableActuators, EnableActuatorsFn)
    initialize = rospy.Service('DriveActuators', DriveActuators, DriveActuatorsFn)
    print("Ready to start")
    rospy.spin()