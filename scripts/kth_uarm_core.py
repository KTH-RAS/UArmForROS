#!/usr/bin/env python

'''

# File Name : kth_uarm_core.py
# Authors : Diogo Almeida, Joshua Haustein; based on uarm_core.py by Joey Song
# Version : V1.00
# Date : 20 Oct, 2016
# Modified Date : 20 Oct, 2016
# Description : This module contains a ROS node that provides access to the KTH uarm python wrapper.
# Copyright(C) 2016 uArm Team. All right reserved.

'''
# All libraries needed to import
# Import system library
import sys
import time
import rospy
import logging

# Import kth uarm python library
import kth_uarm

import geometry_msgs

# Import messages type
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int32
from uarm.msg import Angles
from uarm.msg import Coords
from uarm.msg import CoordsWithTime
from uarm.msg import CoordsWithTS4
from uarm.srv import MoveTo, MoveToResponse
from uarm.srv import GetPositions, GetPositionsResponse

class ROSKTHUarmBridge(object):

    def __init__(self, calibration_file_name):
        self._uarm = kth_uarm.KTHUarm(calibration_file_name)

    def handle_get_positions(self, request):
        """Process a GetPositions request."""
        rospy.loginfo("A GetPositions request has been received. Processing...")
        response = GetPositionsResponse()
        eef_position = self._uarm.get_position()
        response.position.x = eef_position[0]
        response.position.y = eef_position[1]
        response.position.z = eef_position[2]

        config = self._uarm.get_configuration()
        response.angles = list(config)
        return response

    def handle_move_to(self, request):
        """Process a MoveTo service request."""
        begin_time = rospy.Time.now()
        target_position = request.position
        move_mode = request.move_mode
        duration = request.movement_duration
        ignore_orientation = request.ignore_orientation
        interpolation_type_int = request.interpolation_type
        check_limits = request.check_limits

        if move_mode != request.ABSOLUTE_MOVEMENT and move_mode != request.RELATIVE_MOVEMENT:
            rospy.logerr("MoveTo request contains an invalid move mode. Aborting.")
            return self.create_move_to_error(begin_time)

        interpolation_type = None
        if interpolation_type_int == request.NO_INTERPOLATION:
            interpolation_type = 'None'
        elif interpolation_type_int == request.CUBIC_INTERPOLATION:
            interpolation_type = 'Cubic'
        elif interpolation_type_int == request.LINEAR_INTERPOLATION:
            interpolation_type = 'Linear'
        else:
            rospy.logerr("MoveTo request contains an invalid ease type. Aborting")
            return self.create_move_to_error(begin_time)

        if duration.to_sec() <= 0.0 and interpolation_type != 'None':
            rospy.logerr("MoveTo request contains an invalid duration (must be >= 0.0). Aborting.")
            return self.create_move_to_error(begin_time)

        rospy.loginfo("A proper MoveTo request has been received. Processing...")

        theta = None
        if not ignore_orientation:
            theta = request.eef_orientation
        # Based on the request either do a relative movement or absolute movement
        success = False
        if move_mode == request.RELATIVE_MOVEMENT:
            success = self._uarm.move_cartesian_relative(dx=target_position.x,
                                                         dy=target_position.y,
                                                         dz=target_position.z,
                                                         dtheta=theta,
                                                         interpolation_type,
                                                         duration=duration,
                                                         check_limits=check_limits)
        else:
            success = self._uarm.move_cartesian(x=target_position.x, y=target_position.y,
                                                z=target_position.z, theta=theta,
                                                interpolation_type, duration=duration,
                                                check_limits=check_limits)

        response = MoveToResponse()
        move_time = rospy.Time.now()
        eef_position = self._uarm.get_position()
        response.position.x = eef_position[0]
        response.position.y = eef_position[1]
        response.position.z = eef_position[2]
        response.elapsed = move_time - begin_time
        response.error = not success
        return response

    def handle_move_to_joints(self, request):
        """Process a MoveToJoints service request."""
        begin_time = rospy.Time.now()
        target_configuration = [request.j0, request.j1, request.j2, request.j3]
        move_mode = request.move_mode
        duration = request.movement_duration
        interpolation_type_int = request.interpolation_type
        check_limits = request.check_limits

        if move_mode != request.ABSOLUTE_MOVEMENT and move_mode != request.RELATIVE_MOVEMENT:
            rospy.logerr("MoveToJoints request contains an invalid move mode. Aborting.")
            return self.create_move_to_joints_error(begin_time)

        interpolation_type = None
        if interpolation_type_int == request.NO_INTERPOLATION:
            interpolation_type = 'None'
        elif interpolation_type_int == request.CUBIC_INTERPOLATION:
            interpolation_type = 'Cubic'
        elif interpolation_type_int == request.LINEAR_INTERPOLATION:
            interpolation_type = 'Linear'
        else:
            rospy.logerr("MoveToJoints request contains an invalid ease type. Aborting")
            return self.create_move_to_joints_error(begin_time)

        if duration.to_sec() <= 0.0 and interpolation_type != 'None':
            rospy.logerr("MoveToJoints request contains an invalid duration (must be >= 0.0). Aborting.")
            return self.create_move_to_joints_error(begin_time)

        rospy.loginfo("A proper MoveToJoints request has been received. Processing...")

        # Based on the request either do a relative movement or absolute movement
        success = False
        if move_mode == request.RELATIVE_MOVEMENT:
            success = self._uarm.move_relative(*target_configuration,
                                               interpolation_type,
                                               duration=duration,
                                               check_limits=check_limits)
        else:
            success = self._uarm.move(*target_configuration,
                                      interpolation_type,
                                      duration=duration,
                                      check_limits=check_limits)

        response = MoveToJointsResponse()
        [response.j0, response.j1, response.j2, response.j3] = self._uarm.get_configuration()
        move_time = rospy.Time.now()
        response.elapsed = move_time - begin_time
        response.error = not success
        return response

    def handle_pump(self, request):
        self._uarm.pump(request.pump)
        response = PumpResponse()
        response.pump_status = self._uarm.is_sucking()
        return response

    def handle_attach_detach(self, request):
        reponse = AttachDetachResponse()
        if request.attach:
            self._uarm.attach_all_servos()
            response.attach = True
        else:
            self._uarm.detach_all_servos()
            response.attach = False
        return response

    def create_move_to_error(self, init_time):
        """Return a MoveToResponse filled with information and positive error flag."""
        response = MoveToResponse()
        position, response.theta = self._uarm.get_pose()
        response.position.x = position[0]
        response.position.y = position[1]
        response.position.z = position[2]
        current_time = rospy.Time.now()
        response.elapsed = current_time - init_time
        response.error = True
        return response

    def create_move_to_joints_error(self, init_time):
        """Return a MoveToJointsResponse filled with information and positive error flag."""
        response = MoveToJointsResponse()
        [response.j0, response.j1, response.j2, response.j3] = self._uarm.get_configuration()
        current_time = rospy.Time.now()
        response.elapsed = current_time - init_time
        response.error = True
        return response

    def publish_state(self):
        """ Publishes the current joint angles and end-effector TF """
        # TODO



def listener():
    """monitor mode for listening to all topics."""
    print ' '
    print 'Begin monitor mode - listening to all functional topics'
    print '======================================================='
    print '         Use rqt_graph to check the connection         '
    print '======================================================='

    rospy.init_node('uarm_core', anonymous=True)

    rospy.Subscriber("uarm_status", String, attachCallback)
    rospy.Subscriber("pump_control", UInt8, pumpCallack)
    rospy.Subscriber("pump_str_control", String, pumpStrCallack)

    rospy.Subscriber("read_coords", Int32, currentCoordsCallback)
    rospy.Subscriber("read_angles", Int32, readAnglesCallback)
    rospy.Subscriber("stopper_status", Int32, stopperStatusCallback)

    rospy.Subscriber("write_angles", Angles, writeAnglesCallback)
    rospy.Subscriber("move_to", Coords, moveToCallback)
    rospy.Subscriber("move_to_time", CoordsWithTime, moveToTimeCallback)
    rospy.Subscriber("move_to_time_s4", CoordsWithTS4, moveToTimeAndS4Callback)

    rospy.Service("uarm/move_to", MoveTo, handle_move_to)
    rospy.Service("uarm/get_positions", GetPositions, handle_get_positions)

    rospy.spin()
    pass


def processFailedNum(failed_number):
    """show errors."""

    if failed_number > 20 and failed_number < 26:
        print 'ERROR: Input Connection Address Is Incorrect'
    if failed_number == 20:
        print 'uArm: Please Connect uArm first '


if __name__ == '__main__':

    try:
        # Connect uarm first
        return_value = connectFcn()

        # Control uarm through commands
        controlFcn()

        # Monitor mode
        if listenerFcn is True:
            listener()

    except:
        processFailedNum(failed_number)
        print 'ERROR: Execution Failed'
        pass

    finally:
        print 'DONE: Program Stopped'
        pass
