# -*- coding: utf-8 -*-

################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

################################################################################
# Author flier: Added YARP support for sending bottles containing "hands" data #
# This is done by calculating the relative rotation for each bone in the right #
# hand. Results are in euler angles (could also be done as 3x3)                #
#                                                                              #
# Please do not forget to set your PYTHONPATH (see below) before running this  #
# PYTHONPATH=$PYTHONPATH:/path/to/SDK/lib:/path/to/SDK/lib/x64                 #
#                                                                              #
# This version is implemented using Leap SDK 2.x and Yarp 1.4.x                #
################################################################################

import Leap, sys, thread, time
from numpy.linalg import inv
from math import acos, atan2, cos, pi, sin
from numpy import float64, hypot, zeros, matrix
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


class LeapListener(Leap.Listener):
    
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    # Method to get the bone rotation of a given finger as 3x3 matrix
    def get_bone_rotation(self, bone):
        local_rotation = bone.basis.rigid_inverse().to_array_3x3()
        return local_rotation

    # Method to get the hand rotation as 3x3 matrix
    def get_hand_rotation(self, hand):
        local_rotation = hand.basis.rigid_inverse().to_array_3x3()
        return local_rotation

    # Derive euler angles from rotation matrix
    def mat_to_euler(self, _matrix):
        yaw = acos(_matrix[2, 2])
        pitch = -atan2(_matrix[2, 0], _matrix[2, 1])
        roll = -atan2(_matrix[0, 2], _matrix[1, 2])
        return yaw * (180 / pi), pitch * (180 / pi), roll * (180 / pi)

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        # Get hands
        for hand in frame.hands:

            hand_type = "Left hand" if hand.is_left else "Right hand"

            if hand_type == "Right hand":
                # Get the hand's normal vector and direction
                normal    = hand.palm_normal
                direction = hand.direction
                mat3x3    = self.get_hand_rotation(hand)
                np_hand_mat3x3 = matrix([[mat3x3[0], mat3x3[1], mat3x3[2]],
                                         [mat3x3[3], mat3x3[4], mat3x3[5]],
                                         [mat3x3[6], mat3x3[7], mat3x3[8]]])
                # Get arm bone
                arm = hand.arm

                # Get fingers
                for finger in hand.fingers:
                    # Get bones
                    print "Finger Name: %s" % self.finger_names[finger.type()]
                    mat_prox = matrix([[1, 1, 1],
                                       [1, 1, 1],
                                       [1, 1, 1]])
                    mat_inte = matrix([[1, 1, 1],
                                       [1, 1, 1],
                                       [1, 1, 1]])
                    mat_dist = matrix([[1, 1, 1],
                                       [1, 1, 1],
                                       [1, 1, 1]])
                    for b in range(0, 4):
                        bone = finger.bone(b)
                        mat3x3 = self.get_bone_rotation(bone)
                        if self.bone_names[bone.type] == "Proximal":
                            np_mat3x3 = matrix([[mat3x3[0], mat3x3[1], mat3x3[2]],
                                                [mat3x3[3], mat3x3[4], mat3x3[5]],
                                                [mat3x3[6], mat3x3[7], mat3x3[8]]])
                            mat_prox = np_mat3x3
                        if self.bone_names[bone.type] == "Intermediate":
                            np_mat3x3 = matrix([[mat3x3[0], mat3x3[1], mat3x3[2]],
                                                [mat3x3[3], mat3x3[4], mat3x3[5]],
                                                [mat3x3[6], mat3x3[7], mat3x3[8]]])
                            mat_inte = np_mat3x3
                        if self.bone_names[bone.type] == "Distal":
                            np_mat3x3 = matrix([[mat3x3[0], mat3x3[1], mat3x3[2]],
                                                [mat3x3[3], mat3x3[4], mat3x3[5]],
                                                [mat3x3[6], mat3x3[7], mat3x3[8]]])
                            mat_dist = np_mat3x3

                    print "--> Proximal bone [YAW|PITCH|ROLL]"
                    relative_prox = inv(np_hand_mat3x3) * mat_prox
                    print self.mat_to_euler(relative_prox)

                    print "----> Intermediate bone [YAW|PITCH|ROLL]"
                    relative_inte = inv(relative_prox) * mat_inte
                    print self.mat_to_euler(relative_inte)

                    print "------> Distal bone [YAW|PITCH|ROLL]"
                    relative_dist = inv(relative_inte) * mat_dist
                    print self.mat_to_euler(relative_dist)

            else:
                print "For the moment this implementation only tracks the right hand. Feel free to add the left hand"
                print "Be advised: the coordinate system is left-handed in that case ;)"

        # Get tools, we yet don't need those, save CPU time...
        '''
        for tool in frame.tools:
            pass
        '''

        # Get gestures, we yet don't need those, save CPU time...
        '''
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)

                # Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    clockwiseness = "clockwise"
                else:
                    clockwiseness = "counterclockwise"

                # Calculate the angle swept since the last frame
                swept_angle = 0
                if circle.state != Leap.Gesture.STATE_START:
                    previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                    swept_angle     = (circle.progress - previous_update.progress) * 2 * Leap.PI


            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)

            if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                keytap = KeyTapGesture(gesture)

            if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                screentap = ScreenTapGesture(gesture)
        '''

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            pass

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"


def main():
    # Create a sample listener and controller
    listener   = LeapListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()
