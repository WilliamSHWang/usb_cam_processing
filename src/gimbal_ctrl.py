#!/usr/bin/python

import sys
import serial
import time
import rospy
from std_msgs.msg import String
import numpy as np
from usb_cam_processing.msg import Centroid

ser = serial.Serial()
ser.baudrate = 19200
ser.port = '/dev/ttyACM0'
ser.open()

x = 1500
y = 1500

def pololu_set_target(device, channel, target):
    if target > 2000:
        target = 2000
    elif target < 1000:
        target = 1000
    target *= 4
    target_upper_seven = (target >> 7) & 0xff
    target_lower_seven = (target & 0b1111111) & 0xff

    return bytearray([0xAA, device, 0x04, channel, target_lower_seven, target_upper_seven])

def movecam(ballposition, res, param):
    '''
    take in position (tuple) of the ball, and the resolution of the screen
    output the movement vector for the servos
    '''
    xcenter = res[0]/2
    ycenter = res[1]/2

    xdel = (ballposition[0] - xcenter) * param
    ydel = (ballposition[1] - ycenter) * param

    return xdel, ydel

def callback(data_string):
    x_ball = data_string.x_center
    y_ball = data_string.y_center
    width = data_string.width
    height = data_string.height

    xcenter = width / 2
    ycenter = height / 2

    xd = (x_ball - xcenter) * 0.05
    yd = (y_ball - ycenter) * 0.05




    global x, y, ser

    if np.abs(x_ball - xcenter) > 10:
        x = int(x - xd)
    if np.abs(y_ball - ycenter) > 10:
        y = int(y + yd)



    if x > 2200:
        x = 2200
    elif x < 800:
        x = 800
    if y > 2500:
        y = 2500
    elif y < 500:
        y = 500

    print x, y

    ser.write(pololu_set_target(12, 2, y)) # tilt
    ser.write(pololu_set_target(12, 3, x)) # pan


    # s = String(data)
    # if s[0].lower() == 'p':
    #     ser.write(pololu_set_target(12, 0, int(s[1:])))
    # elif s[0].lower() == 't':
    #     ser.write(pololu_set_target(12, 1, int(s[1:])))




def listener():

    ser.write(pololu_set_target(12, 2, 2000))
    ser.write(pololu_set_target(12, 3, 1500))

    rospy.init_node('pan_tilt_act_node', anonymous=True)
    rospy.Subscriber('/ball_center', Centroid, callback)
    rospy.spin()



if __name__ == '__main__':

    listener()
