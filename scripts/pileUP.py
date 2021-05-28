#!/usr/bin/env python

import time
import math
from numpy.linalg import inv
import numpy as np
import cv2
from tm_msgs.srv import *
from tm_msgs.msg import *
import rospy
import sys
sys.path.append('/home/robot/catkin_ws/devel/lib/python2.7/dist-packages')


def send_script(script):
    rospy.wait_for_service('/tm_driver/send_script')
    try:
        script_service = rospy.ServiceProxy(
            '/tm_driver/send_script', SendScript)
        move_cmd = SendScriptRequest()
        move_cmd.script = script
        resp1 = script_service(move_cmd)
    except rospy.ServiceException as e:
        print("Send script service call failed: %s" % e)


def set_io(state):
    rospy.wait_for_service('/tm_driver/set_io')
    try:
        io_service = rospy.ServiceProxy('/tm_driver/set_io', SetIO)
        io_cmd = SetIORequest()
        io_cmd.module = 1
        io_cmd.type = 1
        io_cmd.pin = 0
        io_cmd.state = state
        resp1 = io_service(io_cmd)
    except rospy.ServiceException as e:
        print("IO service call failed: %s" % e)


def gotoget(x_g, y_g, t_g, pile_z):
    targetP1 = " %d, %d, 200.0 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)

    targetP1 = "  %d, %d, 107.50 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    time.sleep(0.5)
    set_io(1.0)
    time.sleep(0.5)

    targetP1 = "  %d, %d, 200.0 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(1.0)
    targetP1 = "400.00 , 0.00 , 200.0 , 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(1.0)  # 1.0: close gripper, 0.0: open gripper
    targetP1 = "400.00 , 0.00 , %f , 180.00 , 0.00 , 135.00" %(pile_z)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)
    targetP1 = "400.00 , 0.00 , 200.0, 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)


def getpic(T_w_c, worldOrigin, imageOrigin):
    
    
    # take platform image
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    ret, frame = cap.read()
    cap.release()

    # frame = cv2.imread('4.jpg')

    cv2.imshow('frame', frame)
    cv2.waitKey(500)

    # image processing parameters
    # rectangle_point = [(194, 115), (815, 553)]
    rectangle_point = [(185, 79), (785, 498)]
    thresh1 = 127
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    objSizeBound = [150000, 700000]
    lengthOfLine = 200

    phiOffset = math.atan2(T_w_c[0, 0], T_w_c[1, 0])

    mask = np.zeros(frame.shape[0:2], dtype='uint8')
    cv2.rectangle(mask, rectangle_point[0], rectangle_point[1], 255, -1)
    mask_bit = cv2.bitwise_and(mask, mask)

    # frame_withRectangle = cv2.rectangle(frame.copy(), rectangle_point[0], rectangle_point[1], (0,255,255), 1)
    # cv2.imshow('frame', frame_withRectangle)
    # cv2.waitKey(0)

    frame_mask = cv2.bitwise_and(frame, frame, mask=mask_bit)
    gray = cv2.cvtColor(frame_mask.copy(), cv2.COLOR_BGR2GRAY)
    _,  binarized = cv2.threshold(gray, thresh1, 255, cv2.THRESH_BINARY)

    # cv2.imshow('frame', binarized)
    # cv2.waitKey(0)

    opening = cv2.morphologyEx(
        binarized, cv2.MORPH_OPEN, kernel_open, iterations=2)
    # cv2.imshow('frame', opening)
    # cv2.waitKey(0)

    _, contours, hierarchy = cv2.findContours(
        opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    allObj_plot = np.zeros(opening.shape, dtype='uint8')
    moments = []

    for ct in contours:
        obj_plot = np.zeros(opening.shape, dtype='uint8')
        cv2.fillPoly(obj_plot, [ct], 255)

        m = cv2.moments(obj_plot)
        if m["m00"] > objSizeBound[0] and m["m00"] < objSizeBound[1]:

            cv2.fillPoly(allObj_plot, [ct], 255)
            moments.append(m)
            # print(m["m00"])
            # cv2.imshow('frame', obj_plot)
            # cv2.waitKey(0)

    # cv2.imshow('frame', allObj_plot)
    # cv2.waitKey(0)

    img_poses = []
    result = cv2.cvtColor(allObj_plot, cv2.COLOR_GRAY2BGR)
    item_num = 0
    for M in moments:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        phi = np.arctan2(2.0*M["nu11"], M["nu20"] - M["nu02"])*0.5

        p1 = (int(cX - 0.5*lengthOfLine*math.cos(phi)),
              int(cY - 0.5*lengthOfLine*math.sin(phi)))
        p2 = (int(cX + 0.5*lengthOfLine*math.cos(phi)),
              int(cY + 0.5*lengthOfLine*math.sin(phi)))
        print("camera position", [cX, cY, phi*180/np.pi])

        img_poses.append([cX, cY, phi])
        cv2.line(result, p1, p2, (0, 0, 255), 2)
        cv2.circle(result, (cX, cY), 4, (0, 255, 0), -1)
        cv2.putText(result, "cam phi: {:0.2f}".format(
            phi*180/np.pi), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(result, "base phi: {:0.2f}".format(
            (phi+phiOffset)*180/np.pi), (cX, cY+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(result, str(cX)+","+str(cY), (cX - 30, cY - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(result, str(item_num), (cX, cY+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        item_num += 1
    print()


    cv2.imshow('frame', result)
    cv2.waitKey(800)

    cv2.destroyAllWindows()

    world_poses = []

    for pose in img_poses:
        
        P_c = np.array(pose[0:2])
        P_world = np.dot(T_w_c, (P_c - imageOrigin.T)) + worldOrigin.T

        
        phi_world = (-pose[2] + phiOffset)*180/np.pi + 90
        world_poses.append([P_world[0], P_world[1], phi_world])
        print("obj_pose_world  ", [P_world[0], P_world[1], phi_world])
    # print("wp",world_poses)
    return world_poses


def gotofun(x_g, y_g):
    t_g = 135
    targetP1 = "400.00 , 0.00 , 145.50 , 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)
    targetP1 = "400.00 , 0.00 , 110.50 , 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    asd = raw_input("Press \"Enter\" when item is in the gripper...")
    print("gripped item!!!")
    set_io(1.0)
    targetP1 = "400.00 , 0.00 , 145.50 , 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(1.0)
    targetP1 = " %d, %d, 140.50 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(1.0)

    targetP1 = "  %d, %d, 110.50 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)
    targetP1 = "  %d, %d, 140.50 , 180.00 , 0.00 , %f" % (x_g, y_g, t_g)
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)
    targetP1 = "400.00 , 0.00 , 145.50 , 180.00 , 0.00 , 135.00"
    script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    send_script(script)
    set_io(0.0)  # 1.0: close gripper, 0.0: open gripper


def getT_w_c(cali_base_poses):
    # calibration

    # cali_base_poses = [[250, 250], [350, 250], [450, 250], [250, 350], [250, 450]]

    # put tiles
    for cali_base_pose in cali_base_poses:
        gotofun(cali_base_pose[0], cali_base_pose[1])

    raw_input("Press \"Enter\" when the robot arm move to origin...")
    print("Ready to take Calibration Photo!!!")

    # take platform image
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    ret, frame = cap.read()
    cap.release()

    # frame = cv2.imread('4.jpg')

    

    # image processing parameters
    # rectangle_point = [(194, 115), (815, 553)]
    rectangle_point = [(185, 79), (785, 498)]
    thresh1 = 127
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    objSizeBound = [150000, 700000]
    lengthOfLine = 200

    # get image points
    mask = np.zeros(frame.shape[0:2], dtype='uint8')
    cv2.rectangle(mask, rectangle_point[0], rectangle_point[1], 255, -1)
    mask_bit = cv2.bitwise_and(mask, mask)
    frame_mask = cv2.bitwise_and(frame, frame, mask=mask_bit)
    gray = cv2.cvtColor(frame_mask.copy(), cv2.COLOR_BGR2GRAY)
    _,  binarized = cv2.threshold(gray, thresh1, 255, cv2.THRESH_BINARY)
    opening = cv2.morphologyEx(
        binarized, cv2.MORPH_OPEN, kernel_open, iterations=2)
    _, contours, _ = cv2.findContours(
        opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    allObj_plot = np.zeros(opening.shape, dtype='uint8')
    moments = []
    for ct in contours:
        obj_plot = np.zeros(opening.shape, dtype='uint8')
        cv2.fillPoly(obj_plot, [ct], 255)

        m = cv2.moments(obj_plot)
        if m["m00"] > objSizeBound[0] and m["m00"] < objSizeBound[1]:

            cv2.fillPoly(allObj_plot, [ct], 255)
            moments.append(m)

    img_poses = []
    result = cv2.cvtColor(allObj_plot, cv2.COLOR_GRAY2BGR)
    item_num = 0
    for M in moments:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        img_poses.append([cX, cY])
        cv2.circle(result, (cX, cY), 4, (0, 255, 0), -1)
        cv2.putText(result, str(cX)+","+str(cY), (cX - 30, cY - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(result, str(item_num), (cX, cY+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        item_num += 1

    cv2.imshow('frame', result)
    cv2.waitKey(0)
    cv2.destroyWindow('frame')
    item_order = raw_input("input item_num with cali_base_poses' order: \n")
    item_order = item_order.split(',')
    print("item order is:")
    print(item_order)
    

    imageOrigin = np.array(img_poses[int(item_order[0])])
    imagePs = []
    for i in range(1, len(item_order)):
        imagePs.append(np.array(img_poses[int(item_order[i])]))
    imagePs_np = np.vstack(imagePs).T

    worldOrigin = np.array(cali_base_poses[0])
    worldPs = []
    for i in range(1, len(cali_base_poses)):
        worldPs.append(np.array(cali_base_poses[i]))
    print(worldPs)
    worldPs_np = np.vstack(worldPs).T
    print(worldPs_np)
    print("worldPs_np.shape: ", worldPs_np.shape)
    print("worldOrigin.T.shape", worldOrigin.T.shape)

    P_wrd = (worldPs_np.T - worldOrigin).T
    P_img = (imagePs_np.T - imageOrigin).T

    # P_w = np.array([P1_wrd, P2_wrd, P3_wrd, P4_wrd]).T
    # # swap
    # P_c = np.array([[P1_img[1],P1_img[0]], [P2_img[1],P2_img[0]], [P3_img[1],P3_img[0]], [P4_img[1],P4_img[0]]])
    # P_c = np.flip(P_c,1).T
    print(P_wrd)
    print(P_img)
    print(P_img.shape)
    T_w_c = np.dot(P_wrd, np.linalg.pinv(P_img))
    print("T_w_c: ", T_w_c)

    return T_w_c, worldOrigin, imageOrigin


if __name__ == '__main__':
    try:
        rospy.init_node('send_scripts', anonymous=True)
        #--- move command by joint angle ---#
        # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'
        #--- move command by end effector's pose (x,y,z,a,b,c) ---#

        ### calibration
        # cali_base_poses = [[250, 250], [450, 250],
        #                    [250, 450]]
        # T_w_c, worldOrigin, imageOrigin = getT_w_c(cali_base_poses)
        # np.save('T_w_c.npy', T_w_c)
        # np.save('worldOrigin.npy', worldOrigin)
        # np.save('imageOrigin.npy', imageOrigin)

        
        ### execute
        T_w_c = np.load('T_w_c.npy')
        worldOrigin = np.load('worldOrigin.npy')
        imageOrigin = np.load('imageOrigin.npy')

        # back to origin
        targetP1 = "400.00 , 0.00 , 145.50 , 180.00 , 0.00 , 135.00"
        script = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
        send_script(script)
        set_io(0.0)  # 1.0: close gripper, 0.0: open gripper
        time.sleep(5)

        # grip every tile
        item_poses = getpic(T_w_c, worldOrigin, imageOrigin)
        pile_z0 = 110.5
        i=0
        for p in item_poses:
            gotoget(p[0], p[1], p[2], pile_z0+i*23)
            i+=1

        

    except rospy.ROSInterruptException:
        pass
