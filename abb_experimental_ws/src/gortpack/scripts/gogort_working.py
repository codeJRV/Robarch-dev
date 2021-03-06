#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from itertools import izip_longest

# import read_serial


import serial
import time


INPUT_FILE_PATH = "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/Pattern/Output80.txt"
OUTPUT_FILE_PATH = "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/Pattern/Corrected/Output80_op.txt"
REQUIRED_OFFSET = 0.05
DIST_CORR  = 52

def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return izip_longest(fillvalue=fillvalue, *args)

def extract_input_waypoints(path):
  lines = [line.rstrip('\n') for line in open(path) if line.startswith(("MoveL")) ]
  ip_waypoints = []

  for line in lines:
    line = line.split(']', 1)[0]
    line = line[8:]
    coordinates  = line.split(',')
    ip_waypoints.append(coordinates)
  
  # print ip_waypoints

  return ip_waypoints

def move_gort():

  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('gogort_node',
                  anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("manipulator")
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
  # sensor = read_serial.ReadSerial()

  ser  = serial.Serial('/dev/ttyUSB0', 38400)
  ser.write('/000R4D.')
  time.sleep(0.5)

  print "============ Waiting for RVIZ..."
  rospy.sleep(10)

  wpose = geometry_msgs.msg.Pose()  
  wpose = group.get_current_pose().pose
  print wpose

  ip_waypoints = extract_input_waypoints(INPUT_FILE_PATH)
  
  waypoints=[]
  zvals = []

  for coordinates in ip_waypoints:
    wpose.position.x = float(coordinates[0])/1000
    wpose.position.y = float(coordinates[1])/1000
    wpose.position.z = float(coordinates[2])/1000 
    waypoints.append(copy.deepcopy(wpose))

  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  
  print "============ Waiting while RVIZ displays the sensing path..."
  rospy.sleep(5)

  is_safe = raw_input("Plan ok to exexute? : y/n ")
  slow_move = raw_input("Move slow to checkpoints? : y/n ")

  if(is_safe=='y'):                     
    for coordinates1, coordinates2 in grouper(2, ip_waypoints):
      waypoints=[]
      wpose.position.x = float(coordinates1[0])/1000
      wpose.position.y = float(coordinates1[1])/1000
      wpose.position.z = float(coordinates1[2])/1000 
      waypoints.append(copy.deepcopy(wpose))
      wpose.position.x = float(coordinates2[0])/1000
      wpose.position.y = float(coordinates2[1])/1000
      wpose.position.z = float(coordinates2[2])/1000 
      waypoints.append(copy.deepcopy(wpose))
      (plan, fraction) = group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)         # jump_threshold
      
      print("Executing trajectory")

      dist = 400.0
      ser.write('/020D0059.')
      seq = []
      count = 1
      while(True):
        flag = False
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
              flag = True
              break
        if (flag == True):
          break  

      print('Z dist reading :',dist)
      zvals.append(dist)
      group.execute(plan)    

    # Write the Z values to an output file 
    opfile = open(OUTPUT_FILE_PATH, 'w')
    for item in zvals:
      opfile.write("%s\n" % item)
  else:
    print "============ STOPPING"
    rospy.sleep(5)
    collision_object = moveit_msgs.msg.CollisionObject()
    moveit_commander.roscpp_shutdown()
    return

  # Equation used for correction
  # z_new = z_input - (sensor_reading - required_offset)

  corrected_waypoints = []
  i = 0
  print len(zvals)
  print len(ip_waypoints)
  res = raw_input("Plan size correct? : y/n ")
  if(res != 'y'):
    return

  for coordinates in ip_waypoints:
    wpose.position.x = float(coordinates[0])/1000
    wpose.position.y = float(coordinates[1])/1000
    wpose.position.z = float(coordinates[2])/1000 - ( zvals[i] - REQUIRED_OFFSET )
    corrected_waypoints.append(copy.deepcopy(wpose))
    i +=1

  if(len(waypoints) ==  len(corrected_waypoints)):
    (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold


    print "============ Waiting while RVIZ displays the corrected path..."
    rospy.sleep(5)

    is_safe = raw_input("Plan ok to exexute? : y/n ")
    slow_move = raw_input("Move slow to checkpoints? : y/n ")


    if(is_safe=='y' and slow_move =='y'):
      for coordinates1, coordinates2 in grouper(2, corrected_waypoints):
        waypoints=[]
        wpose.position.x = float(coordinates1[0])/1000
        wpose.position.y = float(coordinates1[1])/1000
        wpose.position.z = float(coordinates1[2])/1000
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x = float(coordinates2[0])/1000
        wpose.position.y = float(coordinates2[1])/1000
        wpose.position.z = float(coordinates2[2])/1000
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        print("Executing trajectory")
        group.execute(plan)

    elif(is_safe=='y'):
      print("Executing trajectory")
      group.execute(plan3)
    else:
      print "============ STOPPING"
      rospy.sleep(5)
      collision_object = moveit_msgs.msg.CollisionObject()
      moveit_commander.roscpp_shutdown()
      return

  print "============ STOPPING"
  rospy.sleep(5)
  collision_object = moveit_msgs.msg.CollisionObject()
  moveit_commander.roscpp_shutdown()


if __name__=='__main__':

  print "TrackGort"  
  try:
    move_gort()
  except rospy.ROSInterruptException:
    pass
