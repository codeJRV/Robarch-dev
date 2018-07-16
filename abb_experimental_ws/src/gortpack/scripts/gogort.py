#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String


from itertools import izip_longest

def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return izip_longest(fillvalue=fillvalue, *args)

FILE_PATH = "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/Pattern/GFT.txt"


def move_group_python_interface_tutorial():

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

  print "============ Waiting for RVIZ..."
  rospy.sleep(10)

  wpose = geometry_msgs.msg.Pose()  
  wpose = group.get_current_pose().pose
  print wpose

  lines = [line.rstrip('\n') for line in open(FILE_PATH) if line.startswith(("MoveL")) ]
  ip_waypoints = []

  # for line in lines:
  #   print lines

  for line in lines:
    line = line.split(']', 1)[0]
    line = line[8:]
    coordinates  = line.split(',')
    ip_waypoints.append(coordinates)

  print ip_waypoints
  
  waypoints=[]
  path_plan = []

  for coordinates in ip_waypoints:
    wpose.position.x = float(coordinates[0])/1000
    wpose.position.y = float(coordinates[1])/1000
    waypoints.append(copy.deepcopy(wpose))

  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
  
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)

  is_safe = raw_input("Plan ok to exexute? : y/n ")
  slow_move = raw_input("Move slow to checkpoints? : y/n ")

  if(slow_move == 'y' and is_safe=='y'):
    for coordinates1, coordinates2 in grouper(2, ip_waypoints):
      waypoints=[]
      wpose.position.x = float(coordinates1[0])/1000
      wpose.position.y = float(coordinates1[1])/1000
      waypoints.append(copy.deepcopy(wpose))
      wpose.position.x = float(coordinates2[0])/1000
      wpose.position.y = float(coordinates2[1])/1000
      waypoints.append(copy.deepcopy(wpose))
      (plan, fraction) = group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)         # jump_threshold
      #path_plan.append(plan)
      if(is_safe=='y'):
        print("Executing trajectory")
        group.execute(plan)
  else:
    if(is_safe=='y'):
      print("Executing trajectory")
      group.execute(plan3)


                               
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)





  

  collision_object = moveit_msgs.msg.CollisionObject()
  moveit_commander.roscpp_shutdown()

  print "============ STOPPING"


if __name__=='__main__':

  print "TrackGort"  
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
