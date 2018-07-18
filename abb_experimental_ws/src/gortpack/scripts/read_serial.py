DIST_CORR  = 52

import serial
import time

class ReadSerial:
  ser  = serial.Serial('/dev/ttyUSB0', 38400)
  
  def __init__(self):
    print(self.ser.name)
    self.ser.write('/000R4D.')
    time.sleep(0.5)

  
  def getDist(self):    
    self.ser.write('/020D0059.')
    seq = []
    count = 1

    for c in self.ser.read():
	    #print(c)
      seq.append(c) #convert from ANSII
      joined_seq = ''.join(str(v) for v in seq) #Make a string from array

    if c == '.':
      #print("Line " + str(count) + ': ' + joined_seq[8:14] + "Time : " + str(rospy.get_time()))
      seq = []
      self.ser.write('/020D0059.')
      time.sleep(0.005)
      count += 1
      dist_str = joined_seq[8:11]
      if(dist_str.isdigit()):
        dist = float( int(dist_str) + DIST_CORR )/1000
        return dist
      

  def __del__(self):

    self.ser.write('/020D0a08.')
    time.sleep(3)
    self.ser.close()
     
    print 'Serial closed'