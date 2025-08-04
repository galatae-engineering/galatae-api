import serial
import time
from threading import Thread
import json
import math

class Robot:
  def __init__(self,port):
    self.arduino=serial.Serial(port=port, baudrate=9600, timeout=0.1)
    self.absolute_distance_mode=True
    self.foetus_pos=[9.07343,0,103.7765,84.99841]
    self.debug=False
    time.sleep(2)

  def send_message(self,message):
    self.arduino.write((message+"\n").encode('utf-8'))
    if(self.debug):
      print(message+" sent")

  def read_message(self):
    a=self.arduino.readline()
    data = a.decode('utf-8', errors='ignore')
    return data

  def print_message_from_serial(self,message):
    print("arduino: "+message)

  def read_serial_buffer(self):
    message=self.read_message()
    return message.strip()

  def wait_for_message(self):
    message=self.read_serial_buffer()
    
    while(message==""):
      message=self.read_serial_buffer()
      time.sleep(0.001)

    if(self.debug):
      self.print_message_from_serial(message)
    
    return message

  def drop_all_serial_messages(self):
    message=self.read_serial_buffer()

    while(message!=""):
      if(self.debug):
        self.print_message_from_serial(message)
      message=self.read_serial_buffer()


  def send_message_and_wait_conf(self,message):
    self.send_message(message)
    message=self.wait_for_message()
    
    while(message!="ok" and message!="error"):
      message=self.wait_for_message()

    if(message=="ok"):
      success=True
    else:
      success=False
      self.drop_all_serial_messages()

    return success

  def get_gcode_arguments_string(self,args):
    names=["X","Y","Z","A","B","C"]
    s=""
    for i in range(len(args)):
      s=s+names[i]+str(round(args[i],2))

    return s

  def move(self,point):
    message="G1"+self.get_gcode_arguments_string(point)
    return self.send_message_and_wait_conf(message)
  
  def probe(self,point):
    message="G38.2"+self.get_gcode_arguments_string(point)
    return self.send_message_and_wait_conf(message)
  
  def update_absolute_distance_mode(self,expected_value):
    if(self.absolute_distance_mode != expected_value):
      self.send_message_and_wait_conf(["G91","G90"][expected_value])
      self.absolute_distance_mode=expected_value

  #point=[x,y,z,pitch (relative to z axis), roll (relative to arm),gripper angle]
  def go_to_point(self,point):
    self.update_absolute_distance_mode(True)
    self.move(point)
      
  def jog(self,point):
    self.update_absolute_distance_mode(False)
    self.move(point)

  def get_robot_pos_json(self):
    self.send_message("?")
    message=self.wait_for_message()
    if(message == "ok"):
      print("error, get_tool_pose intercepted ok message")
    return message

  def get_tool_pose(self):
    message=self.get_robot_pos_json()
    return json.loads(message)["tool_pose"]

  def get_point_in_line_segment(self,p1,p2,rel_pos):
    p=[]
    number_of_indexes=min(len(p1),len(p2))

    for i in range(number_of_indexes):
      p.append(p1[i]+(p2[i]-p1[i])*rel_pos)

    return p

  def dist_between_vectors(self,p1,p2):
    square_dist=0
    for i in range(len(p2)):
      print(i)
      square_dist+=math.pow(p2[i]-p1[i],2)

    return math.sqrt(square_dist)

  def linear_move_to_point(self,p2):
    p1=self.get_tool_pose()
    N=math.ceil(self.dist_between_vectors(p1,p2))

    for i in range(N+1):
      intermediate_point=self.get_point_in_line_segment(p1,p2,i/N)
      self.go_to_point(intermediate_point)

  def get_angles(self):
    message=self.get_robot_pos_json()
    return json.loads(message)["angles"]
    
  #place the robot in foetus position and launch this command
  def reset_pos(self):
    message="G92"+self.get_gcode_arguments_string(self.foetus_pos)
    self.send_message_and_wait_conf(message)

  #speed in deg/s , 100 deg/s is like a good value
  def set_joint_speed(self,speed):
    message="F"+str(speed)
    self.send_message_and_wait_conf(message)

  def open_gripper(self,extra_degrees=0):
    gcode="M101"
    if(extra_degrees!=0):
      gcode+="P"+str(extra_degrees)
    self.send_message_and_wait_conf(gcode)

  def close_gripper(self,extra_degrees=0):
    self.send_message_and_wait_conf("M100P"+str(extra_degrees))

  def calibrate_gripper(self):
    self.send_message_and_wait_conf("$H")

  def go_to_foetus_pos(self):
    self.send_message_and_wait_conf("G28")

  def set_tool(self,args): #[x,y,z]
    self.send_message_and_wait_conf("G10L2P1"+self.get_gcode_arguments_string(args))