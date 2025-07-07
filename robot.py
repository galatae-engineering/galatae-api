import serial
import time
from threading import Thread
import json

class Robot:
  def __init__(self,port):
    self.arduino=serial.Serial(port=port, baudrate=9600, timeout=0.1)
    self.absolute_distance_mode=True
    self.foetus_pos=[9.07343,0,103.7765,84.99841]
    time.sleep(2)

  def send_message(self,message):
    #print(message+" sent")
    self.arduino.write((message+"\n").encode('utf-8'))

  def read_message(self):
    a=self.arduino.readline()
    data = a.decode('utf-8', errors='ignore')
    return data

  def wait_for_message(self):
    message=self.read_message()
    
    while(message==""):
      message=self.read_message()
      time.sleep(0.001)
    
    message=message.strip()
    #print("arduino: "+message)
    
    return message

  def send_message_and_wait_conf(self,message):
    self.send_message(message)
    message=self.wait_for_message()
    
    while(message!="ok" and message!="error"):
      message=self.wait_for_message()

    ok=False
    if(message=="ok"):
      ok=True
    return ok

  def get_gcode_arguments_string(self,args):
    names=["X","Y","Z","A","B","C"]
    s=""
    for i in range(len(args)):
      s=s+names[i]+str(args[i])

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