GALATAE API
=

Introduction
-
Library to control the Galatae robot through Python

Installation
-
# On the robot controller
The library is installed by default, just add **from robot import Robot** on the top of your file and you can start programming.

# On your computer

On your terminal, write:
1. On your terminal, write: **git clone https://github.com/galatae-engineering/galatae-api.git**
2. Go to your repository
3. On your terminal write : 
    * pip install inputs
    * pip install pyserial
    * pip install opencv-python
4. Above you main python file, write :
    * sys.path.append('path/to/galatae-api/')
    * from robot import Robot
    * start programming

# Example

from robot import Robot  
  
r=Robot(False)  
r.set_joint_speed(100) # max joint speed = 100° per second  
r.calibrate()  
r.go_to_pose([100, 10, 50, 180, 0]) # x=100mm, y=10mm, z=50mm, pitch=180°, roll=0°  
r.go_to_foetus_pos()  
r.disable_motors()  

# Docs

## def Robot(debug=False):
    Constructor, if you set debug as true, you can follow the communication between robot and controller. Normally not needed.

## def calibrate():
    You should calibrate the robot once before every program to ensure precision. This function must be runned when robot is placed in foetus position.

##  def set_joint_speed(speed):
    Speed in deg per sec.

##  def go_to_pose(pose):
    pose is a python array : [x,y,z,pitch,roll]

##  def get_tool_pose():
    return self._ask_for_pos_json_and_return_property_value("tool_pose")

##  def get_angles(self):
    return self._ask_for_pos_json_and_return_property_value("angles")

  #speed in deg/s , 100 deg/s is like a good value

##  def linear_move_to_pose(self,p2):
    p1=self.get_tool_pose()
    N=self._set_joint_speed_and_get_number_of_iterations(p1,p2)

    for i in range(N+1):
      intermediate_pose=self._get_point_in_line_segment(p1,p2,i/N)
      self.go_to_pose(intermediate_pose)



##  def go_to_foetus_pos(self):
    self.send_message_and_wait_conf("G28")

##  def disable_motors(self):
    self.send_message_and_wait_conf("M18")












##  def send_message(self,message):
    self.arduino.write((message+"\n").encode('utf-8'))
    if(self.debug):
      print(message+" sent")

##  def send_message_and_wait_conf(self,message):
    self.send_message(message)
    return self._wait_for_conf()

##  def reset(self):
    return self.send_message_and_wait_conf("$RST=*")


  
##  def probe(self,pose):
    return self._send_command_followed_by_arguments("G38.2",pose,True)
  
##  def update_absolute_distance_mode(self,expected_value):
    if(self.absolute_distance_mode != expected_value):
      self.send_message_and_wait_conf(["G91","G90"][expected_value])
      self.absolute_distance_mode=expected_value

  #pose=[x,y,z,pitch (relative to z axis), roll (relative to arm),gripper angle]
  

      
##  def jog(self,pose):
    self.update_absolute_distance_mode(False)
    self._move(pose)


  

  

  


##  def linear_probe(self,p2):
    self.update_absolute_distance_mode(True)
    p1=self.get_tool_pose()
    N=self._set_joint_speed_and_get_number_of_iterations(p1,p2)
    
    success=False
    i=0
    while(i<=N and success==False):
      intermediate_pose=self._get_point_in_line_segment(p1,p2,i/N)
      success=self.probe(intermediate_pose)
      i+=1

    return success

##  def open_gripper(self,extra_degrees=0):
    gcode="M101"
    if(extra_degrees!=0):
      gcode+="P"+str(extra_degrees)
    self.send_message_and_wait_conf(gcode)

##  def close_gripper(self,extra_degrees=0):
    self.send_message_and_wait_conf("M100P"+str(extra_degrees))

##  def calibrate_gripper(self):
    self.send_message_and_wait_conf("$H")



##  def set_tool(self,coord): #[x,y,z]
    return self._send_command_followed_by_arguments("G10L2P1",coord,True)

##  def reset_angles(self,angles):
    return self._send_command_followed_by_arguments("G92",angles,False)



##  def show_video(self):
    cap = cv.VideoCapture(0)

    if(cap.isOpened()):
      while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        cv.imshow('frame',frame)
        if cv.waitKey(1) == ord('q'):
            break
    else:
        print("Cannot open camera, so running without it.")

    cap.release()
    cv.destroyAllWindows()

    