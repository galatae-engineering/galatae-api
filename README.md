INSTALL:

cd ~/git/galatae-api
python3 -m venv venv
source venv/bin/activate
pip install inputs
pip install pyserial

LAUNCH:
cd ~/git/galatae_api
source venv/bin/activate
python robot.py


usefull functions:

reset_pos()
send_message_and_wait_conf(message)
go_to_point(point) ["X","Y","Z","A","B"] A=pitch B=roll
get_position()
set_joint_speed(speed)
open_gripper()
close_gripper()
go_to_foetus_pos()