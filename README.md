INSTALL:

cd ~/git/galatae_software/python
python3 -m venv venv
source venv/bin/activate
pip install inputs
pip install pyserial

LAUNCH:
cd ~/git/galatae_software/python
source venv/bin/activate
python main.py


usefull functions:

reset_pos()
send_message_and_wait_conf(message)
go_to_point(point)
get_position()
set_joint_speed(speed)
open_gripper()
close_gripper()
got_to_foetus_pos()