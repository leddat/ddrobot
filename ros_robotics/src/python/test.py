from robot_control_class import RobotControl

robotcontrol=RobotControl()

a=robotcontrol.get_laser(360)

while a>1:
    robotcontrol.move_straight()
    a=robotcontrol.get_laser(360)
robotcontrol.stop_robot()
