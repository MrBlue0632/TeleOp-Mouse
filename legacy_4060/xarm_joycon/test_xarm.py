from xarm.wrapper import XArmAPI  
import numpy as np

arm = XArmAPI('192.168.1.199')
arm.motion_enable(enable=True) 
arm.set_gripper_enable(enable=True) 
arm.set_mode(6)  
arm.set_state(0)  

init_qpos = np.array([-12.6, 22.7, -8.1, 166.4, 105.1, -1.2])
init_qpos = np.radians(init_qpos)

arm.set_servo_angle(angle=init_qpos,speed=0.2,is_radian=True)
arm.set_gripper_position(pos=840, wait=False)  # 设置夹持器位置为 50%，速度为 50%
      # 设置夹持力为 50%，可调q
      
