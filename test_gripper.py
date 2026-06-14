import jkrc
import time

robot = jkrc.RC("192.168.1.102")
robot.login()
robot.power_on()
robot.enable_robot()

IO_TOOL = 1

_gripper_state = robot.get_digital_output(IO_TOOL, 0)

time.sleep(1)
print(f"Initial gripper state: {_gripper_state}")

robot.set_digital_output(IO_TOOL, 1, 1)

print("Set 1")

time.sleep(1)

robot.set_digital_output(IO_TOOL, 1, 0)

print("Set 0")