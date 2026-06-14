'''
使用TCP/IP协议控制并采用伺服模式的版本
'''

import logging
import os
from typing import Any
import sys
import socket
import json
import time
import queue
import threading
import math

# 处理Python SDK依赖的DLL加载问题
current_dir = os.path.dirname(os.path.abspath(__file__))

# 添加到PATH
os.environ['PATH'] = current_dir + os.pathsep + os.environ.get('PATH', '')

# 添加到Python路径
sys.path.insert(0, current_dir)

# jaka IO类型
IO_CABINET  =0  #控制柜面板IO
IO_TOOL = 1     #工具IO
IO_EXTEND = 2   #扩展IO
GRIPPER_OUTPUT_INDEX = 1

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from lerobot_robot_jaka.config_jaka import JakaConfig

logger = logging.getLogger(__name__)

# 以下为新版
# 闭环限速器，在两jaka机械臂遥操作中表现良好，但是在舵机机械臂控制时抖动较大，暂时弃用
class JointClosedLoopRateLimiter:
    """关节空间闭环限速器"""

    def __init__(
        self,
        kp_far=1.0,
        kp_mid=0.6,
        kp_near=0.25,
        e_far_deg=1.0,
        e_mid_deg=0.2,
        e_stop_deg=0.08,
        e_resume_deg=0.15,
        vmax_deg_s=160.0,
        amax_deg_s2=2600.0,
        joint_num=6,
    ):
        self.kp_far = float(kp_far)
        self.kp_mid = float(kp_mid)
        self.kp_near = float(kp_near)
        self.e_far = float(e_far_deg)
        self.e_mid = float(e_mid_deg)
        self.e_stop = float(e_stop_deg)
        self.e_resume = float(e_resume_deg)
        self.joint_num = int(joint_num)
        self.vmax = [float(vmax_deg_s)] * self.joint_num
        self.amax = [float(amax_deg_s2)] * self.joint_num
        self.prev_step = [0.0] * self.joint_num
        self.in_stop_band = False

    def reset(self):
        self.prev_step = [0.0] * self.joint_num
        self.in_stop_band = False

    @staticmethod
    def _clip(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _smoothstep(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    def update(self, target, actual, dt):
        """
        target: 主臂目标关节角（绝对角）
        actual: 从臂当前关节角（绝对角）
        dt: 控制周期（秒）
        return: 下一拍从臂目标关节角（绝对角）
        """
        if target is None or actual is None:
            return None
        if len(target) != self.joint_num or len(actual) != self.joint_num:
            return None
        if dt <= 0:
            return None

        next_cmd = [0.0] * self.joint_num
        errors = [target[i] - actual[i] for i in range(self.joint_num)]
        max_abs_err = max(abs(e) for e in errors)

        # 整个机械臂统一回差死区，避免关节独立进出死区导致的两点抖动
        if self.in_stop_band:
            if max_abs_err < self.e_resume:
                self.prev_step = [0.0] * self.joint_num
                return actual.copy()
            self.in_stop_band = False

        if max_abs_err < self.e_stop:
            self.in_stop_band = True
            self.prev_step = [0.0] * self.joint_num
            return actual.copy()

        for i in range(self.joint_num):
            err = errors[i]
            abs_err = abs(err)

            # 分段增益，大误差高增益快速跟随，小误差低增益防止振荡
            if abs_err > self.e_far:
                kp = self.kp_far
            elif abs_err > self.e_mid:
                kp = self.kp_mid
            else:
                kp = self.kp_near

            raw_step = kp * err

            # 速度限幅
            max_step = self.vmax[i] * dt
            step_v = self._clip(raw_step, -max_step, max_step)

            # 加速度限幅，限制每拍步长变化
            max_step_change = self.amax[i] * dt * dt
            step = self._clip(
                step_v,
                self.prev_step[i] - max_step_change,
                self.prev_step[i] + max_step_change,
            )

            # 防止一步跨过目标，减少来回抖动
            if err > 0:
                step = min(step, err)
            else:
                step = max(step, err)

            next_cmd[i] = actual[i] + step
            self.prev_step[i] = step

        return next_cmd

# 直接约束命令输出，避免反馈基线变化把命令重新折弯，适用于舵机机械臂控制
class JointCommandStateLimiter:
    """直接约束发送命令序列本身，避免反馈基线变化把命令重新折弯。"""

    def __init__(
        self,
        kp_far=1.0,
        kp_mid=0.6,
        kp_near=0.25,
        e_far_deg=1.0,
        e_mid_deg=0.2,
        e_stop_deg=0.08,
        e_resume_deg=0.15,
        vmax_deg_s=160.0,
        amax_deg_s2=2600.0,
        deadband_deg=0.0,
        feedback_gain=0.0,
        feedback_max_step_deg=0.0,
        joint_num=6,
    ):
        self.kp_far = float(kp_far)
        self.kp_mid = float(kp_mid)
        self.kp_near = float(kp_near)
        self.e_far = float(e_far_deg)
        self.e_mid = float(e_mid_deg)
        self.e_stop = float(e_stop_deg)
        self.e_resume = float(e_resume_deg)
        self.joint_num = int(joint_num)
        self.vmax = self._expand(vmax_deg_s)
        self.amax = self._expand(amax_deg_s2)
        self.deadband = self._expand(deadband_deg)
        self.feedback_gain = self._expand(feedback_gain)
        self.feedback_max_step = self._expand(feedback_max_step_deg)
        self.prev_cmd = None
        self.prev_step = [0.0] * self.joint_num
        self.in_stop_band = False

    def _expand(self, value):
        if isinstance(value, (list, tuple)):
            if len(value) != self.joint_num:
                raise ValueError("Per-joint limiter parameter length must match joint_num")
            return [float(item) for item in value]
        return [float(value)] * self.joint_num

    def reset(self):
        self.prev_cmd = None
        self.prev_step = [0.0] * self.joint_num
        self.in_stop_band = False

    @staticmethod
    def _clip(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _smoothstep(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    def update(self, target, dt, actual=None):
        if target is None or len(target) != self.joint_num or dt <= 0:
            return self.prev_cmd.copy() if self.prev_cmd is not None else None

        if self.prev_cmd is None:
            self.prev_cmd = list(target)
            self.prev_step = [0.0] * self.joint_num
            return self.prev_cmd.copy()

        base_cmd = self.prev_cmd.copy()
        if actual is not None and len(actual) == self.joint_num:
            for index in range(self.joint_num):
                feedback_error = actual[index] - base_cmd[index]
                feedback_step = self._clip(
                    self.feedback_gain[index] * feedback_error,
                    -self.feedback_max_step[index],
                    self.feedback_max_step[index],
                )
                base_cmd[index] += feedback_step

        next_cmd = [0.0] * self.joint_num
        errors = [target[index] - base_cmd[index] for index in range(self.joint_num)]
        self.in_stop_band = False

        for index in range(self.joint_num):
            err = errors[index]
            if abs(err) < self.deadband[index]:
                next_cmd[index] = base_cmd[index]
                self.prev_step[index] *= 0.4
                continue

            abs_err = abs(err)
            if abs_err > self.e_far:
                kp = self.kp_far
            elif abs_err > self.e_mid:
                kp = self.kp_mid
            else:
                kp = self.kp_near

            raw_step = kp * err
            max_step = self.vmax[index] * dt
            step_v = self._clip(raw_step, -max_step, max_step)

            max_step_change = self.amax[index] * dt * dt
            if self.e_resume > self.e_stop and abs_err < self.e_resume:
                near_stop_ratio = (abs_err - self.e_stop) / (self.e_resume - self.e_stop)
                near_stop_scale = self._smoothstep(near_stop_ratio)
                max_step *= near_stop_scale
                max_step_change *= max(0.2, near_stop_scale)
                step_v = self._clip(step_v, -max_step, max_step)

            step = self._clip(
                step_v,
                self.prev_step[index] - max_step_change,
                self.prev_step[index] + max_step_change,
            )

            if err > 0:
                step = min(step, err)
            else:
                step = max(step, err)

            next_cmd[index] = base_cmd[index] + step
            self.prev_step[index] = step

        self.prev_cmd = next_cmd.copy()
        return next_cmd


class JointEmaSmoother:
    """关节目标坐标平滑器，主臂目标与从臂反馈的轻量去噪"""

    def __init__(self, alpha=0.7, joint_num=6):
        self.alpha = float(alpha)
        self.joint_num = int(joint_num)
        self.smoothed = None

    def reset(self):
        self.smoothed = None

    def update(self, values):
        if values is None:
            return self.smoothed
        if len(values) != self.joint_num:
            return self.smoothed
        if self.smoothed is None:
            self.smoothed = values.copy()
        else:
            self.smoothed = [
                self.alpha * values[i] + (1 - self.alpha) * self.smoothed[i]
                for i in range(self.joint_num)
            ]
        return self.smoothed.copy()


class JointControlTargetBridge:
    """在从臂控制频率下桥接短时目标平台，减少连续转动中的断续感。"""

    def __init__(self, input_deadband_deg=0.01, hold_decay=0.75, max_offset_deg=0.25, joint_num=6):
        self.joint_num = int(joint_num)
        self.input_deadband = self._expand(input_deadband_deg)
        self.hold_decay = self._expand(hold_decay)
        self.max_offset = self._expand(max_offset_deg)
        self.prev_target = None
        self.output = None
        self.velocity = [0.0] * self.joint_num

    def _expand(self, value):
        if isinstance(value, (list, tuple)):
            if len(value) != self.joint_num:
                raise ValueError("Bridge parameter length must match joint_num")
            return [float(item) for item in value]
        return [float(value)] * self.joint_num

    @staticmethod
    def _clip(value, lower, upper):
        return max(lower, min(upper, value))

    def reset(self):
        self.prev_target = None
        self.output = None
        self.velocity = [0.0] * self.joint_num

    def update(self, target, dt):
        if target is None or len(target) != self.joint_num or dt <= 0:
            return self.output.copy() if self.output is not None else None

        if self.prev_target is None or self.output is None:
            self.prev_target = list(target)
            self.output = list(target)
            self.velocity = [0.0] * self.joint_num
            return self.output.copy()

        next_output = [0.0] * self.joint_num
        for index in range(self.joint_num):
            delta = target[index] - self.prev_target[index]
            if abs(delta) > self.input_deadband[index]:
                self.velocity[index] = delta / dt
                desired = target[index]
            else:
                self.velocity[index] *= self.hold_decay[index]
                desired = self.output[index] + self.velocity[index] * dt

            next_output[index] = self._clip(
                desired,
                target[index] - self.max_offset[index],
                target[index] + self.max_offset[index],
            )

        self.prev_target = list(target)
        self.output = next_output
        return self.output.copy()


class CommandDeadbandGate:
    """发送门控，命令变化过小则不重复下发，降低末端微抖"""

    def __init__(self, deadband_deg=0.04, joint_num=6):
        self.deadband_deg = float(deadband_deg)
        self.joint_num = int(joint_num)
        self.last_sent_cmd = None

    def reset(self):
        self.last_sent_cmd = None

    def should_send(self, cmd):
        if cmd is None or len(cmd) != self.joint_num:
            return False
        if self.last_sent_cmd is None:
            self.last_sent_cmd = cmd.copy()
            return True
        cmd_change = max(abs(cmd[i] - self.last_sent_cmd[i]) for i in range(self.joint_num))
        if cmd_change < self.deadband_deg:
            return False
        self.last_sent_cmd = cmd.copy()
        return True


class JAKA_RobotController:
    def __init__(self, ip: str):
        # 机械臂连接
        self.PORT = 10001 # 机械臂通信端口
        self.ip = ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, self.PORT))
        self.sock.settimeout(0.005)  # 5ms超时，保证不阻塞

        # 机械臂位置信息
        self.joint_pos = None
        self.max_joint_speed = 160 # 关节最大速度限制，单位是度/秒，官方文档限制180，此处提高到160提升快速跟随能力
        self.servo_deadband_deg = 0.06 # 末端到位抑抖死区
        self.joint_state_period = 0.02 # 从臂后台位置轮询周期，适度降低查询压力

        # 响应队列（如果需要处理）
        self.response_queue = queue.Queue()

        # 待处理命令队列：用于匹配请求和响应
        self.pending_commands = {}  # {cmdName: response_queue}
        self.command_counter = 0

        # 并发保护：避免多线程同时send导致报文粘连/竞争
        self.send_lock = threading.Lock()
        self.request_lock = threading.Lock()
        self.servo_lock = threading.Lock()
        self.joint_pos_lock = threading.Lock()
        self.servo_log_lock = threading.Lock()

        log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "servo_logs"))
        os.makedirs(log_dir, exist_ok=True)
        log_name = f"servo_{self.ip.replace('.', '_')}_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.servo_log_path = os.path.join(log_dir, log_name)
        self.servo_log_file = open(self.servo_log_path, "a", encoding="utf-8", buffering=1)

        # 启动后台线程持续接收机械臂响应（如果需要处理响应）
        self.running = True
        self.consumer_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.consumer_thread.start()
        self.joint_pos_thread = threading.Thread(target=self._get_joint_pos_loop, daemon=True)
        self.joint_pos_thread.start()

        # 机械臂伺服控制相关
        self.control_period = 0.033  # 默认控制周期，可在进入伺服模式时覆盖
        self.servo_step_num = 4  # 默认分频，可在进入伺服模式时覆盖
        self.slave_smooth_alpha = 0.65  # 减少从臂反馈平滑带来的相位滞后
        self.cmd_send_deadband_deg = 0.01  # 缩小发送死区，避免把小步进攒成突发命令

        self.slave_smoother = JointEmaSmoother(alpha=self.slave_smooth_alpha, joint_num=6)
        self.command_bridge = JointControlTargetBridge(
            input_deadband_deg=(0.01, 0.01, 0.01, 0.008, 0.008, 0.008),
            hold_decay=(0.82, 0.78, 0.76, 0.72, 0.72, 0.72),
            max_offset_deg=(0.35, 0.25, 0.2, 0.12, 0.12, 0.12),
            joint_num=6,
        )
        self.send_gate = CommandDeadbandGate(deadband_deg=self.cmd_send_deadband_deg, joint_num=6)
        self.limiter = JointCommandStateLimiter(
                kp_far=0.75,
                kp_mid=0.5,
                kp_near=0.2,
                e_far_deg=1.5,
                e_mid_deg=0.35,
            e_stop_deg=0.12,
            e_resume_deg=0.20,
                vmax_deg_s=(60.0, 56.0, 72.0, 75.0, 75.0, 75.0), # joint2下调一档，优先减少连续饱和追赶
                amax_deg_s2=(120.0, 105.0, 150.0, 160.0, 160.0, 160.0), # joint2下调一档，优先减轻硬拽和过载
            deadband_deg=(0.03, 0.03, 0.03, 0.03, 0.03, 0.03),
                feedback_gain=(0.03, 0.03, 0.03, 0.03, 0.03, 0.03),
                feedback_max_step_deg=(0.015, 0.015, 0.015, 0.015, 0.015, 0.015),
                joint_num=6,
            )

    def _receive_loop(self):
        """后台线程：持续接收所有响应，按顺序放入队列"""
        buffer = ""
        while self.running:
            try:
                chunk = self.sock.recv(4096).decode()
                if chunk:
                    buffer += chunk
                    
                    while True:
                        start = buffer.find('{')
                        if start == -1:
                            break
                        brace_count = 0
                        for i in range(start, len(buffer)):
                            if buffer[i] == '{':
                                brace_count += 1
                            elif buffer[i] == '}':
                                brace_count -= 1
                                if brace_count == 0:
                                    # 找到完整JSON，尝试解析并放入队列
                                    resp_str = buffer[start:i+1] # 提取完整JSON字符串
                                    buffer = buffer[i+1:] # 移除已处理部分
                                    
                                    try:
                                        resp = json.loads(resp_str)
                                        cmd_name = resp.get('cmdName')
                                        
                                        # 过滤掉不需要响应的命令
                                        if cmd_name == 'servo_j':
                                            # servo_j的响应直接丢弃，不放入队列
                                            pass
                                        elif cmd_name == 'get_actual_joint_pos':
                                            # get_actual_joint_pos响应直接更新关节位置属性
                                            with self.joint_pos_lock:
                                                self.joint_pos = resp.get('position')
                                        else:
                                            # 其他命令的响应放入队列
                                            self.response_queue.put(resp)
                                    except json.JSONDecodeError:
                                        pass
                                    break
                        else:
                            break
            except socket.timeout:
                continue
            except OSError as e:
                if not self.running:
                    break
                print(f"Receive error: {e}")
                time.sleep(0.001)
            except Exception as e:
                if not self.running:
                    break
                print(f"Receive error: {e}")
                time.sleep(0.001)

    def _get_joint_pos_loop(self):
        """后台线程：持续获取关节位置更新属性"""
        while self.running:
            self._get_actual_joint_pos() # 发送获取关节位置命令但不等待响应，确保持续获取关节位置更新
            time.sleep(self.joint_state_period)

    def _send_with_response(self, command, timeout=0.05):
        """发送命令并等待响应（顺序匹配）"""
        cmd_name = command.get("cmdName")
        deadline = time.time() + timeout

        # 同一连接上串行请求，避免响应错配
        with self.request_lock:
            with self.send_lock:
                self.sock.send(json.dumps(command).encode())

            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    print(f"命令 {cmd_name} 超时")
                    return {
                        "cmdName": cmd_name,
                        "errorCode": "-1",
                        "errorMsg": "timeout"
                    }

                try:
                    response = self.response_queue.get(timeout=remaining)
                except queue.Empty:
                    print(f"命令 {cmd_name} 超时")
                    return {
                        "cmdName": cmd_name,
                        "errorCode": "-1",
                        "errorMsg": "timeout"
                    }

                # 仅接收当前命令对应响应，其他响应忽略
                if response.get("cmdName") == cmd_name:
                    return response
    
    def send_command_no_recv(self, command):
        """发送命令但不接受响应（不等待，不生成ID）"""
        try:
            with self.send_lock:
                self.sock.send(json.dumps(command).encode())
            return True
        except Exception as e:
            print(f"Send failed: {e}")
            return False

    # 获取关节实际位置，该函数会发送获取关节位置的命令但不等待响应，确保持续获取关节位置更新，并直接返回属性值
    def _get_actual_joint_pos(self):
        cmd = {"cmdName":"get_actual_joint_pos"}
        self.send_command_no_recv(cmd) # 发送命令但不等待响应，确保持续获取关节位置更新

    # 获取关节实际位置，该函数直接读取属性值
    def get_actual_joint_pos(self):
        # 直接返回属性值，可能是None（如果还没收到过响应）
        with self.joint_pos_lock:
            return self.joint_pos.copy() if self.joint_pos is not None else None

    def _log_servo_command(self, rel_flag, joint_position, step_num):
        if self.servo_log_file is None or joint_position is None:
            return

        record = {
            "timestamp": time.time(),
            "ip": self.ip,
            "relFlag": rel_flag,
            "stepNum": step_num,
            "jointPosition": list(joint_position),
        }
        with self.servo_log_lock:
            self.servo_log_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _close_servo_log(self):
        with self.servo_log_lock:
            if self.servo_log_file is not None:
                self.servo_log_file.flush()
                self.servo_log_file.close()
                self.servo_log_file = None

    def stop(self, join_timeout=0.2):
        """停止后台线程并安全关闭socket"""
        self.running = False

        # 先shutdown再close，唤醒可能阻塞在recv的线程
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass

        try:
            self.sock.close()
        except Exception:
            pass

        # 等待线程结束
        for t in (getattr(self, "consumer_thread", None), getattr(self, "joint_pos_thread", None)):
            if t is not None and t.is_alive():
                t.join(timeout=join_timeout)

        self._close_servo_log()
        
    # 获取末端实际位姿
    def get_tcp_pose(self):
        cmd = {"cmdName":"get_tcp_pos"}
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return response_data.get("tcp_pos")
        else:
            print("获取末端位姿失败:", response_data.get("errorMsg"))
            return None
        
    # 设置数字输出变量
    def set_digital_output(self, type, index, value):
        cmd = {
            "cmdName": "set_digital_output",
            "type": type,
            "index": index,
            "value": value
            }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("设置数字输出失败:", response_data.get("errorMsg"))
            return False

    # 获取数字输出变量，注意索引从1开始
    def get_digital_output(self, type, index):
        cmd = {
            "cmdName": "get_digital_output",
            "type": type,
            "index": index
            }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return response_data.get("value")
        else:
            print("获取数字输出失败:", response_data.get("errorMsg"))
            return None 

    # 机械臂初始化（上电并使能）
    def robot_init(self):
        power_on_result = False
        enable_result = False

        # 上电
        cmd = {"cmdName": "power_on"}
        response = self._send_with_response(cmd,timeout=10) # 上电指令可能需要较长时间，适当增加超时时间
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            power_on_result = True
        else:
            print("上电失败:", response_data.get("errorMsg"))
        time.sleep(8) # 此处时间需要尽可能长，确保机械臂完全上电完成，否则会使能失败但返回成功，导致后续指令都无响应

        # 使能
        cmd = {"cmdName":"enable_robot"}
        response = self._send_with_response(cmd,timeout=10) # 使能指令可能需要较长时间，适当增加超时时间
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            enable_result = True
        else:
            print("使能失败:", response_data.get("errorMsg"))
        time.sleep(3)
        return power_on_result and enable_result

    # 机械臂关节运动到指定位置
    def joint_move(self, target_pos, speed=50, accel=500):
        '''
        target_pos: 代表目标关节位置，单位是度，不是弧度，运动的正负由jointPosition值的正负来确定
        speed: 代表关节的速度，单位是 (°/S)，用户可以自行填入适合的参数
        accel: 代表关节的加速度，单位是 (°/S²)，用户可以自行填入适合的参数，加速度的值建议不要超过720
        '''
        cmd = {
            "cmdName": "joint_move",
            "relFlag":0, # 默认为绝对运动
            "jointPosition": target_pos,
            "speed": speed,
            "accel": accel
        }
        response = self._send_with_response(cmd, timeout=10) # 关节运动可能需要较长时间，适当增加超时时间
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("关节运动失败:", response_data.get("errorMsg"))
            return False
        
    # 机械臂末端运动到指定位姿（阻塞）
    def end_move(self, target_pose, speed=50, accel=500):
        '''
        target_pose: 代表目标末端位姿，指定TCP末端xyzabc的值
        speed: 代表末端的速度，单位是 (mm/s)，用户可以自行填入适合的参数
        accel: 代表末端的加速度，单位是 (mm/s²)，用户可以自行填入适合的参数，加速度的值建议不要超过2000
        该命令是阻塞的,end_move指令并不是从当前位置直线运动到目标位置点，
        这条指令是先对用户输入的笛卡尔空间目标点进行逆解，然后使用joint_move指令，让机器人关节运动到指定位置。
        '''
        cmd = {
            "cmdName": "end_move",
            "endPosition": target_pose,
            "speed": speed,
            "accel": accel
        }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("末端运动失败:", response_data.get("errorMsg"))
            return False
        
    # 机械臂末端直线运动
    def moveL(self, target_pose, speed=50, accel=500):
        '''
        target_pose: 代表目标末端位姿，指定TCP末端xyzabc的值
        speed: 代表末端的速度，单位是 (mm/s)，用户可以自行填入适合的参数
        accel: 代表末端的加速度，单位是 (mm/s²)，用户可以自行填入适合的参数，加速度的值建议不要超过2000
        该命令是阻塞的, moveL指令会让机器人末端沿直线运动到目标位置。
        '''
        cmd = {
            "cmdName": "moveL",
            "relFlag":0, # 默认为绝对运动
            "cartPosition": target_pose,
            "speed": speed,
            "accel": accel
        }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("末端直线运动失败:", response_data.get("errorMsg"))
            return False

    # 查询机械臂是否在伺服模式中
    def is_in_servomove(self):
        cmd = {"cmdName": "is_in_servomove"}
        response = self._send_with_response(cmd)
        response_data = response
        if response_data.get("errorCode") == "0":
            return response_data.get("in_servomove")
        else:
            print("查询运动状态失败:", response_data.get("errorMsg"))
            return None
        
    # 机械臂伺服位置控制使能
    def servo_move(self, relFlag, is_block=0):
        '''
        relFlag: 1代表进入servo_move模式，0代表退出
        is_block: 0代表非阻塞模式，1代表阻塞模式，默认为非阻塞模式
        '''
        if(relFlag != 0 and relFlag != 1):
            print("enable参数必须为0或1")
            return
        # 这里官方文档控制器版本1.7.2无is_block参数，3.0版本可添加
        cmd = {
            "cmdName": "servo_move",
            "relFlag": relFlag
            }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("伺服模式控制使能失败:", response_data.get("errorMsg"))
            return False
    
    # 内部函数：机械臂伺服位置控制（相对位置）
    def _servo_j(self, jointPosition, stepNum = 1):
        '''
        jointPosition: 各个关节的角度，单位是度
        stepNum: 是周期分频，机器人将以num*8ms的周期执行接收到的servo运动指令（目前测试将此调大可以降低速度）

        需要注意的是，由于控制器的控制周期是8ms，这个指令需要8ms发送一次才能有效果，
        而且需要连续的发才行，只发一次看不出效果，并且六个关节的最大速度是180度/秒。
        例如[1.5,0.5,0.5,0.5,0.5,0.5]，1.5/0.008= 187.5超出了关节速度限制, 那么servo_j指令就不会生效
        '''
        cmd = {
            "cmdName": "servo_j",
            "relFlag":1, # 默认为相对运动
            "jointPosition": jointPosition,
            "stepNum": stepNum
        }
        if self.send_command_no_recv(cmd):
            self._log_servo_command(rel_flag=1, joint_position=jointPosition, step_num=stepNum)

    # 内部函数：机械臂伺服位置控制（绝对位置）
    def _servo_j_abs(self, jointPosition, stepNum=1):
        cmd = {
            "cmdName": "servo_j",
            "relFlag": 0, # 绝对运动
            "jointPosition": jointPosition,
            "stepNum": stepNum
        }
        if self.send_command_no_recv(cmd):
            self._log_servo_command(rel_flag=0, joint_position=jointPosition, step_num=stepNum)

    # 内部函数：同步插值发送
    def _sync_interpolate(self, delta, steps):
        step_delta = [delta[i] / steps for i in range(6)]
        for step in range(steps):
            step_start = time.perf_counter()
            self._servo_j(step_delta, stepNum=1)

            if step < steps - 1:
                # 忙等待
                while time.perf_counter() - step_start < 0.008: pass

    # 外部接口：机械臂伺服位置控制并进行插值，不接收返回，jointPosition单位是度
    def servo_j(self, jointPosition):
        # 获取当前位置
        current_pos = self.get_actual_joint_pos()
        if current_pos is None:
            print("无法获取当前关节位置，无法执行servo_j指令")
            return
        # 机械臂单次servo_j最大角度
        max_delta = self.max_joint_speed * 0.008 # 8ms周期，单位是度
        max_reachable = max_delta * 6
        raw_delta = [jointPosition[i] - current_pos[i] for i in range(6)]

        # 小于死区不发增量，避免目标附近来回抖动
        if max(abs(d) for d in raw_delta) < self.servo_deadband_deg:
            return

        # 变化过大时进行限幅
        delta = [max(-max_reachable, min(max_reachable, d)) for d in raw_delta]
        steps = max(1, math.ceil(max(abs(delta[i]) / max_delta for i in range(6))))
        steps = min(steps, 6)

        # 串行执行插值
        with self.servo_lock:
            self._sync_interpolate(delta, steps)

    # 外部接口：用于闭环限速模式的绝对servo_j（单周期发送，避免二次插值引入相位抖动）
    def servo_j_abs(self, jointPosition, stepNum=6):
        if jointPosition is None:
            return
        self._servo_j_abs(jointPosition, stepNum=stepNum)

    # 机械臂设置伺服模式一阶低通滤波器
    def set_servo_lpf(self, lpf):
        '''
        设置关节空间一阶低通滤波器，需要退出servo模式后设置
        lpf: 代表伺服模式的一阶低通滤波器参数，单位是Hz
        '''
        cmd = {
            "cmdName": "set_servo_move_filter",
            "filter_type": 1,
            "lpf_cf": lpf
        }
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("设置伺服模式一阶低通滤波器失败:", response_data.get("errorMsg"))
            return False

    # 机械臂禁用滤波器
    def set_no_filter(self):
        cmd = {"cmdName":"set_servo_move_filter","filter_type":0}
        response = self._send_with_response(cmd)
        response_data = response
        # 状态码为0表示成功
        if response_data.get("errorCode") == "0":
            return True
        else:
            print("禁用滤波器失败:", response_data.get("errorMsg"))
            return False

    # 下使能并关闭电源
    def disconnect(self):
        # 下使能
        cmd = {"cmdName":"disable_robot"}
        self.send_command_no_recv(cmd)
        time.sleep(0.5)
        # 清空可能的响应
        try:
            self.sock.recv(4096)
        except:
            pass
        # 关闭电源
        cmd = {"cmdName":"power_off"}
        self.send_command_no_recv(cmd)
        time.sleep(0.5)
        # 清空可能的响应
        try:
            self.sock.recv(4096)
        except:
            pass

        self.stop()

        return True # 此函数不抛出异常默认返回True

    # 关闭控制器进程，关闭socket连接
    def close(self):
        # 该指令无返回
        cmd = {"cmdName":"shutdown"}
        self.send_command_no_recv(cmd)
        self.stop()

        return True # 此函数默认返回True

# 夹爪类 目前仅支持简单的开合控制
class JakaGripper:
    def __init__(self, arm: JAKA_RobotController):
        self._arm = arm
        self._prev_gripper_state = None
        self._gripper_state = None
        self._refresh_gripper_state()  # 初始状态从IO读取
        # 未启用
        self._gripper_open_time = 0.0
        self._gripper_stopped = False

    @staticmethod
    def _normalize_gripper_state(gripper_state: float | int | None) -> float | None:
        if gripper_state is None:
            return None
        return 1.0 if float(gripper_state) >= 0.5 else 0.0

    def _refresh_gripper_state(self) -> float | None:
        io_value = self._arm.get_digital_output(IO_TOOL, GRIPPER_OUTPUT_INDEX)
        normalized = self._normalize_gripper_state(io_value)
        if normalized is not None:
            self._gripper_state = normalized
        return self._gripper_state

    def open(self):
        if self._arm.set_digital_output(IO_TOOL, GRIPPER_OUTPUT_INDEX, 1):
            self._gripper_state = 1.0
            return True
        self._refresh_gripper_state()
        return False

    def close(self):
        if self._arm.set_digital_output(IO_TOOL, GRIPPER_OUTPUT_INDEX, 0):
            self._gripper_state = 0.0
            return True
        self._refresh_gripper_state()
        return False

    def stop(self):
        pass

    def set_gripper_state(self, gripper_state: float) -> None:
        # gripper_state小于0.5则关闭夹爪，大于等于0.5则打开夹爪
        requested_state = self._normalize_gripper_state(gripper_state)
        current_state = self._gripper_state

        # 热路径只看本地已确认缓存，避免每次set都阻塞读取IO。
        # 初始化或写失败时才会刷新实际IO状态。
        if requested_state is not None and current_state is not None:
            if requested_state == current_state:
                return

        if requested_state is not None:
            if requested_state < 0.5:
                self.close()
            else:
                self.open()

        self._prev_gripper_state = self._gripper_state

    def get_gripper_state(self) -> float:
        return self._gripper_state

    def reset_gripper(self) -> None:
        self._prev_gripper_state = None
        self._gripper_state = 0.0
        self._gripper_open_time = 0.0
        self._gripper_stopped = False


class Jaka(Robot):
    '''
    从臂
    '''
    config_class = JakaConfig
    name = "jaka"

    def __init__(self, config: JakaConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        self.config = config
        self._is_connected = False
        self._arm = None
        self._gripper = None
        self._initial_pose = None
        self._prev_observation = None
        self.is_servo_mode = False # 记录当前是否在伺服模式

    def connect(self, calibrate: bool = True) -> None:
        # 连接机械臂
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        # 连接jaka机械臂并使能
        self._arm = JAKA_RobotController(self.config.ip)
        print("连接jaka机械臂：", self.config.ip)
        self._arm.robot_init()

        # 连接时先退出伺服模式
        self._arm.servo_move(relFlag=0)
        time.sleep(0.1)

        # self._arm = None # 调试需要

        # 初始化夹爪
        self._gripper = JakaGripper(self._arm)

        for cam in self.cameras.values():
            cam.connect()

        self.is_connected = True
        self.configure()
        logger.info(f"{self} connected.")

    @property
    def _motors_ft(self) -> dict[str, type]:
        motors = {f"joint{i}.pos": float for i in range(1, 7)}

        motors["gripper.pos"] = float
        return motors   

    @property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft
    
    def init_servo_mode(self, lpf=30, control_freq=None, servo_step_num=None):
        if control_freq is not None and control_freq > 0:
            self._arm.control_period = 1.0 / float(control_freq)
        if servo_step_num is not None:
            self._arm.servo_step_num = max(1, int(servo_step_num))
        elif control_freq is not None and control_freq > 0:
            self._arm.servo_step_num = max(1, round(self._arm.control_period / 0.008))

        self._arm.limiter.reset()
        self._arm.slave_smoother.reset()

        self._arm.servo_move(relFlag=0)
        time.sleep(0.1)
        
        # 设置滤波器
        self._arm.set_servo_lpf(lpf)
        # self._arm.set_no_filter()
        time.sleep(0.1)

        # 进入伺服模式
        self._arm.servo_move(relFlag=1)
        time.sleep(0.1)

        self.is_servo_mode = True
    
    # 将位姿字典转换为元组，适配jaka SDK
    def dict2tuple(self, pose: dict[str, float]) -> tuple[float, float, float, float, float, float]:
        return (
            pose['x'],
            pose['y'],
            pose['z'],
            pose['rx'],
            pose['ry'],
            pose['rz']
        )

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if "delta_x" in action and "delta_y" in action and "delta_z" in action:
            # 获取当前末端位姿
            print("Received delta pose action: ", action)
            current_pose = self._arm.get_tcp_pose()
            # 计算新位姿
            new_pose = {
                'x': current_pose[0] + action["delta_x"],
                'y': current_pose[1] + action["delta_y"],
                'z': current_pose[2] + action["delta_z"],
                'rx': current_pose[3],  # 如果没有旋转delta，保持原样
                'ry': current_pose[4],
                'rz': current_pose[5]
            }

            # 处理旋转delta
            if "delta_roll" in action and "delta_pitch" in action and "delta_yaw" in action:
                new_pose['rx'] = current_pose[3] + action["delta_roll"]
                new_pose['ry'] = current_pose[4] + action["delta_pitch"]
                new_pose['rz'] = current_pose[5] + action["delta_yaw"]
            
            action["pose"] = new_pose

        # 处理home位置
        if "home" in action:
            action["pose"] = {
                'x': self.config.home_translation[0],
                'y': self.config.home_translation[1],
                'z': self.config.home_translation[2],
                'rx': self.config.home_orientation_euler[0],
                'ry': self.config.home_orientation_euler[1],
                'rz': self.config.home_orientation_euler[2]
            }
            # 记录初始位姿
            self._initial_pose = self._arm.get_tcp_pose()

        # 执行位姿控制
        if "pose" in action:
            pose = action["pose"]
            self._arm.end_move(self.dict2tuple(pose)) # 阻塞移动

            # 获取位置用于反馈
            joint_positions = self._arm.get_actual_joint_pos()
            for i in range(1, 7):
                action[f"joint{i}.pos"] = joint_positions[i-1]

        # 关节运动（伺服模式）
        if "joint1.pos" in action:
            joint_positions = []
            for i in range(1, 7):
                joint_pos = action[f"joint{i}.pos"]
                joint_positions.append(joint_pos)
            cmd_position = self._arm.limiter.update(joint_positions, self._arm.control_period)

            if cmd_position is not None:
                self._arm.servo_j_abs(cmd_position, stepNum=self._arm.servo_step_num)
            # 获取位置用于反馈（直接读取当前位置）
            real_positions = self._arm.get_actual_joint_pos()
            for i in range(1, 7):
                action[f"joint{i}.pos"] = real_positions[i-1]

        # 关节运动（普通模式）
        if "joint1.pos_normal" in action:
            joint_positions = []
            for i in range(1, 7):
                joint_pos = action[f"joint{i}.pos_normal"]
                joint_positions.append(joint_pos)
            result = self._arm.joint_move(joint_positions, speed=100, accel=500) # 非阻塞运动，速度和加速度可以调整
            # 获取位置用于反馈
            if result:
                for i in range(1, 7):
                    action[f"joint{i}.pos_normal"] = joint_positions[i-1]


        # 夹爪
        if "gripper.pos" in action or "gripper" in action:
            gripper_pos = action.get("gripper.pos", action.get("gripper", 0.0))
            self._gripper.set_gripper_state(gripper_pos) if self._gripper else None
            action["gripper.pos"] = self._gripper.get_gripper_state() if self._gripper else None

        return action

    def get_observation(self) -> dict[str, Any]:
        # start = time.time()
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        obs_dict = {}
        
        joint_positions = self._arm.get_actual_joint_pos()
        # # 调试需要
        # ret = 0
        # joint_positions = demo_joint_positions
        # t1 = time.time() - start
        if joint_positions is None:
            raise DeviceNotConnectedError("Failed to get joint positions")
        
        obs_dict = {f"joint{i+1}.pos": joint_positions[i] for i in range(6)}
        # 暂时不设置爪子
        if self._gripper:
            obs_dict["gripper.pos"] = self._gripper.get_gripper_state()

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        self._prev_observation = obs_dict
        return obs_dict

    def reset(self):
        if self._gripper:
            self._gripper.reset_gripper()

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._gripper is not None:
            self._gripper.stop()
            self._gripper = None

        if self._arm is not None:
            self._arm.disconnect() # 发送断开指令
            self._arm = None

        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info(f"{self} disconnected.")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # 获取并记录初始位姿
        self._initial_pose = self._arm.get_tcp_pose()
        # # 调试需要
        # ret = 0
        # self._initial_pose = demo_joint_positions
        if(self._initial_pose is None):
            raise DeviceNotConnectedError(f"Failed to get initial pose from {self}")
        # print(f"Initial pose: {self._initial_pose}")

    def is_calibrated(self) -> bool:
        return self.is_connected

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        features = {**self._motors_ft, **self._cameras_ft}
        if self.config.use_effort:
            for i in range(1, 7):
                features[f"joint{i}.effort"] = float
        if self.config.use_velocity:
            for i in range(1, 7):
                features[f"joint{i}.vel"] = float
        if self.config.use_acceleration:
            for i in range(1, 7):
                features[f"joint{i}.acc"] = float
        return features

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value