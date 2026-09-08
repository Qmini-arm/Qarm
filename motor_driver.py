import serial
import struct
import math
import time
from dataclasses import dataclass
import threading

@dataclass
class MotorCmd:
    motorType: int = 1
    mode: int = 0
    id: int = 0
    kp: float = 0.0
    kd: float = 0.0   
    q: float = 0.0    
    dq: float = 0.0   
    tau: float = 0.0
    offset: float = 0.0     # 物理零点偏移量
    direction: int = 1      # 电机方向 (1 为正，-1 为反)

@dataclass
class MotorData:
    motorType: int = 1
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    temp: int = 0
    merror: int = 0

class SerialPort:
    # 初始化
    def __init__(self, port, baudrate=4000000):
        # 尝试连接串口
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.01)
        except Exception as e:
            print(f"打开串口 {port} 失败: {e}")
            self.serial = None

    # 指令运行逻辑
    def sendRecv(self, cmd: MotorCmd, data: MotorData):
        if not self.serial or not self.serial.is_open:
            return False

        # 1. 应用方向和偏置
        raw_tau = cmd.tau * cmd.direction
        raw_omega = cmd.dq * cmd.direction
        raw_pos = (cmd.q * cmd.direction) + cmd.offset

        # 2. 打包字节流
        cmd_bytes = self._pack_motor_cmd(cmd.id, cmd.mode, raw_tau, raw_omega, raw_pos, cmd.kp, cmd.kd)
        
        # 3. 发送与接收
        self.serial.reset_input_buffer()
        self.serial.write(cmd_bytes)
        response_bytes = self.serial.read(16)

        # 4. 解包并更新传入的 data 对象
        if len(response_bytes) == 16:
            feedback = self._parse_motor_feedback(response_bytes)
            if feedback:
                # 接收后：电机原始坐标系 -> 真实坐标系
                data.q = (feedback["pos"] - cmd.offset) * cmd.direction
                data.dq = feedback["omega"] * cmd.direction
                data.tau = feedback["tau"] * cmd.direction
                data.temp = feedback["temp"]
                return True
        return False

    # 打包
    def _pack_motor_cmd(self, motor_id, mode, tau, omega, pos, kp, kw):

        #考虑传动比
        tau /= 6.33
        omega *= 6.33
        pos *= 6.33

        buf = bytearray(17) 
        buf[0] = 0xFE
        buf[1] = 0xEE
        buf[2] = (motor_id & 0x0F) | ((mode & 0x07) << 4) 
        
        tau = max(min(tau, 127.99), -127.99)
        kp = max(min(kp, 25.599), 0.0)
        kw = max(min(kw, 25.599), 0.0)

        t_set = int(tau * 256) 
        w_set = int((omega / (2 * math.pi)) * 256)
        pos_set = int((pos / (2 * math.pi)) * 32768)
        kp_set = int(kp * 1280)
        kw_set = int(kw * 1280)

        struct.pack_into('<hhiHH', buf, 3, t_set, w_set, pos_set, kp_set, kw_set)
        crc = self._crc16_ccitt(buf[:15])
        struct.pack_into('<H', buf, 15, crc)
        return bytes(buf)

    # 解包
    def _parse_motor_feedback(self, response_bytes):
        if response_bytes[0] != 0xFD or response_bytes[1] != 0xEE:
            return None
        try:
            received_crc = struct.unpack_from('<H', response_bytes, 14)[0]
        except struct.error:
            return None

        calculated_crc = self._crc16_ccitt(response_bytes[:14])
        if received_crc != calculated_crc:
            return None

        try:
            tau_int, omega_int, pos_int, temp = struct.unpack_from('<hhib', response_bytes, 3)
        except struct.error:
            return None

        tau_fbk = tau_int / 256.0
        omega_fbk = (omega_int / 256.0) * (2 * math.pi)
        pos_fbk = (pos_int / 32768.0) * (2 * math.pi)
        
        tau_fbk *= 6.33
        omega_fbk /= 6.33
        pos_fbk /= 6.33

        return {"tau": tau_fbk, "omega": omega_fbk, "pos": pos_fbk, "temp": temp}

    # 生成校验码
    def _crc16_ccitt(self, data):
        crc = 0x0000
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc



def move(ser, cmd, data, target=1.0, duration=1.0, kp=1.0, kd=0.1, tau=0.0):
    """
    非阻塞运动函数：调用后立即返回，后台线程控制电机平滑移动。
    
    参数:
        ser: SerialPort 实例
        cmd: MotorCmd 实例
        data: MotorData 实例
        target: 目标位置 (rad)
        duration: 移动耗时 (s)
        kp: 位置环比例增益
        kd: 位置环微分增益
        tau: 目标力矩 (Nm)
    """
    def _trajectory_task():
        # 1. 发送一次指令以获取当q前真实位置作为起点
        ser.sendRecv(cmd, data)
        start_pos = data.q
        start_time = time.time()
        
        # 2. 初始化控制参数
        cmd.mode = 1
        cmd.kp = kp
        cmd.kd = kd
        cmd.tau = tau
        
        # 3. 后台高频控制循环
        while True:
            t = time.time() - start_time
            
            # 到达设定时间，结束后台任务
            if t >= duration:
                cmd.q = target
                cmd.dq = 0.0
                ser.sendRecv(cmd, data)
                break
                
            # 计算五次多项式平滑曲线
            s = t / duration
            pos_factor = 10 * (s ** 3) - 15 * (s ** 4) + 6 * (s ** 5)
            vel_factor = 30 * (s ** 2) - 60 * (s ** 3) + 30 * (s ** 4)
            
            # 更新指令
            cmd.q = start_pos + (target - start_pos) * pos_factor
            cmd.dq = ((target - start_pos) / duration) * vel_factor
            
            # 发送指令并短暂休眠以控制频率
            ser.sendRecv(cmd, data)
            time.sleep(0.005) # 约 200Hz 刷新率

    # 创建并启动后台守护线程
    t = threading.Thread(target=_trajectory_task)
    t.daemon = True  # 设置为守护线程，主程序结束时它会自动退出
    t.start()

class ArmController:
    def __init__(self, serial_port):
        self.ser = SerialPort(serial_port)
        self.mt0 = MotorCmd(id=0, direction=1, offset=0.0)  # 肩部
        self.mt1 = MotorCmd(id=1, direction=1, offset=0.0)  # 肘部1
        self.mt2 = MotorCmd(id=2, direction=1, offset=0.0)  # 肘部2
        self.mt3 = MotorCmd(id=3, direction=1, offset=0.0)  # 手腕
        self.dt0 = MotorData()
        self.dt1 = MotorData()
        self.dt2 = MotorData()
        self.dt3 = MotorData()
    def get_joint_positions(self):
        """
        获取当前四个关节的角度。
        
        返回:
            joint_positions: 列表 [q0, q1, q2, q3]，单位为弧度
        """
        self.ser.sendRecv(self.mt0, self.dt0)
        self.ser.sendRecv(self.mt1, self.dt1)
        self.ser.sendRecv(self.mt2, self.dt2)
        self.ser.sendRecv(self.mt3, self.dt3)
        
        return [self.dt0.q, self.dt1.q, self.dt2.q, self.dt3.q]
    def moveJ(self, targetQ):
        """
        关节空间运动函数：将手臂移动到指定的关节角度。
        
        参数:
            targetQ: 目标关节角度列表 [q0, q1, q2, q3]，单位为弧度
        """
        if len(targetQ) != 4:
            raise ValueError("targetQ 必须包含四个关节角度")
        
        # 发送每个电机的移动指令
        move(self.ser, self.mt0, self.dt0, target=targetQ[0], duration=1.0)
        move(self.ser, self.mt1, self.dt1, target=targetQ[1], duration=1.0)
        move(self.ser, self.mt2, self.dt2, target=targetQ[2], duration=1.0)
        move(self.ser, self.mt3, self.dt3, target=targetQ[3], duration=1.0)








#-------------使用示例-------------

def main_single_motor():
    # 1. 初始化串口（请根据你的电脑修改串口号，Linux通常是 /dev/ttyUSB0）
    serial = SerialPort("/dev/ttyUSB0") 
    # 2. 初始化电机
    # 其中id为电机ID，direction是旋转方向（+1为逆时针、-1为顺时针），offset是零点位置
    mt2 = MotorCmd(id=0, direction=1, offset=3.137)
    dt2 = MotorData()

if __name__ == "__main__":
    main_single_motor()