import serial
import struct
import math
import time
from dataclasses import dataclass
import threading
from pathlib import Path
import xml.etree.ElementTree as ET
import json

from joint_trajectory import JointTrajectory

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
        self._lock = threading.Lock()
        # 尝试连接串口
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.01)
        except Exception as e:
            print(f"打开串口 {port} 失败: {e}")
            self.serial = None

    # 指令运行逻辑
    def sendRecv(self, cmd: MotorCmd, data: MotorData):
        # A command and its 16-byte response are one transaction.  The lock
        # prevents another motor (or a legacy caller) from interleaving bytes.
        with self._lock:
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

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

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
    """Four-axis controller with one serialized polling loop per serial port."""

    MOTOR_COUNT = 4

    def __init__(self, serial_port, urdf_path=None, directions=None, offsets=None):
        self.ser = SerialPort(serial_port)
        directions = directions or [1] * self.MOTOR_COUNT
        offsets = offsets or [0.0] * self.MOTOR_COUNT
        if len(directions) != self.MOTOR_COUNT or len(offsets) != self.MOTOR_COUNT:
            raise ValueError("directions 和 offsets 必须包含四个关节")
        self.motors = [MotorCmd(id=i, direction=directions[i], offset=offsets[i])
                       for i in range(self.MOTOR_COUNT)]
        self.feedback = [MotorData() for _ in range(self.MOTOR_COUNT)]
        # Compatibility with older code that addressed motors as mt0..mt3.
        for index, (motor, data) in enumerate(zip(self.motors, self.feedback)):
            setattr(self, f"mt{index}", motor)
            setattr(self, f"dt{index}", data)
        self.joint_limits = self._load_joint_limits(urdf_path)
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._motion_thread = None
        # Browser controls must not start a new MoveJ thread for every slider
        # event.  The streaming worker owns the serial cycle and only its
        # latest target is replaced by the UI.
        self._stream_lock = threading.RLock()
        self._stream_stop_event = threading.Event()
        self._stream_thread = None
        self._stream_generation = 0
        self._stream_target = None

    @staticmethod
    def _load_joint_limits(urdf_path=None):
        path = Path(urdf_path) if urdf_path else Path(__file__).resolve().parent / "description" / "qmini_arm.urdf"
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise ValueError(f"无法读取 URDF 限位文件 {path}: {exc}") from exc
        limits = []
        for index in range(1, ArmController.MOTOR_COUNT + 1):
            joint = root.find(f"./joint[@name='joint_{index}']")
            limit = joint.find("limit") if joint is not None else None
            if joint is None or limit is None or limit.get("lower") is None or limit.get("upper") is None:
                raise ValueError(f"URDF 缺少 joint_{index} 的 lower/upper 限位")
            limits.append((float(limit.get("lower")), float(limit.get("upper"))))
        return tuple(limits)

    @property
    def limits(self):
        """返回按 joint_1..joint_4 排列的 (lower, upper)，单位为弧度。"""
        return self.joint_limits

    def _validate_target(self, targetQ):
        if len(targetQ) != self.MOTOR_COUNT:
            raise ValueError("targetQ 必须包含四个关节角度")
        for index, (value, (lower, upper)) in enumerate(zip(targetQ, self.joint_limits)):
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"joint_{index + 1} 目标角度不是有限数值: {value}")
            if value < lower or value > upper:
                raise ValueError(
                    f"joint_{index + 1} 目标角度 {value:.6f} 超出 URDF 限位 [{lower:.6f}, {upper:.6f}]")
        return [float(value) for value in targetQ]
    def get_joint_positions(self):
        """
        获取当前四个关节的角度。
        
        返回:
            joint_positions: 列表 [q0, q1, q2, q3]，单位为弧度
        """
        for motor, data in zip(self.motors, self.feedback):
            self.ser.sendRecv(motor, data)  # 串口轮询顺序固定为 id 0..3
        with self._state_lock:
            return [data.q for data in self.feedback]

    def send_joint_command(self, targetQ, torques=None, kp=None, kd=None, dq=None):
        """Send one serialized command cycle to all four motors.

        This is the synchronous primitive for callers that need to update
        torque compensation and position targets continuously.  Motor IDs are
        always serviced in ascending order on the shared serial port.
        """
        targetQ = self._validate_target(targetQ)
        torques = [0.0] * self.MOTOR_COUNT if torques is None else list(torques)
        dq = [0.0] * self.MOTOR_COUNT if dq is None else list(dq)
        if len(torques) != self.MOTOR_COUNT or len(dq) != self.MOTOR_COUNT:
            raise ValueError("torques 和 dq 必须包含四个关节值")
        kp_values = [1.0] * self.MOTOR_COUNT if kp is None else list(kp)
        kd_values = [0.1] * self.MOTOR_COUNT if kd is None else list(kd)
        if len(kp_values) != self.MOTOR_COUNT or len(kd_values) != self.MOTOR_COUNT:
            raise ValueError("kp 和 kd 必须包含四个关节值")
        self.stop_streaming()
        self.stop_motion()
        for index, motor in enumerate(self.motors):
            motor.mode = 1
            motor.q = targetQ[index]
            motor.dq = float(dq[index])
            motor.tau = float(torques[index])
            motor.kp = float(kp_values[index])
            motor.kd = float(kd_values[index])
        for motor, data in zip(self.motors, self.feedback):
            self.ser.sendRecv(motor, data)
        with self._state_lock:
            return [data.q for data in self.feedback]

    @staticmethod
    def _sample_stream_target(position, velocity, target, elapsed, response_time):
        """Advance a critically damped target filter by ``elapsed`` seconds.

        The filter keeps its position and velocity state when the browser
        target changes.  Unlike restarting a point-to-point polynomial for
        every slider event, it cannot inject a new start-position or velocity
        discontinuity into the motor command stream.
        """
        if elapsed <= 0.0:
            return list(position), list(velocity)

        # At the requested response time the critically damped residual is
        # below 2%, so the UI duration remains a useful response-time control.
        omega = 6.0 / max(response_time, 1e-6)
        decay = math.exp(-omega * elapsed)
        positions, velocities = [], []
        for current, current_velocity, goal in zip(position, velocity, target):
            error = current - goal
            coefficient = current_velocity + omega * error
            next_error = (error + coefficient * elapsed) * decay
            next_velocity = (
                current_velocity - omega * coefficient * elapsed
            ) * decay
            positions.append(goal + next_error)
            velocities.append(next_velocity)
        return positions, velocities

    def start_streaming(self, targetQ, duration=0.5, kp=None, kd=None,
                        control_period=0.005):
        """Start one persistent, feedback-driven target stream.

        ``update_stream_target`` can then replace the target without stopping
        or restarting the worker.  This is intended for interactive clients
        such as Viser, where a slider can produce many updates during one
        physical movement.
        """
        target = self._validate_target(targetQ)
        duration = float(duration)
        control_period = float(control_period)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration 必须是大于 0 的有限数值")
        if not math.isfinite(control_period) or control_period <= 0.0:
            raise ValueError("control_period 必须是大于 0 的有限数值")
        kp_values = [1.0] * self.MOTOR_COUNT if kp is None else [float(v) for v in kp]
        kd_values = [0.1] * self.MOTOR_COUNT if kd is None else [float(v) for v in kd]
        if len(kp_values) != self.MOTOR_COUNT or len(kd_values) != self.MOTOR_COUNT:
            raise ValueError("kp 和 kd 必须包含四个关节值")
        if not all(math.isfinite(v) for v in kp_values + kd_values):
            raise ValueError("kp 和 kd 必须是有限数值")

        # Only one owner may use the serial port.  The old MoveJ worker is
        # stopped before the stream reads its initial feedback pose.
        self.stop_streaming()
        self.stop_motion()
        self.get_joint_positions()
        with self._state_lock:
            start = [float(data.q) for data in self.feedback]

        with self._stream_lock:
            self._stream_generation += 1
            generation = self._stream_generation
            stop_event = threading.Event()
            self._stream_stop_event = stop_event
            self._stream_target = list(target)
            self._stream_thread = threading.Thread(
                target=self._stream_task,
                args=(
                    start,
                    target,
                    duration,
                    kp_values,
                    kd_values,
                    control_period,
                    stop_event,
                    generation,
                ),
                name="qmini-arm-stream",
                daemon=True,
            )
            self._stream_thread.start()

    def update_stream_target(self, targetQ):
        """Replace the active stream target without restarting its worker."""
        target = self._validate_target(targetQ)
        with self._stream_lock:
            thread = self._stream_thread
            if thread is None or not thread.is_alive() or self._stream_stop_event.is_set():
                raise RuntimeError("实机目标流尚未启动")
            self._stream_target = list(target)

    def _stream_task(self, start, target, duration, kp_values,
                     kd_values, control_period, stop_event, generation):
        from gravity import calc_compensated_torque

        command_q = list(start)
        command_dq = [0.0] * self.MOTOR_COUNT
        last_update = time.monotonic()
        next_deadline = last_update

        try:
            for index, motor in enumerate(self.motors):
                motor.mode = 1
                motor.kp = kp_values[index]
                motor.kd = kd_values[index]

            while not stop_event.is_set():
                with self._stream_lock:
                    if generation != self._stream_generation:
                        return
                    latest_target = list(self._stream_target)

                now = time.monotonic()
                elapsed = min(max(now - last_update, 0.0), 0.25)
                command_q, command_dq = self._sample_stream_target(
                    command_q, command_dq, latest_target, elapsed, duration
                )
                last_update = now
                # A feedback sample can be just outside the URDF interval at
                # startup.  Keep every generated stream command inside the
                # same limits used by the public target validator.
                for index, (lower, upper) in enumerate(self.joint_limits):
                    bounded = min(max(command_q[index], lower), upper)
                    if bounded != command_q[index]:
                        command_q[index] = bounded
                        command_dq[index] = 0.0
                tau1, tau2, tau3 = calc_compensated_torque(
                    self.feedback[1], self.feedback[2], self.feedback[3]
                )
                torques = (0.0, tau1, tau2, tau3)
                for index, (motor, data) in enumerate(zip(self.motors, self.feedback)):
                    motor.q = command_q[index]
                    motor.dq = command_dq[index]
                    motor.tau = torques[index]
                    self.ser.sendRecv(motor, data)

                next_deadline += control_period
                wait_time = next_deadline - time.monotonic()
                if wait_time < -control_period:
                    next_deadline = time.monotonic()
                    wait_time = 0.0
                stop_event.wait(max(0.0, wait_time))
        finally:
            with self._stream_lock:
                if self._stream_thread is threading.current_thread():
                    self._stream_thread = None

    def stop_streaming(self):
        """Stop the persistent target stream and wait for its serial cycle."""
        with self._stream_lock:
            thread = self._stream_thread
            stop_event = self._stream_stop_event
            self._stream_generation += 1
            stop_event.set()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._stream_lock:
            if self._stream_thread is thread and (thread is None or not thread.is_alive()):
                self._stream_thread = None

    def disable(self):
        """Disable all motors with one final serialized polling cycle."""
        self.stop_streaming()
        self.stop_motion()
        for motor in self.motors:
            motor.mode = 0
            motor.dq = motor.tau = 0.0
        for motor, data in zip(self.motors, self.feedback):
            self.ser.sendRecv(motor, data)

    def moveJ(self, targetQ, duration=1.0, kp=None, kd=None):
        """
        关节空间运动函数：将手臂移动到指定的关节角度。
        
        参数:
            targetQ: 目标关节角度列表 [q0, q1, q2, q3]，单位为弧度
        """
        targetQ = self._validate_target(targetQ)
        if duration <= 0:
            raise ValueError("duration 必须大于 0")
        kp = [1.0] * self.MOTOR_COUNT if kp is None else list(kp)
        kd = [0.1] * self.MOTOR_COUNT if kd is None else list(kd)
        if len(kp) != self.MOTOR_COUNT or len(kd) != self.MOTOR_COUNT:
            raise ValueError("kp 和 kd 必须包含四个关节值")
        self.stop_streaming()
        self.stop_motion()

        def _trajectory_task():
            # Lazy import avoids the gravity -> motor_driver import cycle.
            from gravity import calc_compensated_torque

            self.get_joint_positions()
            with self._state_lock:
                start = [data.q for data in self.feedback]
            start_time = time.monotonic()
            for index, motor in enumerate(self.motors):
                motor.mode, motor.kp, motor.kd = 1, kp[index], kd[index]
            while not self._stop_event.is_set():
                elapsed = time.monotonic() - start_time
                s = min(elapsed / duration, 1.0)
                factor = 10 * s**3 - 15 * s**4 + 6 * s**5
                velocity_factor = 30 * s**2 - 60 * s**3 + 30 * s**4
                tau1, tau2, tau3 = calc_compensated_torque(
                    self.feedback[1], self.feedback[2], self.feedback[3]
                )
                torques = (0.0, tau1, tau2, tau3)
                for index, (motor, data) in enumerate(zip(self.motors, self.feedback)):
                    motor.q = start[index] + (targetQ[index] - start[index]) * factor
                    motor.dq = (targetQ[index] - start[index]) / duration * velocity_factor
                    motor.tau = torques[index]
                    self.ser.sendRecv(motor, data)
                if s >= 1.0:
                    break
                self._stop_event.wait(0.005)
        self._stop_event.clear()
        self._motion_thread = threading.Thread(target=_trajectory_task, name="qmini-arm-motion", daemon=True)
        self._motion_thread.start()

    def wait_motion(self, timeout=None):
        """等待当前 MoveJ 完成；返回是否在超时前完成。"""
        thread = self._motion_thread
        if thread is None:
            return True
        thread.join(timeout=timeout)
        return not thread.is_alive()

    def record_trajectory(self, path=None, duration=None, sample_period=0.02,
                          kp=None, kd=None, stop_on_interrupt=True):
        """拖动示教并采样关节轨迹。

        示教采用零位置刚度（kp=0）、小阻尼和实时重力补偿，操作者可以直接
        拖动机械臂。轨迹文件为 JSON，包含 ``time``（秒）和 ``q``（弧度）。
        ``duration=None`` 时按 Ctrl-C 结束。该函数是阻塞的，返回采样点列表。
        """
        if sample_period <= 0:
            raise ValueError("sample_period 必须大于 0")
        if duration is not None and duration <= 0:
            raise ValueError("duration 必须大于 0 或为 None")
        kp_values = [0.0] * self.MOTOR_COUNT if kp is None else list(kp)
        kd_values = [0.05] * self.MOTOR_COUNT if kd is None else list(kd)
        if len(kp_values) != self.MOTOR_COUNT or len(kd_values) != self.MOTOR_COUNT:
            raise ValueError("kp 和 kd 必须包含四个关节值")

        self.stop_streaming()
        self.stop_motion()
        self.get_joint_positions()
        started = time.monotonic()
        samples = []
        try:
            while True:
                elapsed = time.monotonic() - started
                if duration is not None and elapsed > duration:
                    break
                # 先读取真实位置，再以该位置作为软位置目标，避免拖动时跳变。
                self.get_joint_positions()
                with self._state_lock:
                    q = [float(data.q) for data in self.feedback]
                from gravity import calc_compensated_torque
                tau1, tau2, tau3 = calc_compensated_torque(
                    self.feedback[1], self.feedback[2], self.feedback[3])
                torques = [0.0, tau1, tau2, tau3]
                for i, motor in enumerate(self.motors):
                    motor.mode = 1
                    motor.q = q[i]
                    motor.dq = 0.0
                    motor.tau = torques[i]
                    motor.kp = float(kp_values[i])
                    motor.kd = float(kd_values[i])
                    self.ser.sendRecv(motor, self.feedback[i])
                samples.append({"time": elapsed, "q": q})
                time.sleep(sample_period)
        except KeyboardInterrupt:
            if not stop_on_interrupt:
                raise
        finally:
            if stop_on_interrupt:
                self.disable()
        if not samples:
            raise RuntimeError("未采集到轨迹点")
        if path is not None:
            self.save_trajectory(path, samples)
        return samples

    @staticmethod
    def save_trajectory(path, samples):
        """保存示教轨迹 JSON；写入前校验每个点的四轴角度。"""
        points = []
        previous_time = 0.0
        for point in samples:
            if not isinstance(point, dict) or "q" not in point:
                raise ValueError("轨迹点必须是包含 q 的字典")
            q = [float(v) for v in point["q"]]
            if len(q) != ArmController.MOTOR_COUNT or not all(math.isfinite(v) for v in q):
                raise ValueError("轨迹点 q 必须包含四个有限角度")
            timestamp = float(point.get("time", len(points)))
            if not math.isfinite(timestamp) or timestamp < previous_time:
                raise ValueError("轨迹时间戳必须为非负且单调递增")
            points.append({"time": timestamp, "q": q})
            previous_time = timestamp
        if not points:
            raise ValueError("轨迹不能为空")
        payload = {"version": 1, "joint_count": ArmController.MOTOR_COUNT, "points": points}
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def replay_trajectory(self, path, speed=1.0, kp=None, kd=None, start_duration=2.0,
                          control_period=0.005):
        """先 MoveJ 到起点，再按统一时间轴连续发送插值位置与速度。

        使用保形三次插值，经过中间采样点时速度连续，整段起止速度为零。
        control_period 是目标更新周期；串口超时时跳过错过的更新时刻。
        本函数阻塞到回放结束，支持 stop_motion() 和 Ctrl-C 中断。
        """
        for name, value in (("speed", speed), ("start_duration", start_duration),
                            ("control_period", control_period)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须是大于 0 的有限数值")
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        points = payload.get("points") if isinstance(payload, dict) else payload
        if not isinstance(points, list) or not points:
            raise ValueError("轨迹文件缺少 points")
        times, positions = [], []
        # Validate the entire file before the first hardware command.
        for point in points:
            if not isinstance(point, dict) or "q" not in point or "time" not in point:
                raise ValueError("轨迹点必须包含 time 和 q")
            positions.append(self._validate_target(point["q"]))
            timestamp = float(point["time"])
            if (not math.isfinite(timestamp) or timestamp < 0
                    or (times and timestamp <= times[-1])):
                raise ValueError("轨迹时间戳必须为非负有限数值且严格递增")
            times.append(timestamp)
        scaled_times = [(timestamp - times[0]) / speed for timestamp in times]
        if (not all(math.isfinite(t) for t in scaled_times)
                or any(b <= a for a, b in zip(scaled_times, scaled_times[1:]))):
            raise ValueError("speed 缩放后的轨迹时间戳无效")
        trajectory = JointTrajectory(scaled_times, positions)

        self.moveJ(positions[0], duration=start_duration, kp=kp, kd=kd)
        self.wait_motion()
        if len(points) == 1 or self._stop_event.is_set():
            return

        from gravity import calc_compensated_torque

        started = time.monotonic()
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - started
            q, dq = trajectory.sample(elapsed)
            tau1, tau2, tau3 = calc_compensated_torque(
                self.feedback[1], self.feedback[2], self.feedback[3])
            torques = (0.0, tau1, tau2, tau3)
            for index, (motor, data) in enumerate(zip(self.motors, self.feedback)):
                motor.q, motor.dq, motor.tau = q[index], dq[index], torques[index]
                self.ser.sendRecv(motor, data)
            if elapsed >= trajectory.duration:
                break
            # Absolute deadlines absorb communication time; missed ticks are
            # skipped instead of queued or added to every recorded interval.
            now = time.monotonic()
            next_tick = math.floor((now - started) / control_period) + 1
            deadline = started + min(next_tick * control_period, trajectory.duration)
            self._stop_event.wait(max(0.0, deadline - now))

    def stop_motion(self):
        self._stop_event.set()
        if self._motion_thread and self._motion_thread.is_alive():
            self._motion_thread.join(timeout=1.0)
        self._motion_thread = None

    def close(self):
        self.stop_streaming()
        self.stop_motion()
        self.ser.close()








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
