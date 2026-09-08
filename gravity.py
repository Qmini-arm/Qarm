import math
import time
from motor_driver import SerialPort, MotorCmd, MotorData, move
from config.config import MOTOR_OFFSETS

def verify_motor_init(ser, mt, dt, motor_name):
    success_count = 0
    # 循环读取，直到连续获得 10 次稳定反馈
    while success_count < 10:
        ser.sendRecv(mt, dt)
        
        # 验证条件：根据你的驱动库，如果通信失败 dt.q 可能是 None，或者 id 对不上
        # 这里做一个基础的防空值判断（如果你的库失败时返回0.0，你需要根据实际情况调整）
        if dt is not None: 
            success_count += 1
        else:
            success_count = 0 # 一旦断掉，重新计数
            print(f"[{motor_name}] 读取失败，重试中...")
            
        time.sleep(0.005) # 留出 5ms 给 Linux 底层 USB 驱动喘息
def calc_compensated_torque(dt1, dt2, dt3, pi_coeffs):
    """
    计算 joint_2、joint_3、joint_4 的重力补偿力矩。

    URDF 中三个平行轴在 base_link 坐标系的方向关系是
    joint_2 : joint_3 : joint_4 = + : - : +。因此连杆绝对角度为
    q1、q1-q2、q1-q2+q3；同一个下游物理力矩映射到相邻上游关节时
    需要反号。

    dt1, dt2, dt3: 分别对应 joint_2、joint_3、joint_4 的反馈角度
    pi_coeffs: (PI_1, PI_2, PI_3) 动力学系数
    """
    PI_1, PI_2, PI_3 = pi_coeffs

    q1, q2, q3 = dt1.q, dt2.q, dt3.q
    angle_link_2 = q1
    angle_link_3 = q1 - q2
    angle_link_6 = q1 - q2 + q3

    # 假定 q=0 是水平参考、正角度抬升连杆时，下面的全局符号为正。
    # joint_3 的轴与 joint_2 反向，所以 tau2 及其传递到 tau1 的项取反。
    tau3 = PI_3 * math.sin(angle_link_6)
    tau2 = PI_2 * math.sin(angle_link_3) + tau3
    tau1 = -(PI_1 * math.sin(angle_link_2) + tau2)

    return tau1, tau2, tau3

def torque_soft_start(ser_list, mt_list, dt_list, pi_coeffs, duration=1.0, steps=100):
    """
    平滑加载重力补偿力矩，防止电机在启动瞬间发生力矩阶跃和震动。
    """
    ser8, ser9, ser10, ser7 = ser_list
    mt0, mt1, mt2, mt3 = mt_list
    dt0, dt1, dt2, dt3 = dt_list
    
    # 1. 继承当前物理位置，记录下初始状态
    # 注意：这些初始角度在整个缓启动期间都不会改变，确保电机“原地绷紧”
    initial_q0 = dt0.q
    initial_q1 = dt1.q
    initial_q2 = dt2.q
    initial_q3 = dt3.q

    mt0.q = initial_q0
    mt1.q = initial_q1
    mt2.q = initial_q2
    mt3.q = initial_q3
    
    # 设置 3号电机的 PD 参数（提前设好，避免后续突变）
    mt3.kp = 0.5
    mt3.kd = 0.02
    
    # 2. 开启电机模式
    mt0.mode = 1
    mt1.mode = 1
    mt2.mode = 1
    mt3.mode = 1
    
    # 纯位置环首次上电锁死
    ser8.sendRecv(mt0, dt0)
    ser9.sendRecv(mt1, dt1)
    ser10.sendRecv(mt2, dt2)
    ser7.sendRecv(mt3, dt3)
    
    print(f"开始力矩缓启动，预计耗时 {duration} 秒...")
    step_delay = duration / steps
    
    # 3. 缓启动插值主循环
    for i in range(1, steps + 1):
        ratio = i / steps  # 从 0.01 逐渐增加到 1.0
        tau1, tau2, tau3 = calc_compensated_torque(dt1, dt2, dt3, pi_coeffs)
        # 计算满负荷受力，并乘以当前步的缓启动比例 ratio
        # 注意：这里我们用实时的反馈 dt.q 来计算重力，更精确
        mt3.tau = ratio * tau3
        mt2.tau = ratio * tau2
        mt1.tau = ratio * tau1
        
        # 【核心修复】：绝对不能在这里改变 mt3.q 的值！
        # 让所有电机保持在 initial_q 的位置
        mt0.q = initial_q0
        mt1.q = initial_q1
        mt2.q = initial_q2
        mt3.q = initial_q3
        
        # 发送插值指令并获取最新反馈
        ser8.sendRecv(mt0, dt0)
        ser9.sendRecv(mt1, dt1)
        ser10.sendRecv(mt2, dt2)
        ser7.sendRecv(mt3, dt3)
        
        time.sleep(step_delay)
        
    print("重力补偿 100% 加载完毕，进入正常运行状态！")

# ================= 主控制流程 =================

LINK1_LENGTH = 0.30 # 肩部电机轴心 到 肘部电机轴心 的距离
LINK2_LENGTH = 0.30 # 肘部电机轴心 到 腕部电机轴心 的距离

def get_motor3_horizon_position(dt1, dt2):
    """
    计算腕部电机（3号）在水平位置时的目标角度。
    假设腕部电机的零位是手臂水平向前。
    """
    # 目标位置：手臂水平向前，意味着腕部电机需要补偿肘部的角度
    target_q3 = dt2.q - dt1.q + math.pi / 2  # +90度，使手臂水平
    return target_q3

if __name__ == "__main__":
    
    # 实例化电机
    ser = SerialPort("/dev/ttyUSB0")
    mt0 = MotorCmd(id=0, direction=1, offset=MOTOR_OFFSETS[0])
    mt1 = MotorCmd(id=1, direction=1, offset=MOTOR_OFFSETS[1])
    mt2 = MotorCmd(id=2, direction=1, offset=MOTOR_OFFSETS[2])
    mt3 = MotorCmd(id=3, direction=1, offset=MOTOR_OFFSETS[3])
    dt0 = MotorData()
    dt1 = MotorData()
    dt2 = MotorData()
    dt3 = MotorData()

    # 重力补偿系数（关节输出侧力矩，单位：N·m）。
    #
    # 这些数值由 qmini_arm.urdf.xacro 中 link_2/link_3/link_6 的
    # mass 和质心位置估算：PI = mass * 9.80665 * sqrt(com_y² + com_z²)。
    # 当前补偿模型假定各段在同一平面内，并把这些值作为 cos() 项的幅值；
    # URDF 的 CAD 零位还可能带有相位偏移，首次上电应从较小比例开始验证。
    PI_1 = 4.4  # link_2: 0.676212997 kg, r_com=0.258230863 m
    PI_2 = 1.712549  # link_3: 0.676213000 kg, r_com=0.258249095 m
    PI_3 = 0.00  # link_6: 0.016653600 kg, r_com=0.017495892 m
    
    #初始化角度
    verify_motor_init(ser, mt0, dt0, "肩部(mt0)")
    verify_motor_init(ser, mt1, dt1, "肘部1(mt1)")
    verify_motor_init(ser, mt2, dt2, "肘部2(mt2)")
    verify_motor_init(ser, mt3, dt3, "手腕(mt3)")
    torque_soft_start(
        ser_list=[ser, ser, ser, ser], 
        mt_list=[mt0, mt1, mt2, mt3], 
        dt_list=[dt0, dt1, dt2, dt3], 
        pi_coeffs=(PI_1, PI_2, PI_3),
        duration=0.3,  # 你可以自由修改这里的启动时间，比如 1.5 秒
        steps=50      # 步数跟着等比调整
    )

    move(ser, mt3, dt3, target= get_motor3_horizon_position(dt1, dt2), duration=0.5)
    time.sleep(0.5)



    try:
        while True:
            # 1. 重力补偿 (这里最好用真实的反馈位置 q1, q2, q3 来计算，因为这是当下的物理受力)
            tau1, tau2, tau3 = calc_compensated_torque(dt1, dt2, dt3, (PI_1, PI_2, PI_3))
            # print(tau1,tau2,tau3,end=' ')
            mt3.tau = tau3
            mt2.tau = tau2
            mt1.tau = tau1
            mt1.kd=0.05
            mt2.kd=0.025
            mt0.tau = 0
            mt3.q = get_motor3_horizon_position(dt1, dt2)  # 始终保持手腕水平
            mt3.kp = 0.5
            mt3.kd = 0.02

            

            ser.sendRecv(mt0, dt0)
            ser.sendRecv(mt1, dt1)
            ser.sendRecv(mt2, dt2)
            ser.sendRecv(mt3, dt3)

            print(f"ID:0 | P:{dt0.q:>+7.2f} | V:{dt0.dq:>+7.2f}  "
              f"ID:1 | P:{dt1.q:>+7.2f} | V:{dt1.dq:>+7.2f}  "
              f"ID:2 | P:{dt2.q:>+7.2f} | V:{dt2.dq:>+7.2f}  "
              f"ID:3 | P:{dt3.q:>+7.2f} | V:{dt3.dq:>+7.2f}   ", end='\r')
            
            time.sleep(0.01)



    except KeyboardInterrupt:
        mt0.mode = 0
        mt1.mode = 0
        mt2.mode = 0
        mt3.mode = 0
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)

        print("\n程序停止")
