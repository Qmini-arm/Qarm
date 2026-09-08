import math
import time
from motor_driver import SerialPort, MotorCmd, MotorData, move
from config.config import MOTOR_OFFSETS
from gravity import calc_compensated_torque, torque_soft_start, verify_motor_init

LINK1_LENGTH = 0.3  # 第一段连杆长度，单位米
LINK2_LENGTH = 0.3  # 第二段连杆长度，单位米
def inverse_kinematics(r, z):
    """
    逆运动学：已知目标腕部位置(r, z)，求 joint_2、joint_3 角度。

    joint_2 和 joint_3 的轴线平行但方向相反，所以第二段连杆的
    物理相对角是 ``-q2``。
    """
    # 限制到原点的距离，防止数学溢出
    d_square = r**2 + z**2
    if d_square > (LINK1_LENGTH + LINK2_LENGTH)**2:
        return None, None # 够不到，超出最大臂展
    if d_square < (LINK1_LENGTH - LINK2_LENGTH)**2:
        return None, None # 靠太近，结构干涉

    # 余弦定理求 q2
    cos_q2 = (d_square - LINK1_LENGTH**2 - LINK2_LENGTH**2) / (2 * LINK1_LENGTH * LINK2_LENGTH)
    # 浮点数防越界
    cos_q2 = max(-1.0, min(1.0, cos_q2)) 
    
    # 采用“肘部朝上”分支。这里 relative_angle 是两段连杆的物理夹角，
    # q2 是 URDF joint_3 角度，二者因轴方向相反而符号相反。
    relative_angle = math.acos(cos_q2)
    q2 = -relative_angle  # joint_3 的角度

    # 求 q1
    k1 = LINK1_LENGTH + LINK2_LENGTH * math.cos(relative_angle)
    k2 = LINK2_LENGTH * math.sin(relative_angle)
    q1 = math.pi/2-(math.atan2(z, r) + math.atan2(k2, k1))

    return q1, q2
    
def forward_kinematics(q1, q2):
    """
    正运动学：已知角度，求腕部位置
    假设原点在肩部电机轴心，正前方为r轴正方向，正上方为z轴正方向。
    角度0度时手臂水平向前。
    """
    r = LINK1_LENGTH * math.sin(q1) + LINK2_LENGTH * math.sin(q1 - q2)
    z = LINK1_LENGTH * math.cos(q1) + LINK2_LENGTH * math.cos(q1 - q2)
    return r, z

def calc_q3_from_q1_q2(q1, q2,offset=0.0):
    """
    计算 joint_4 的角度，使得手腕保持水平。
    joint_4 的轴线与 joint_3 相同方向，所以 q3 = q2 - q1 + 90° + offset
    其中 90° 是为了让手腕水平,offset 是 相对地面角度，单位弧度，正方向朝向地面。
    """
    return q2 - q1 + math.pi / 2 + offset
def from_world_to_joint_angles(x,y,z,wrist_offset=0.0):
    """
    将世界坐标系下的腕部位置 (r, z) 转换为四个关节角度。
    """
    r = math.sqrt(x**2 + y**2)
    q0 = -math.atan2(x, y)  # joint_1 角度
    q1, q2 = inverse_kinematics(r, z)
    if q1 is None or q2 is None:
        return None  # 目标位置不可达
    q3 = calc_q3_from_q1_q2(q1, q2, wrist_offset)
    return [q0, q1, q2, q3]  # joint_4 初始为0，后续可根据需要调整
if __name__ == "__main__":
    # 测试逆运动学和正运动学
    test_x,test_y, test_z = 0.2, 0.2, 0.3
    q0, q1, q2, q3 = from_world_to_joint_angles(test_x, test_y, test_z)
    print(f"Inverse Kinematics: q0={q0:.2f}, q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}")
    print(f"Forward Kinematics: r={forward_kinematics(q1, q2)[0]:.2f}, z={forward_kinematics(q1, q2)[1]:.2f}")
    ser = SerialPort("/dev/ttyUSB0")
    mt0 = MotorCmd(id=0, direction=1, offset=MOTOR_OFFSETS[0])
    mt1 = MotorCmd(id=1, direction=1, offset=MOTOR_OFFSETS[1])
    mt2 = MotorCmd(id=2, direction=1, offset=MOTOR_OFFSETS[2])
    mt3 = MotorCmd(id=3, direction=1, offset=MOTOR_OFFSETS[3])
    dt0 = MotorData()
    dt1 = MotorData()
    dt2 = MotorData()
    dt3 = MotorData()

    
    #初始化角度
    verify_motor_init(ser, mt0, dt0, "肩部(mt0)")
    verify_motor_init(ser, mt1, dt1, "肘部1(mt1)")
    verify_motor_init(ser, mt2, dt2, "肘部2(mt2)")
    verify_motor_init(ser, mt3, dt3, "手腕(mt3)")
    torque_soft_start(
        ser_list=[ser, ser, ser, ser], 
        mt_list=[mt0, mt1, mt2, mt3], 
        dt_list=[dt0, dt1, dt2, dt3], 
        duration=0.3,  # 你可以自由修改这里的启动时间，比如 1.5 秒
        steps=50      # 步数跟着等比调整
    )
    time.sleep(0.5)
    try:
            while True:
                # 1. 重力补偿 (这里最好用真实的反馈位置 q1, q2, q3 来计算，因为这是当下的物理受力)
                tau1, tau2, tau3 = calc_compensated_torque(dt1, dt2, dt3)
                # print(tau1,tau2,tau3,end=' ')
                mt3.tau = tau3
                mt2.tau = tau2
                mt1.tau = tau1
                mt0.tau = 0
                move(ser, mt0, dt0, target=q0, duration=5,tau=0)
                move(ser, mt1, dt1, target=q1, duration=5,tau=tau1)
                move(ser, mt2, dt2, target=q2, duration=5,tau=tau2)
                move(ser, mt3, dt3, target=q3, duration=5,tau=tau3)
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
    