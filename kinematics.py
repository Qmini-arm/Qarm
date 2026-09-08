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
    q1 = math.pi-(math.atan2(r, z) + math.atan2(k2, k1))

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
    q0 = math.atan2(y, x)  # joint_1 角度
    q1, q2 = inverse_kinematics(r, z)
    if q1 is None or q2 is None:
        return None  # 目标位置不可达
    q3 = calc_q3_from_q1_q2(q1, q2, wrist_offset)
    return [q0, q1, q2, q3]  # joint_4 初始为0，后续可根据需要调整
if __name__ == "__main__":
    # 测试逆运动学和正运动学
    test_r, test_z = 0.4, 0.0
    q1, q2 = inverse_kinematics(test_r, test_z)
    if q1 is not None and q2 is not None:
        print(f"Inverse Kinematics: q1={math.degrees(q1):.2f}°, q2={math.degrees(q2):.2f}°")
        r, z = forward_kinematics(q1, q2)
        print(f"Forward Kinematics: r={r:.3f}, z={z:.3f}")
    else:
        print("Target position is unreachable.")