import time
from motor_driver import SerialPort, MotorCmd, MotorData


from config.config import MOTOR_OFFSETS


ser = SerialPort("/dev/ttyUSB0")  

mt0 = MotorCmd(id=0, direction=1, offset=MOTOR_OFFSETS[0])
mt1 = MotorCmd(id=1, direction=1, offset=MOTOR_OFFSETS[1])
mt2 = MotorCmd(id=2, direction=1, offset=MOTOR_OFFSETS[2])
mt3 = MotorCmd(id=3, direction=1, offset=MOTOR_OFFSETS[3])
dt0 = MotorData()
dt1 = MotorData()
dt2 = MotorData()
dt3 = MotorData()

try:
    while True:
    
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
        mt0.kp = 0.0        
        mt0.kd = 0.0       
        mt0.dq = 0.0       
        mt0.tau = 0.0
        ser.sendRecv(mt0, dt0)
        print("\n程序停止")

