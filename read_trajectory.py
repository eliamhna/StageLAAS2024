import time
import csv

from utils import MotorController
from vac_grip_control.vacuum_cup_librairy import gripper_strive, gripper_release


# connexion bus can (linux console) :
#      sudo ip link set can0 up type can bitrate 1000000

# Connect can bus for control motors
SO = [21, 18, 20, 8, 33.33]     # Use 'SO' if you use Orange SCARA (ID: 21 | 18 | 20 | 8) (33.33 for endless screw)
SR = [6, 13, 12, 2, 3.333]      # Use 'SR' if you use Red SCARA (ID: 6 | 13 | 12 | 2) (3.333 for endless screw)

params_robots = SR      # <- Use here 'SO' or 'SR'

motor1 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[0]})
motor2 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[1]})
motor3 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[2]})
motor4 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[3]})

motors = [motor1, motor2, motor4, motor3]
max_vel = [1500, 1500, 1500, 1500]          # motors velocities

hertz = 100  # same as simulation's file

q1, q2, q3, q4, gripper = [], [], [], [], []
with open('SCARA_Final/trajectory.csv', newline='') as csvfile:     # read csv file
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        q1.append(float(row[0]))    # first leg motor
        q2.append(float(row[1]))    # second leg motor
        q4.append(float(row[2]))    # gripper rotation
        q3.append(float(row[3]))    # gripper height
        gripper.append(int(row[4]))   # gripper opened/closed

"""  
for m in motors:
    m.readMotorStatus()
offsets = [motor1.position, motor2.position, motor4.position, motor3.position]  # offset motors : q1, q2, q4, q3
"""

# use 'pos_0torque.py' for offsets
offsets = [0, 0, 0, 0]  # offset motors : q1, q2, q4, q3

gripper_strive()
for q in range(len(q1)):
    t_act = time.time()
    
    motor1.sendPosition(q1[q]+offsets[0], max_vel[0])       # move motors
    motor2.sendPosition(q2[q]+offsets[1], max_vel[1])
    motor4.sendPosition(q4[q]+offsets[2], max_vel[2])
    motor3.sendPosition((q3[q]*params_robots[4])+offsets[3], max_vel[3])

    if (q1[q]<-60 or q1[q]>60) or (q2[q]<-60 or q2[q]>60) or (q3[q]<-150 or q3[q]>10):
        print('limits reach')
        motor1.sendTorque(0)
        motor2.sendTorque(0)
        motor4.sendTorque(0)
        motor3.sendTorque(0)
        break
    
    if gripper[q] == 1:       # gripper 0 open, 1 close
        gripper_release()
    elif gripper[q] == 0:
        gripper_strive()
        
    while time.time()-t_act < (1/hertz):      # wait time for hertz
        time.sleep(0.001)