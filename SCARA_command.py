from utils import MotorController
import matplotlib.pyplot as plt
import numpy as np
import time

# connexion bus can (linux console) :
#      sudo ip link set can0 up type can bitrate 1000000

hz = 100
ts = 1/hz
positions = [[],[],[],[]]

def moveJ(qi, qf, tf, Te):          # return all pos from 'qi' to 'qf' in 'tf' sec with 'Te' points spacing
    Y = [qi, 0, qf, 0]
    A = np.array([[1, 0, 0*0, 0*0*0], [0, 1, 2*0, 3*0*0], [1, tf, tf*tf, tf*tf*tf], [0, 1, 2*tf, 3*tf*tf]])
    invA = np.linalg.inv(A)
    coeff = np.matmul(invA, Y)
    t = np.linspace(0, tf, round(tf/Te))
    q = coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    return q

def move4axes(qd, tf):
    for m in ms:
        m.readMotorStatus()
    offsets = [motor1.position, motor2.position, motor3.position/params_robots[4], motor4.position]
    q = [moveJ(offsets[0], zeros4pos[0]+qd[0], tf, ts),
         moveJ(offsets[1], zeros4pos[1]+qd[1], tf, ts),
         moveJ(offsets[2], zeros4pos[2]+qd[2], tf, ts),
         moveJ(offsets[3], zeros4pos[3]+qd[3], tf, ts)]
    for i in range(len(q[0])):
        t_st = time.time()
        motor1.sendPosition(q[0][i], max_vel[0])
        motor2.sendPosition(q[1][i], max_vel[1])
        motor3.sendPosition((q[2][i])*params_robots[4], max_vel[2])
        motor4.sendPosition(q[3][i], max_vel[3])
        
        positions[0].append((motor1.position)-zeros4pos[0])
        positions[1].append((motor2.position)-zeros4pos[1])
        positions[2].append((motor3.position/params_robots[4])-zeros4pos[2])
        positions[3].append((motor4.position)-zeros4pos[3])
        while time.time()-t_st < ts:
            pass

def pickandplace(q1, q2, q3, q4):
    move4axes([q1, q2, 0, q4], 1)
    move4axes([q1, q2, q3, q4], 2)
    move4axes([q1, q2, 0, q4], 2)
    move4axes([0, 0, 0, 0], 1)

# Connect can bus for control motors
SO = [21, 18, 20, 8, 33.33]     # Use 'SO' if you use Orange SCARA (ID: 21 | 18 | 20 | 8) (33.33 for endless screw)
SR = [6, 13, 12, 2, 3.333]      # Use 'SR' if you use Red SCARA (ID: 6 | 13 | 12 | 2) (3.333 for endless screw)

params_robots = SR      # <- Use here 'SO' or 'SR'

motor1 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[0]})
motor2 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[1]})
motor3 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[2]})
motor4 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[3]})

ms = [motor1, motor2, motor3, motor4]
max_vel = [1500, 1500, 1500, 1500]

ofsts = input('Auto offsets : 1\nHand offsets : 2\n: ')

zeros4pos = [0, 0, 0, 0]
if ofsts == '1':
    for m in ms:
        m.readMotorStatus()
    zeros4pos = [motor1.position, motor2.position, motor3.position/params_robots[4], motor4.position]
if ofsts == '2':
    q1 = int(input('q1[°] = '))
    q2 = int(input('q2[°] = '))
    q3 = int(input('q3[mm] = '))
    q4 = int(input('q4[°] = '))
    zeros4pos = [q1, q2, q3, q4]

# change values
pickandplace(58.91, -68.88, -151.17611761176119, 120.06)
pickandplace(-42.14, 74.89000000000001, -150.95109510951096, -110.65)

time.sleep(0.5)
for m in ms:
    m.sendTorque(0)

# plot motors' trajectory
plt.figure()
plt.plot(positions[0], label='motor1')
plt.plot(positions[1], label='motor2')
plt.plot(positions[2], label='motor3')
plt.plot(positions[3], label='motor4')
plt.legend()
plt.show()