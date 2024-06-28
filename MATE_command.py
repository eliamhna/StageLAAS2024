from utils import RobotController
import numpy as np
import time

# connexion bus can (linux console) :
#      sudo ip link set can0 up type can bitrate 1000000

hz = 100
ts = 1/hz

def moveJ(qi, qf, tf, Te):          # return all pos from 'qi' to 'qf' in 'tf' sec with 'Te' points spacing
    Y = [qi, 0, qf, 0]
    A = np.array([[1, 0, 0*0, 0*0*0], [0, 1, 2*0, 3*0*0], [1, tf, tf*tf, tf*tf*tf], [0, 1, 2*tf, 3*tf*tf]])
    invA = np.linalg.inv(A)
    coeff = np.matmul(invA, Y)
    t = np.linspace(0, tf, round(tf/Te))
    q = coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    return q

def move2x3axes(qd1, qd2, tf):
    robot1.readMotorStatus()
    robot2.readMotorStatus()
    offsets1 = [robot1.positions[0], robot1.positions[1], robot1.positions[2]]
    offsets2 = [robot2.positions[0], robot2.positions[1], robot2.positions[2]]
    q1 = [moveJ(offsets1[0], zeros1[0]+qd1[0], tf, ts), moveJ(offsets1[1], zeros1[1]+qd1[1], tf, ts), moveJ(offsets1[2], zeros1[2]+(qd1[2]*1.25), tf, ts)]
    q2 = [moveJ(offsets2[0], zeros2[0]+qd2[0], tf, ts), moveJ(offsets2[1], zeros2[1]+qd2[1], tf, ts), moveJ(offsets2[2], zeros2[2]+(qd2[2]*1.25), tf, ts)]
    for i in range(len(q1[0])):
        t_st = time.time()
        while time.time()-t_st < ts:
            robot1.send3Position(q1[0][i], q1[1][i], q1[2][i])
            robot2.send3Position(q2[0][i], q2[1][i], q2[2][i])

# don't forget to change can channels
robot1 = RobotController({"channel":'can0',"bus_index":0,"IDs":[14, 5, 11],"max_vels":[1500, 1500, 1500]})
robot2 = RobotController({"channel":'can1',"bus_index":0,"IDs":[17, 23, 10],"max_vels":[1500, 1500, 1500]})

# Setup
robot1.send3Torque(0, 0, 0)
robot2.send3Torque(0, 0, 0)
robot1.readMotorStatus()
robot2.readMotorStatus()
zeros1 = [robot1.positions[0], robot1.positions[1], robot1.positions[2]]
zeros2 = [robot2.positions[0], robot2.positions[1], robot2.positions[2]]

##########################
########## MAIN ##########
##########################

# move2x3axes ([list of angle robot 1], [list of angle robot 2], time final)

move2x3axes([0, 0, 0], [0, 0, 0], 1)

move2x3axes([45, 45, 45], [45, 45, 45], 1)
input("wait")

move2x3axes([0, 0, 0], [0, 0, 0], 1)

#####
## End
robot1.send3Torque(0, 0, 0)
robot2.send3Torque(0, 0, 0)
