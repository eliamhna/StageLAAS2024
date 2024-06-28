from utils import MotorController, RobotController
from vac_grip_control.vacuum_cup_librairy import gripper_strive, gripper_release
import numpy as np
import time
import csv

# connexion bus can (linux console) :
#       sudo ip link set can0 up type can bitrate 1000000

# variable
hz = 100
ts = 1/hz

nb_tours = 1

#########################
####### FUNCTIONS #######
#########################

def moveJ(qi, qf, tf, Te):          # return all pos from 'qi' to 'qf' in 'tf' sec with 'Te' points spacing
    Y = [qi, 0, qf, 0]
    A = np.array([[1, 0, 0*0, 0*0*0], [0, 1, 2*0, 3*0*0], [1, tf, tf*tf, tf*tf*tf], [0, 1, 2*tf, 3*tf*tf]])
    invA = np.linalg.inv(A)
    coeff = np.matmul(invA, Y)
    t = np.linspace(0, tf, round(tf/Te))
    q = coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    return q

def writeCSV():                     # write all final positions in file
    with open('trajectory.csv', 'w', newline='') as fichier_csv:
        writer = csv.writer(fichier_csv)
        for pos in all_pos:
                writer.writerow(pos)

def move4axes(qd, tf):              # move SCARA
    for m in ms:
        m.readMotorStatus()
    offsets = [motor1.position, motor2.position, motor3.position, motor4.position]
    q = [moveJ(offsets[0], qd[0], tf, ts),
         moveJ(offsets[1], qd[1], tf, ts),
         moveJ(offsets[2], qd[2], tf, ts),
         moveJ(offsets[3], qd[3], tf, ts)]
    for i in range(len(q[0])):
        t_st = time.time()
        motor1.sendPosition(q[0][i], max_vel[0])
        motor2.sendPosition(q[1][i], max_vel[1])
        motor3.sendPosition(q[2][i], max_vel[2])
        motor4.sendPosition(q[3][i], max_vel[3])
        while time.time()-t_st < ts:
            pass

def move2x3axes(qd1, qd2, tf):      # move MATEs
    robot1.readMotorStatus()
    robot2.readMotorStatus()
    offsets1 = [robot1.positions[0], robot1.positions[1], robot1.positions[2]]
    offsets2 = [robot2.positions[0], robot2.positions[1], robot2.positions[2]]
    q1 = [moveJ(offsets1[0], qd1[0], tf, ts), moveJ(offsets1[1], qd1[1], tf, ts), moveJ(offsets1[2], qd1[2], tf, ts)]
    q2 = [moveJ(offsets2[0], qd2[0], tf, ts), moveJ(offsets2[1], qd2[1], tf, ts), moveJ(offsets2[2], qd2[2], tf, ts)]
    for i in range(len(q1[0])):
        t_st = time.time()
        while time.time()-t_st < ts:
            robot1.send3Position(q1[0][i], q1[1][i], q1[2][i])
            robot2.send3Position(q2[0][i], q2[1][i], q2[2][i])
            
def setAllTorque0():                # set all robots to 0 torque for hand moving
    for m in ms:
        m.sendTorque(0)
    for r in rs:
        r.send3Torque(0, 0, 0)
        
def seePositions():                 # read and print all positions
    setAllTorque0()
    while True:
        for m in ms:
            m.readMotorStatus()
        for r in rs:
            r.readMotorStatus()
        
        print("\n\n")
        print(f'SCARA : || {"%.2f" % motor1.position} || {"%.2f" % motor2.position} || {"%.2f" % motor3.position} || {"%.2f" % motor4.position} ||')
        print(f'MATE 1 : || {"%.2f" % robot1.positions[0]} || {"%.2f" % robot1.positions[1]} || {"%.2f" % robot1.positions[2]} ||')
        print(f'MATE 2 : || {"%.2f" % robot2.positions[0]} || {"%.2f" % robot2.positions[1]} || {"%.2f" % robot2.positions[2]} ||')
        print("\n\n")
        
def movement(iter, tf, witch):      # move SCARA and MATEs
    if witch == 1:
        move4axes([pos_read[iter][0], pos_read[iter][1], pos_read[iter][2], pos_read[iter][3]], tf)
    if witch == 2:
        move2x3axes([pos_read[iter][4], pos_read[iter][5], pos_read[iter][6]], [pos_read[iter][7], pos_read[iter][8], pos_read[iter][9]], tf)
        
def setNewPosition():               # Learn new Positions
    setAllTorque0()
    wait = input('press "Enter" when pos OK ')

    for m in ms:
        m.readMotorStatus()
    for r in rs:
        r.readMotorStatus()
    all_pos.append([st_pos[0], st_pos[1], st_pos[2], st_pos[3], 
                    robot1.positions[0], robot1.positions[1], robot1.positions[2], 
                    robot2.positions[0], robot2.positions[1], robot2.positions[2]]) # MATE POS
    all_pos.append([motor1.position, motor2.position, st_pos[2], motor4.position, 
                    robot1.positions[0], robot1.positions[1], robot1.positions[2], 
                    robot2.positions[0], robot2.positions[1], robot2.positions[2]]) # SCARA APP
    all_pos.append([motor1.position, motor2.position, motor3.position, motor4.position, 
                    robot1.positions[0], robot1.positions[1], robot1.positions[2], 
                    robot2.positions[0], robot2.positions[1], robot2.positions[2]]) # SCARA POS
    all_pos.append([motor1.position, motor2.position, st_pos[2], motor4.position, 
                    robot1.positions[0], robot1.positions[1], robot1.positions[2], 
                    robot2.positions[0], robot2.positions[1], robot2.positions[2]]) # SCARA APP
    all_pos.append([st_pos[0], st_pos[1], st_pos[2], st_pos[3], 
                    robot1.positions[0], robot1.positions[1], robot1.positions[2], 
                    robot2.positions[0], robot2.positions[1], robot2.positions[2]]) # SCARA 0
    all_pos.append(st_pos)  # MATE 0

    move4axes([motor1.position, motor2.position, st_pos[2], motor4.position], 1)
    time.sleep(0.5)
    move4axes([st_pos[0], st_pos[1], st_pos[2], st_pos[3]], 1)
    move2x3axes([st_pos[4], st_pos[5], st_pos[6]], [st_pos[7], st_pos[8], st_pos[9]], 1)

def goPosition(iter, gripper):      # go to 6 positions 'pick and place'
    movement(iter*6+0, 0.5, 2)
    movement(iter*6+1, 1, 1)
    movement(iter*6+2, 0.25, 1)
    time.sleep(0.5)
    if gripper == 0:
        gripper_release()
    if gripper == 1:
        gripper_strive()
    time.sleep(1)
    movement(iter*6+3, 0.25, 1)
    time.sleep(0.5)
    movement(iter*6+4, 1, 1)
    movement(iter*6+5, 0.5, 2)
          
##############################
####### INITIALISATION #######
##############################
  
motor1 = MotorController({"channel":'can0',"bus_index":0,"motor_id":4})     # SCARA
motor2 = MotorController({"channel":'can1',"bus_index":0,"motor_id":13})    # |
motor3 = MotorController({"channel":'can2',"bus_index":0,"motor_id":12})    # |
motor4 = MotorController({"channel":'can3',"bus_index":0,"motor_id":2})     # |

robot1 = RobotController({"channel":'can4',"bus_index":0,"IDs":[14, 5, 6],"max_vels":[750, 750, 750]})      # MATE 1
robot2 = RobotController({"channel":'can5',"bus_index":0,"IDs":[17, 23, 10],"max_vels":[750, 750, 750]})    # MATE 2

ms = [motor1, motor2, motor3]#, motor4]
max_vel = [750, 750, 1500]#, 750]
rs = [robot1, robot2]

all_pos = []    # Required for save movements
for m in ms:
    m.readMotorStatus()
for r in rs:
    r.readMotorStatus()
st_pos = [motor1.position, motor2.position, motor3.position, 0,#motor4.position, 
          robot1.positions[0], robot1.positions[1], robot1.positions[2], 
          robot2.positions[0], robot2.positions[1], robot2.positions[2]]    # Positions at 0 for new positions

pos_read = []       # Read trajectory file for movement
with open('trajectory.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        row_nb = []
        for act_pos in row:
            row_nb.append(float(act_pos))
        pos_read.append(row_nb)

##############################
############ MAIN ############
##############################

choice = '2'
"""input('1 : Save New Positions\n'
               '2 : Movement\n'
               '? --> ')                        # choice for learn new positions & movement 'pick and place'
"""
if choice == '1':   # learn new positions
    nb_pos = int(input('\n\nHow many new pos ?\n? --> '))
    for nnp in range(nb_pos):
        setNewPosition()
    writeCSV()
    
if choice == '2':   # movement 'pick and place'
    nb_pos = int(len(pos_read)/6)
    gripper = [0, 1]
    print('\n\nThe gripper is ...\n1 : open\n0 : close')    # Gripper for each movement
    grp_st = int(input('Starting pos --> '))
    for nb_iter in range(nb_pos):
        gripper.append(int(input(f'Movement {nb_iter+1} --> ')))
    input('\npress "Enter" for start ')
    if grp_st == 0:
        gripper_release()
    if grp_st == 1:
        gripper_strive()
        time.sleep(1)
    for i in range(nb_tours):
        for nb_iter in range(nb_pos):
            goPosition(nb_iter, gripper[nb_iter])

time.sleep(0.2)
setAllTorque0()     # end program with 0 torque
