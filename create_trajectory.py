import matplotlib.pyplot as plt
import numpy as np
import time
import csv

from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()

sim = client.getObject('sim')

#required for IK calculation
sim_ik = client.getObject('simIK')
ik_env=sim_ik.createEnvironment()

client.setStepping(True)
sim.startSimulation()

motor1=sim.getObject("/q1")
motor2=sim.getObject("/q2")
motor3=sim.getObject("/q3")
motor4=sim.getObject("/q4")


hz = 100      # hertz

time_all_prog = time.time()
all_positions = [[], [], [], [], []]
all_time = []

def moveJ(qi, qf, tf, Te):          # return all pos from 'qi' to 'qf' in 'tf' sec with 'Te' points spacing
    Y = [qi, 0, qf, 0]
    A = np.array([[1, 0, 0*0, 0*0*0], [0, 1, 2*0, 3*0*0], [1, tf, tf*tf, tf*tf*tf], [0, 1, 2*tf, 3*tf*tf]])
    invA = np.linalg.inv(A)
    coeff = np.matmul(invA, Y)
    t = np.linspace(0, tf, round(tf/Te))
    q = coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    return q


def moveMotor(qf, tf, gripper):      # move every motors (q1, q2, q4) [°] - (q3) [mm] - (gripper) [1/0]
       ts = 1/hz

       all_q = [0, 0, 0, 0]
       motors = [motor1, motor2, motor4, motor3]
       
       for m in range(3):   # create moveJ trajectory for every motors
              all_q[m] = moveJ(sim.getJointPosition(motors[m]), np.deg2rad(qf[m]), tf, ts)        #q1,q2,q4
       all_q[-1] = moveJ(sim.getJointPosition(motors[-1]), 0.001*qf[-1], tf, ts)                  #q3

       for q_act in range(len(all_q[0])): # movement
              t_act = time.time()         # time for hertz

              for m in range(3):          # set position + write in list 'all_positions' for csv
                     sim.setJointPosition(motors[m], all_q[m][q_act])                             #q1,q2,q4
                     all_positions[m].append(np.rad2deg(sim.getJointPosition(motors[m])))
              sim.setJointPosition(motors[-1], all_q[-1][q_act])                                  #q3
              all_positions[-2].append(sim.getJointPosition(motors[-1])*1000)
              all_positions[-1].append(gripper)                                                     #gripper

              all_time.append(time.time()-time_all_prog)
              client.step() # step
              
              while time.time()-t_act < ts:      # wait time for hertz
                     time.sleep(0.001)
                     
##########################
########## MAIN ##########
##########################

# change this part for another movement
# moveMotor use this params ->  || list of angles values [q1[°], q2[°], q4[°], q3[mm]] 
#                               || time uses for this action 
#                               || gripper [0 open / 1 close]

moveMotor([45, -45, 90, 0], 2, 0)   # Move to A
moveMotor([45, -45, 90, -120], 7, 0)      # Descente A
moveMotor([45, -45, 90, 0], 7, 1)   # Montee A

moveMotor([-45, 45, -90, 0], 4, 1)  # Move to B
moveMotor([-45, 45, -90, -120], 7, 1)     # Descente B
moveMotor([-45, 45, -90, 0], 7, 0)  # Montee B

moveMotor([0, 0, 0, 0], 2, 0)       # Move 0
time.sleep(0.5)

#####
##

sim.stopSimulation() # end sim


# Création du fichier csv 'trajectory'
q1, q2, q4, q3, gripper = all_positions[0], all_positions[1], all_positions[2], all_positions[3], all_positions[4]
donnees = list(zip(q1, q2, q4, q3, gripper))
with open('trajectory.csv', 'w', newline='') as fichier_csv:
       writer = csv.writer(fichier_csv)
       for ligne in donnees:
              writer.writerow(ligne)
