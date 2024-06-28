from utils import MotorController
import matplotlib.pyplot as plt
import time

# connexion bus can (linux console) :
#      sudo ip link set can0 up type can bitrate 1000000

hz = 100
ts=1/hz     # change hertz if needed

# Connect can bus for control motors
SO = [21, 18, 20, 8, 33.33]     # Use 'SO' if you use Orange SCARA (ID: 21 | 18 | 20 | 8) (33.33 for endless screw)
SR = [6, 13, 12, 2, 3.333]      # Use 'SR' if you use Red SCARA (ID: 6 | 13 | 12 | 2) (3.333 for endless screw)

params_robots = SR      # <- Use here 'SO' or 'SR'

motor1 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[0]})
motor2 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[1]})
motor3 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[2]})
motor4 = MotorController({"channel":'can0',"bus_index":0,"motor_id":params_robots[3]})


motors = [motor1, motor2, motor3, motor4]
for ms in motors:
    ms.sendTorque(0)     # set motors for collaborative usage
    ms.readMotorStatus()
offsets = [motor1.position, motor2.position, motor3.position, motor4.position]      # Set offsets
print(f'offsets : {offsets[0]}, {offsets[1]}, {offsets[2]}, {offsets[3]}')


positions = [[], [], [], []]    # for plot
    
tf = 10     # Change final time

for t in range(int(tf/ts)):
    t_s = time.time()
    for m in motors:
        m.readMotorStatus()     # readStatus for each motors' positions
    print(f'|| {"%.2f" % (motor1.position-offsets[0])} || {"%.2f" % (motor2.position-offsets[1])} || {"%.2f" % (motor3.position-offsets[2])/params_robots[4]} || {"%.2f" % (motor4.position-offsets[3])} ||')
    
    for i in range(len(motors)):
        positions[i].append(ms[i].position-offsets[i])  # for plot

    while time.time()-t_s < ts:     # wait time
        time.sleep(0.001)


# Plot all motors' positions
plt.figure()
plt.plot(positions[0], label='motor1')
plt.plot(positions[1], label='motor2')
plt.plot(positions[2], label='motor3')
plt.plot(positions[3], label='motor4')
plt.legend()
plt.show()
