import numpy as np
import time
import asyncio
import can
import struct


def calculate_RMSE(N,x1,x2):
    somme=0
    for i in range (N): 
        somme +=(x1[i]-x2[i])**2
    RMSE = np.sqrt(somme/N)
    return RMSE



def move_to_zero_position(qi,qf,tf):
    
    
    nb_joints=len(qi)
    
    A=[1,0,0,0]
    A=np.vstack((A,[0,1,0,0]))
    A=np.vstack((A,[1,tf,tf*tf,tf*tf*tf]))
    A=np.vstack((A,[0,1,2*tf,3*tf*tf]))

    for ii in range(nb_joints):
        coeff=np.matmul( np.linalg.inv(A), [qi[ii],0,qf[ii],0] )
    
    print(coeff)
    
    return  

def sleep(duration, get_now=time.perf_counter):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()

def generate_poly3_traj(pi,pf,tf,param):
    pi = np.atleast_1d(pi)
    pf = np.atleast_1d(pf)
    nb_elements=len(pi)
    ts=param["ts"]
    A=[1,0,0,0]
    A=np.vstack((A,[0,1,0,0]))
    A=np.vstack((A,[1,tf,tf*tf,tf*tf*tf]))
    A=np.vstack((A,[0,1,2*tf,3*tf*tf]))

    p=np.empty((nb_elements,int(tf/ts)))
    t=np.linspace(0,tf,int(tf/ts))
 
    for ii in range(nb_elements):
        coeff=np.matmul( np.linalg.inv(A), [pi[ii],0,pf[ii],0] )
        p[ii,:]=coeff[0]+coeff[1]*t+coeff[2]*t*t+coeff[3]*t*t*t
    
    # If only one trajectory is requested, return a flattened array
    if nb_elements == 1:
        return p.flatten()
    else:
        return p

def calculate_rolling_average(data, n,index):
    #Calculate the rolling average of the last n values for each component in the input data array.
    #Parameters:
    #- data: The input data array .
    #- n: The number of values to consider for the rolling average.
    #Returns:
    #- A new array containing the rolling averages for each component.
    if index > n:
        window = data[-n:]
    else:
        window = data[-index:]
    average = sum(window) / len(window)

    return average



read_multi_angle_command = [0x92, 0, 0, 0, 0, 0, 0, 0]
read_torque_command = [0x9C, 0, 0, 0, 0, 0, 0, 0]

class MotorController:
    def __init__(self, param):
        self.bustype = "socketcan"
        self.bus_index = param["bus_index"]
        self.bitrate=1000000
        self.channel = param['channel']
        bus = can.Bus(channel = self.channel, bustype = self.bustype, index = self.bus_index, bitrate = self.bitrate)
        self.bus = bus
        self.ID = param['motor_id']
        self.position = 0.0
        self.torque = 0
        self.velocity = 0
        self.reduction_pos = 1
        self.compteur = 0
        self.previous_time = time.time()
        #self.sendPosition(0.0,50)
        # Start the notifier to continuously listen for messages
        self.notifier = can.Notifier(self.bus, [self.process_message])
        #self.sendPosition(0.0,50)
        
        

    def canSend(self, data):
    # This function sends data to the motors via CAN 
    # INPUT: data as 8 byte structure crated by struct.pack("xxxxxxxx",value1,value2,...)
    # OUTPUT: 8 bytes answered by the motor via CAN, to read them struct.unpack("xxxxxxxx",answer)
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=0x140 + self.ID  ,data=data))
        
    def canSendAll(self, data):
        self.bus.send(can.Message(is_extended_id=False, arbitration_id=0x280    ,data=data))


    def update_position_on_response(self, data):
        #current_time = time.time()
        #print(f'{self.ID} : {current_time-self.previous_time}')
        #self.previous_time = current_time
        #data = struct.unpack("Bxxxi", response_data)
        if abs((0.01 * self.reduction_pos * data[1])-self.position)<30: #Check for communication or numerical errors
            self.position = 0.01 * self.reduction_pos * data[1]  # If ok update, otherwise keep previous value
        else:
            self.position = 0.01 * self.reduction_pos * data[1]

    def update_torque_speed_on_response(self, data):
        #data = struct.unpack("Bxxxi", response_data)
        Ki=1
        if abs((0.01*(1/self.reduction_pos)*Ki*data[2])-self.torque)<2: #Check for communication or numerical errors
            self.torque = 0.01*(1/self.reduction_pos)*Ki*data[2]  # If ok update, otherwise keep previous value
        if abs((self.reduction_pos*data[3] )-self.velocity)<1000:
            self.velocity = self.reduction_pos*data[3]  # If ok update, otherwise keep previous value

    def sendTorque(self,tau_pre_reduct):
        Ki=1   # Torque-current gain of the motors
        i=0
        tau_max=9
        tau = tau_pre_reduct
        #tau[-1]=tau[-1]*0.8 # to consider the reduction in the 3DOF
        if abs(tau)>tau_max:
            tau=np.sign(tau)*tau_max     # Saturate at tau_max to avoid limits
        i = (1/Ki)*(self.reduction_pos)*tau # motor gain * reduction axe * value
        #i = 0
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("Bxxxhxx",0xA1,int(i*100)))# LSB is 0.01       
        time.sleep(0.002)

    def sendPosition(self,pos,vel_max):
        pos_max = 1000000000000
        if abs(pos)>pos_max:
            pos=np.sign(pos)*pos_max     # Saturate at tau_max to avoid limits
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("Bxhi",0xA4,int(vel_max*(1/self.reduction_pos)),int(pos*100*(1/self.reduction_pos)))) # LSB is 0.01
        time.sleep(0.002) 
        
    def sendPosRot(self,rot,pos,vel_max):
        pos_max = 600
        if abs(pos)>pos_max:
            pos=np.sign(pos)*pos_max     # Saturate at tau_max to avoid limits
        self.canSend(read_multi_angle_command)
        time.sleep(0.001)
        self.canSend(struct.pack("BBhi",0xA6, rot, int(vel_max*(1/self.reduction_pos)), int(pos*100*(1/self.reduction_pos)))) # LSB is 0.01
        time.sleep(0.002)
        
    def readVersion(self):
        self.canSend([0xB2, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.002)
    
    def update_current_position_on_response(self,data):
        print(data)

    def readMotorStatus(self):
        self.canSend(read_multi_angle_command)
        time.sleep(0.002)
        self.canSend(read_torque_command)
        time.sleep(0.002)

    def sendSpeed(self, speed_dps):
        speedControl = int(speed_dps * 100)  # Convert dps to 0.01dps/LSB
        self.canSend(struct.pack("Bxxxi", 0xA2, speedControl))
        time.sleep(0.002)
        
    def active_reply_function_command(self):
        self.canSend([0xB6, 0x92, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.002)
        
    def testExemple(self):
        print('pass')
        self.canSendAll([0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.004)

    def process_message(self, message):
        # Process the received message here
        message_type = struct.unpack("Bxxxxxxx", message.data)

        if (message.arbitration_id == 0x240 + self.ID):
            if hex(message_type[0]) == "0x92":
                data = struct.unpack("Bxxxi", message.data)
                self.compteur+=1
                self.update_position_on_response(data)
            if hex(message_type[0]) == "0xa1" or hex(message_type[0]) == "0xa4" or hex(message_type[0]) == "0x9c":
                data = struct.unpack("BBhhh",message.data)
                self.update_torque_speed_on_response(data)
            
            if hex(message_type[0]) == "0xa6":
                data = struct.unpack("BBhhh", message.data)
                self.update_current_position_on_response(data)
                
            if hex(message_type[0]) == "0xb2":
                data = struct.unpack("Bxxxi", message.data)
                print(data[1])
            if hex(message_type[0]) == "0x60":
                data = struct.unpack("Bxxxi", message.data)
                print(data)
                

    def stop_bus(self):
        # Stop the notifier when done
        self.notifier.stop()
        self.bus.shutdown()
        
class RobotController():
    def __init__(self, param):
        
        self.bustype = "socketcan"
        self.bus_index = param["bus_index"]
        self.bitrate = 1000000
        self.channel = param['channel']
        bus = can.Bus(channel = self.channel, bustype = self.bustype, index = self.bus_index, bitrate = self.bitrate)
        self.bus = bus
        self.notifier = can.Notifier(self.bus, [self.process_message])
        
        self.IDs = param["IDs"]
        self.max_vels = param["max_vels"]
        
        self.positions = [0.0, 0.0, 0.0]
        self.compteurs = [0, 0, 0]

        param1={"channel":self.channel,
        "bus_index":self.bus_index,
        "motor_id":self.IDs[0]}
        self.motor1 = MotorController(param1)

        param2={"channel":self.channel,
        "bus_index":self.bus_index,
        "motor_id":self.IDs[1]}
        self.motor2 = MotorController(param2)
        
        param3={"channel":self.channel,
        "bus_index":self.bus_index,
        "motor_id":self.IDs[2]}
        self.motor3 = MotorController(param3)
    
    def send3Position(self, pos1, pos2, pos3):
        self.motor1.sendPosition(pos1, self.max_vels[0])
        self.motor2.sendPosition(pos2, self.max_vels[1])
        self.motor3.sendPosition(pos3, self.max_vels[2])
        
    def send3Speed(self, vit1, vit2, vit3):
        self.motor1.sendSpeed(vit1)
        self.motor2.sendSpeed(vit2)
        self.motor3.sendSpeed(vit3)
        
    def send3Torque(self, trq1, trq2, trq3):
        self.motor1.sendTorque(trq1)
        self.motor2.sendTorque(trq2)
        self.motor3.sendTorque(trq3)
        
    def active_reply_function_command(self):
        self.motor1.canSend([0xB6, 0x92, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.002)
        self.motor2.canSend([0xB6, 0x92, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.002)
        self.motor3.canSend([0xB6, 0x92, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        time.sleep(0.002)

    def readMotorStatus(self):
        self.motor1.readMotorStatus()
        self.motor2.readMotorStatus()
        self.motor3.readMotorStatus()
        
    def process_message(self, message):
        message_type = struct.unpack("Bxxxxxxx", message.data)
        for index in range(len(self.IDs)):
            if (message.arbitration_id == 0x240 + self.IDs[index]):
                if hex(message_type[0]) == "0x92":
                    data = struct.unpack("Bxxxi", message.data)
                    self.positions[index] = 0.01 * data[1]
                    self.compteurs[index] += 1
                
    def stop_bus(self):
        self.notifier.stop()
        self.bus.shutdown()
        
        
        