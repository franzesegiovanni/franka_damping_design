#%%
#!/usr/bin/env python
#from winreg import REG_EXPAND_SZ
import rospy
# import rosbag
import math
import numpy as np
import time

#from zmq import RECONNECT_IVL_MAX
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from sys import exit
from pynput.keyboard import Listener, KeyCode

class LfD():
    def __init__(self):
        rospy.init_node('LfD', anonymous=True)
        self.r=rospy.Rate(10)
        self.curr_pos=None
        self.curr_joint = None
        self.width=None
        self.recorded_traj = None 
        self.recorded_experiment = None 
        self.recorded_gripper= None
        self.recorded_joint = None 
        self.step_change = 0.25
        self.end = False
        self.save_joint_position = False
        # bag_name = 'test_results-' + time.strftime("%H-%M-%S") + '.bag'
        # self.bag = rosbag.Bag(bag_name, 'w')
        self.recording_state = False
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        #self.results_sub = rospy.Subscriber("/joint_states", JointState, self.rec_results)
        self.joints=rospy.Subscriber("/joint_states", JointState, self.joint_callback)  
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) #TODO check the name of this topic  
        self.joint_pub = rospy.Publisher('/equilibrium_configuration', JointState , queue_size=0)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]
    def joint_callback(self,data):
        self.curr_joint =data.position[0:7]      
        if self.recording_state == True:
            self.recorded_experiment = np.c_[self.recorded_experiment, self.curr_joint]
            self.recorded_time = np.c_[self.recorded_time, time.time()]
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True        
        if key == KeyCode.from_char('j'):
            self.save_joint_position = True  

    def set_non_diag_stiffness(self, stiff):
        stiff_des = Float32MultiArray()
        # stiff_des.data = np.array([pos_stiff[0], pos_stiff[1], pos_stiff[2], rot_stiff[0], rot_stiff[1], rot_stiff[2], null_stiff[0]]).astype(np.float32)
        stiff_des.data = np.array(stiff)
        self.stiff_pub.publish(stiff_des)    

    def set_stiffness_joint(self, k_1, k_2, k_3, k_4, k_5, k_6, k_7):

        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({"joint_1": k_1})
        set_K.update_configuration({"joint_2": k_2})
        set_K.update_configuration({"joint_3": k_3})        
        set_K.update_configuration({"joint_4": k_4}) 
        set_K.update_configuration({"joint_5": k_5}) 
        set_K.update_configuration({"joint_6": k_6})
        set_K.update_configuration({"joint_7": k_7}) 

    def joint_rec_point(self): 
        self.set_stiffness_joint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.recorded_joint = self.curr_joint
        self.recorded_gripper= self.width

        while self.end ==False:       
            if  self.save_joint_position==True: 
                self.recorded_joint = np.c_[self.recorded_joint, self.curr_joint]
                self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
                print('Adding joint position')
                time.sleep(0.5) 
                self.save_joint_position=False   
        np.savez('recorded_points', recorded_joint = self.recorded_joint, recorded_gripper = self.recorded_gripper)
        print('End of the demonstration')        
  

    def joint_rec(self):
        self.set_stiffness_joint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        trigger = 0.05 
        init_joint = self.curr_joint
        vel = 0
        while not(vel > trigger):
            vel= math.sqrt((self.curr_joint[0]-init_joint[0])**2 + (self.curr_joint[1]-init_joint[1])**2 + (self.curr_joint[2]-init_joint[2])**2+ (self.curr_joint[3]-init_joint[3])**2+ (self.curr_joint[4]-init_joint[4])**2+ (self.curr_joint[5]-init_joint[5])**2+(self.curr_joint[6]-init_joint[6])**2)
        print("Recording Trajectory")
        self.recorded_joint = self.curr_joint
        self.recorded_gripper= self.width
        # recorded_joint = joint_pos
        self.end=False
        while not(self.end):
            now = time.time()      

            self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
            self.recorded_joint = np.c_[self.recorded_joint, self.curr_joint]
            self.r.sleep()

        np.savez('recorded_traj', self.recorded_joint, self.recorded_gripper)
        self.end=False    

    def go_to_start_joint(self):
        start = self.curr_joint
        goal_=np.array([self.recorded_joint[0][0], self.recorded_joint[1][0], self.recorded_joint[2][0], self.recorded_joint[3][0], self.recorded_joint[4][0], self.recorded_joint[5][0], self.recorded_joint[6][0]])
        print("goal:", goal_)
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.05
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        q1 = np.linspace(start[0], goal_[0], step_num)
        q2 = np.linspace(start[1], goal_[1], step_num)
        q3 = np.linspace(start[2], goal_[2], step_num)
        q4 = np.linspace(start[3], goal_[3], step_num)
        q5 = np.linspace(start[4], goal_[4], step_num)
        q6 = np.linspace(start[5], goal_[5], step_num)
        q7 = np.linspace(start[6], goal_[6], step_num)
        goal=JointState()
        goal.position=[q1[0],q2[0],q3[0],q4[0],q5[0],q6[0],q7[0]]
        self.joint_pub.publish(goal)
        self.set_stiffness_joint(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0)

        for i in range(step_num):
            goal=JointState()
            goal.position=[q1[i],q2[i],q3[i],q4[i],q5[i],q6[i],q7[i]]
            self.joint_pub.publish(goal)
            self.r.sleep()  

    def home(self):
        start = self.curr_joint
        goal_ = np.array([0.0, 0.0, 0.0, -2.4, 0.0, 2.4, 0.0])
        # goal_=np.array([self.recorded_joint[0][0], self.recorded_joint[1][0], self.recorded_joint[2][0], self.recorded_joint[3][0], self.recorded_joint[4][0], self.recorded_joint[5][0], self.recorded_joint[6][0]])
        print("goal:", goal_)
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.05
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        q1 = np.linspace(start[0], goal_[0], step_num)
        q2 = np.linspace(start[1], goal_[1], step_num)
        q3 = np.linspace(start[2], goal_[2], step_num)
        q4 = np.linspace(start[3], goal_[3], step_num)
        q5 = np.linspace(start[4], goal_[4], step_num)
        q6 = np.linspace(start[5], goal_[5], step_num)
        q7 = np.linspace(start[6], goal_[6], step_num)
        goal=JointState()
        goal.position=[q1[0],q2[0],q3[0],q4[0],q5[0],q6[0],q7[0]]
        self.joint_pub.publish(goal)
        self.set_stiffness_joint(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0)

        for i in range(step_num):
            goal=JointState()
            goal.position=[q1[i],q2[i],q3[i],q4[i],q5[i],q6[i],q7[i]]
            self.joint_pub.publish(goal)
            self.r.sleep()  

    def track_start_joint(self):
        start = self.curr_joint
        goal_=np.array([self.recorded_joint[0][0], self.recorded_joint[1][0], self.recorded_joint[2][0], self.recorded_joint[3][0], self.recorded_joint[4][0], self.recorded_joint[5][0], self.recorded_joint[6][0]])
        print("goal:", goal_)
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.05
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        q1 = np.linspace(start[0], goal_[0], step_num)
        q2 = np.linspace(start[1], goal_[1], step_num)
        q3 = np.linspace(start[2], goal_[2], step_num)
        q4 = np.linspace(start[3], goal_[3], step_num)
        q5 = np.linspace(start[4], goal_[4], step_num)
        q6 = np.linspace(start[5], goal_[5], step_num)
        q7 = np.linspace(start[6], goal_[6], step_num)
        goal=JointState()
        goal.position=[q1[0],q2[0],q3[0],q4[0],q5[0],q6[0],q7[0]]
        self.joint_pub.publish(goal)
        self.set_stiffness_joint(100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 5.0)

        for i in range(step_num):
            goal=JointState()
            goal.position=[q1[i],q2[i],q3[i],q4[i],q5[i],q6[i],q7[i]]
            self.joint_pub.publish(goal)
            self.r.sleep()  


    def execute_joints(self):
        self.set_stiffness_joint(30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 8.0)

        for i in range(np.shape(self.recorded_joint)[1]): 
            goal=JointState()
            goal.position=self.recorded_joint[:,i] 
            self.joint_pub.publish(goal)
            grip_command = Float32()
            grip_command.data = self.recorded_gripper[0,i]
            self.grip_pub.publish(grip_command) 
            self.r.sleep()

    def load_file(self):
        goal_points = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/recorded_points.npz')
        self.recorded_joint = goal_points['recorded_joint']
        self.recorded_gripper = goal_points['recorded_gripper']

    def execute_step_change(self):
        self.recording_state = True
        # K_non_diag = np.diag([80.0, 184.0, 80.0, 76.0, 80.0, 80.0, 20.0])
        K_non_diag = np.diag([80.0, 40.0, 80.0, 40.0, 80.0, 80.0, 20.0])
        # K_non_diag[3, 1] = -72.0
        # K_non_diag[1, 3] = -72.0
        self.set_non_diag_stiffness(np.reshape(K_non_diag, -1))
        self.set_stiffness_joint(100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 20.0)
        self.recorded_experiment= np.array(self.curr_joint)
        self.recorded_time=time.time()
        goal = JointState()
        # goal.position = self.recorded_joint
        goal.position = np.array(self.curr_joint)
        # goal.position=np.array([self.recorded_joint[0][0], self.recorded_joint[1][0], self.recorded_joint[2][0], self.recorded_joint[3][0], self.recorded_joint[4][0], self.recorded_joint[5][0], self.recorded_joint[6][0]])
        print('goal position before step change: ', goal.position)
        goal.position= goal.position + self.step_change
        print('goal position after step change: ', goal.position)
        self.joint_pub.publish(goal)
        time.sleep(4.0)
        #for i in range(np.shape(self.recorded_joint)[1]): 
        self.recording_state = False
        np.savez('recorded_experiments', recorded_joints=self.recorded_experiment, time_stamp=self.recorded_time) 

    def execute_joints_points(self):
        self.recording_state = True
        self.set_stiffness_joint(600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 20.0)
        self.recorded_experiment= self.curr_joint
        self.recorded_time=time.time()
        for i in range(np.shape(self.recorded_joint)[1]): 
            goal=JointState()
            goal.position=self.recorded_joint[:,i]
            print(self.recorded_joint[:,i]) 
            self.joint_pub.publish(goal)
            grip_command = Float32()
            grip_command.data = self.recorded_gripper[0,i]
            self.grip_pub.publish(grip_command) 
            time.sleep(5) 

        self.recording_state = False
        np.savez('recorded_experiments', recorded_joints=self.recorded_experiment, time_stamp=self.recorded_time) 

#%%    
LfD=LfD()

#%%
LfD.joint_rec_point() 
#%%
LfD.load_file()
#%% 
LfD.home()
#%%
LfD.go_to_start_joint()
#%%
LfD.track_start_joint()
#%% 
LfD.execute_step_change()

#%%
# LfD.execute_joints_points()

#%%
LfD.joint_rec() 

#%%
LfD.go_to_start_joint()

#%%
LfD.execute_joints()

# %%
