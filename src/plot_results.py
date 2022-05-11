from matplotlib import pyplot as plt
import numpy as np

test_K50_0_3 = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_experiments_0.3.npz')
test_K50_0_5 = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_experiments_0.5.npz')
test_K50_0_7 = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_experiments_0.7071.npz')
test_K50_1 = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_experiments_1.0.npz')
test_K50_1_5 = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_experiments_1.5.npz')
test_points = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_points.npz')

#test_K50_0_3 = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/p2p_panda_extended/K50_0_3_p2p.npz')
#test_K50_0_5 = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/p2p_panda_extended/K50_0_5_p2p.npz')
#test_K50_0_7 = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/p2p_panda_extended/K50_0_7_p2p.npz')
#test_K50_1 = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/p2p_panda_extended/K50_1_p2p.npz')
#test_K50_1_5 = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/p2p_panda_extended/K50_1_5_p2p.npz')
#test_points = np.load('/home/userpanda/Desktop/CriticalDamping/src/franka_ros_TUD/python/recorded_points.npz')

joints_0_3 = test_K50_0_3['recorded_position']
joints_0_5 = test_K50_0_5['recorded_position']
joints_0_7 = test_K50_0_7['recorded_position']
joints_1 = test_K50_1['recorded_position']
joints_1_5 = test_K50_1_5['recorded_position']

time_0_3 = test_K50_0_3['time_stamp']
time_0_5 = test_K50_0_5['time_stamp']
time_0_7 = test_K50_0_7['time_stamp']
time_1 = test_K50_1['time_stamp']
time_1_5 = test_K50_1_5['time_stamp']

set_points = test_points['recorded_points']

def norm_time(time_data):
    zero_time = time_data[0, 1]
    return time_data[0, 1:] - zero_time

norm_time_0_3 = norm_time(time_0_3)
norm_time_0_5 = norm_time(time_0_5)
norm_time_0_7 = norm_time(time_0_7)
norm_time_1 = norm_time(time_1)
norm_time_1_5 = norm_time(time_1_5)

for i in range(joints_0_5.shape[0]):
    plt.plot(norm_time_0_3, joints_0_3[i, 1:], label='0.3')
    plt.plot(norm_time_0_5, joints_0_5[i, 1:], label='0.5')
    plt.plot(norm_time_0_7, joints_0_7[i, 1:], label='0.7071')
    plt.plot(norm_time_1, joints_1[i, 1:], label='1.0')
    plt.plot(norm_time_1_5, joints_1_5[i, 1:], label='1.5')
    plot_title = 'joint ' + str(i+1)
    plt.title(plot_title)
    plt.legend()
    plt.show()