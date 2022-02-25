'''
This is the main code for capstone project. It uses TrajectoryGenerator, FeedbackControl and NextStatehelper functions.
All variables are initialied (and can be altered) in main.
Just run the file (python3 main.py) to to generate the csv file (CoppeliaSim.csv) in the working directory. 

'''
import modern_robotics as mr
import numpy as np
from math import sqrt, sin, cos, tan, pi
import matplotlib.pyplot as plt
import logging
from TrajectoryGenerator import TrajectoryGenerator
from FeedbackControl import FeedbackControl
from NextState import NextState, h_generator, h_matrix_generator


# init log
log_name = 'newTask.log'
logging.basicConfig(filename=log_name, level=logging.INFO)

def plot(x_error_list,kp,ki):
    '''
    Plot helper function.
    '''
    plt.figure()
    time = np.linspace(1, 16, len(x_error_list))
    plt.plot(time, x_error_list[:, 0], label='Wx')
    plt.plot(time, x_error_list[:, 1], label='Wy')
    plt.plot(time, x_error_list[:, 2], label='Wz')
    plt.plot(time, x_error_list[:, 3], label='Vx')
    plt.plot(time, x_error_list[:, 4], label='Vy')
    plt.plot(time, x_error_list[:, 5], label='Vz')
    plt.xlim([1, 16])
    plt.xlabel('time (s)')
    plt.ylabel('error')
    plt.title(f'X Error Plot for kp={kp}, ki={ki}')
    plt.legend(loc="best")
    plt.savefig(f'Xerr-kp-{kp}-ki-{ki}.png')
    plt.show()
    

def main():
    # Constants from MR wiki
    radius = 0.0475
    l = 0.235 
    w =  0.15
    k = 1

    # Set Kp and Ki
    kp = 24
    ki = 5
    kp_mat = np.eye(6)*kp
    ki_mat = np.eye(6)*ki
    
    # Set delta t, max vel and total simulation time
    dt = 0.01
    max_velocity = 10
    total_time = 16
    
    # Initialize Transforms

    # End Effector to global initial transform (from mr wiki)
    T_se_initial = np.array([[0,0,1,0],
                            [0,1,0,0],
                            [-1,0,0,0.5],
                            [0,0,0,1]])
    # Cube to global initial transform (from mr wiki)
    T_sc_initial = np.array([[1,0,0,1],
                             [0,1,0,0],
                             [0,0,1,0.025],
                             [0,0,0,1]])
    # Cube final transform
    T_sc_final = np.array([[0,1,0,0],
                           [-1,0,0,-1],
                           [0,0,1,0.025],
                           [0,0,0,1]])

    # Cube to End Effector transform during grasp
    T_ce_grasp = np.array([[-1/sqrt(2),0,1/sqrt(2),0],
                           [0,1,0,0],
                           [-1/sqrt(2),0,-1/sqrt(2),0],
                           [0,0,0,1]])

    # Cube to End Effector transform during standoff
    T_ce_standoff = np.array([[-1/sqrt(2),0,1/sqrt(2), -0.05],
                              [0,1,0,0],
                              [-1/sqrt(2),0,-1/sqrt(2),0.05],
                              [0,0,0,1]])

    # offset from base to arm transform
    T_b0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])
    
    # End-effector to base relative transform
    M_0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])
    
    # B list for arm on th eyou-bot
    Blist =  np.array([[0,0,1,0,0.0330,0],
                    [0,-1,0,-0.5076,0,0],
                    [0,-1,0,-0.3526,0,0],
                    [0,-1,0,-0.2176,0,0],
                    [0,0,1,0,0,0]]).T

    # init config with zeros
    current_config = np.zeros(13)
    # Set initial config
    np.put(current_config, [0,1,5,6], [0.1,-0.2,0.2,-1.6])

    # init error counter with zeros
    ec = np.zeros(6)
    # h = h_matrix_generator(radius,l,w)

    # init robot's H matrix
    h = radius/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]]).T

    h1 = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])
    h1 = np.concatenate((np.zeros((2,4)),h1*radius/4, np.zeros((1,4))), axis=0)

    # Generate Trajectory
    traj = TrajectoryGenerator(T_se_initial,T_sc_initial,T_sc_final,T_ce_grasp,T_ce_standoff,k,write2csv=False)
    
    config_list = []
    x_error_list = []

    # print(len(traj))
    # Iterate for generated end-effector trajectory
    for i in range(len(traj)-1):
        # Split config into individual variables
        q = current_config[0:3]
        joint_angles = current_config[3:8]
        wheel_angles = current_config[8:12]

        # Transforms
        T_sb = np.array([[cos(current_config[0]),-sin(current_config[0]), 0,current_config[1]],
                        [sin(current_config[0]), cos(current_config[0]), 0,current_config[2]],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])

        T_0e = mr.FKinBody(M_0e, Blist, joint_angles)
        T_be = np.matmul(T_b0,T_0e)

        # Compute X, Xd, Xd next using current trajectory configuration
        X_d = np.array([[traj[i][0], traj[i][1], traj[i][2],  traj[i][9]],
                       [traj[i][3], traj[i][4],  traj[i][5], traj[i][10]],
                       [traj[i][6], traj[i][7],  traj[i][8], traj[i][11]],
                       [0, 0, 0, 1]])
        X_dn = np.array([[traj[i+1][0], traj[i+1][1], traj[i+1][2],  traj[i+1][9]],
                        [traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
                        [traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
                        [0, 0, 0, 1]])
        X = np.matmul(T_sb,T_be)

        # Call FeedbackControl and get v_twist, controls, x_error, ec
        v_twist, controls, x_error, ec = FeedbackControl(X, X_d, X_dn, kp_mat, ki_mat, dt, h1, T_b0, M_0e, Blist, current_config,ec,debug=False)
        
        # Alter control elements 
        controls = np.concatenate((controls[4:9], controls[:4]), axis=None)

        # Update current config using NextState
        current_config = NextState(current_config, controls, dt, max_velocity,h)
        config_output = np.concatenate((current_config,traj[i][-1]), axis=None)
        config_list.append(config_output)
        x_error_list.append(x_error)
    
    # Save config list and x_error to csv
    config_list = np.array(config_list)
    x_error_list = np.array(x_error_list)
    
    # Init File
    filename1 = "CoppeliaSim.csv"
    file1 = open(filename1, "w")
    np.savetxt(file1, config_list, delimiter=",", fmt='%f')
    logging.info('Generating animation csv file...')
    file1.close()

    filename2 = "error.csv"
    file2 = open(filename2, "w")
    np.savetxt(file2, x_error_list, delimiter=",", fmt='%f')
    logging.info('Writing error plot data.')
    file2.close()
    
    # Plot using helper function
    plot(x_error_list,kp,ki)
    logging.info('Done.')
    

if __name__ == '__main__':
    main()