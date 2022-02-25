'''
This code generates a trajectory for end effector to grasp a cube.
All variables are initialied (and can be altered) in main.
Just run the file (python3 TrajectoryGenerator.py) to generate the csv file (Trajectory.csv) in the working directory. 

Main function calls TrajectoryGenerator function which generates the trajectory.
'''
import modern_robotics as mr
import numpy as np
from math import sqrt


    

def TrajectoryGenerator(T_se_initial,T_sc_initial,T_sc_final,T_ce_grasp,T_ce_standoff,k,write2csv):

    
    tg = 3
    n_d = 500
    n_p = 100
    # Compute Transmorms

    # End Effector to Start Standoff Transform
    T_se_standoff_s = np.dot(T_sc_initial, T_ce_standoff)

    # End Effector to Start Grasp Transform
    T_se_grip_open_s = np.dot(T_sc_initial, T_ce_grasp)

    # End Effector to Final Standoff Transform
    T_se_standoff_g = np.dot(T_sc_final, T_ce_standoff)

    # End Effector to Final Grasp Transform
    T_se_grip_open_g = np.dot(T_sc_final, T_ce_grasp)

    # initial to start standoff
    traj_1 = mr.ScrewTrajectory(T_se_initial, T_se_standoff_s, tg, n_d, 5)
    state_array_1 = traj2array(traj_1,0,'None')


    # start standoff to cube start and grip close
    traj_2 = mr.ScrewTrajectory(T_se_standoff_s, T_se_grip_open_s, tg, n_p, 5)
    state_array_2 = traj2array(traj_2,0,'Close')


    # grip open to start standoff 
    traj_4 = mr.ScrewTrajectory(T_se_grip_open_s, T_se_standoff_s, tg, n_p, 5)
    state_array_4 = traj2array(traj_4,1,'None')


    # start standoff to goal standoff
    traj_5 = mr.ScrewTrajectory(T_se_standoff_s, T_se_standoff_g, tg, n_d, 5)
    state_array_5 = traj2array(traj_5,1,'None')



    # goal standoff to cube goal and grip open
    traj_6 = mr.ScrewTrajectory(T_se_standoff_g, T_se_grip_open_g, tg, n_p, 5)
    state_array_6 = traj2array(traj_6,1,'Open')


    # grip open to goal standoff
    traj_8 = mr.ScrewTrajectory(T_se_grip_open_g, T_se_standoff_g, tg, n_p, 5)
    state_array_8 = traj2array(traj_8,0,'None') 

    print('Trajectory Generated!')

    # print(state_list)

    if write2csv:
        # Init File
        filename = "Trajectory.csv"
        file = open(filename, "w")
        # Save to file as csv
        np.savetxt(file, state_array_1, delimiter=",")
        np.savetxt(file, state_array_2, delimiter=",")
        np.savetxt(file, state_array_4, delimiter=",")
        np.savetxt(file, state_array_5, delimiter=",")
        np.savetxt(file, state_array_6, delimiter=",")
        np.savetxt(file, state_array_8, delimiter=",")

        file.close()


    trajectory = state_array_1 + state_array_2 + state_array_4 + state_array_5 + state_array_6 + state_array_8

    return trajectory

def traj2array(traj,gripper_state,grip):
    # init empty traj array
    traj_array = []
    
    # For every array (transform) in generated trajectory, extract elements from transform and append to array
    for i in range(len(traj)):
        r11, r12, r13, px, r21, r22, r23, py, r31, r32, r33, pz, gripper_state = traj[i][0][0], traj[i][0][1], traj[i][0][2], traj[i][0][3], traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][1][3], traj[i][2][0], traj[i][2][1], traj[i][2][2], traj[i][2][3], gripper_state
        traj_array.append([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state])

    # if grip is to be closed/opened, add 63 lines of same transform with altered gripper state
    if grip =='Close':
        for i in range(63):
            r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state = traj_array[-1][0], traj_array[-1][1], traj_array[-1][2], traj_array[-1][3], traj_array[-1][4], traj_array[-1][5], traj_array[-1][6], traj_array[-1][7], traj_array[-1][8], traj_array[-1][9], traj_array[-1][10], traj_array[-1][11], traj_array[-1][12]   
            traj_array.append([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 1])
    
    elif grip == 'Open':
        for i in range(63):
            r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state = traj_array[-1][0], traj_array[-1][1], traj_array[-1][2], traj_array[-1][3], traj_array[-1][4], traj_array[-1][5], traj_array[-1][6], traj_array[-1][7], traj_array[-1][8], traj_array[-1][9], traj_array[-1][10], traj_array[-1][11], traj_array[-1][12]   
            traj_array.append([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, 0])

    return traj_array



def main():
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

    k = 1

    # Generate Trajectory
    trajectory = TrajectoryGenerator(T_se_initial,T_sc_initial,T_sc_final,T_ce_grasp,T_ce_standoff,k,write2csv=True)




if __name__ == '__main__':
    main()