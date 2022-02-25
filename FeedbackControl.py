'''
This code uses current configuraton, Kp, Ki and returns twist and x_error
All variables are initialied (and can be altered) in main.
Just run the file (python3 FeedbackControl.py) to run the code. The main function prints test variables in the terminal.

Main function calls FeedbackControl function which computes youbot's kinematics and implements PI feedback control.
'''
import modern_robotics as mr
import numpy as np
from math import sqrt, sin, cos, tan, pi

def FeedbackControl(X, X_d, X_dn, kp, ki, dt, h, T_b0, M_0e, Blist, current_config,ec,debug):
    '''
    This function computes the youbot's kinematics and implements PI feedback control.
    '''
    # Split config into individual variables
    q = current_config[0:3]
    joint_angles = current_config[3:8]
    wheel_angles = current_config[8:12]

    # Compute Transforms T_0e offset arm to end effector, T_eb end-effector to base
    T_0e = mr.FKinBody(M_0e, Blist, joint_angles)
    
    T_eb = np.matmul(np.linalg.inv(T_0e), np.linalg.inv(T_b0))

    # Compute feed forward ref twise
    X_inv = np.linalg.inv(X)
    X_d_inv = np.linalg.inv(X_d)
    X_dn_inv = np.linalg.inv(X_dn)
    inp = np.matmul(X_d_inv, X_dn)/dt
    Vd = mr.se3ToVec(mr.MatrixLog6(inp))

    # Compute Ad x1 xd Vd
    adj_x1 = mr.Adjoint(np.matmul(X_inv, X_d))
    adj_vd = np.matmul(adj_x1, Vd)
    
    # add error integral
    x_error = mr.se3ToVec(mr.MatrixLog6(np.matmul(X_inv, X_d)))
    ec = ec+ x_error*dt
    v_twist = adj_vd + np.matmul(kp, x_error) + np.matmul(ki, ec)

    # Calculate Jacobians
    J_arm = mr.JacobianBody(Blist, joint_angles)
    J_base = np.matmul(mr.Adjoint(T_eb), h)
    J_end = np.concatenate((J_base, J_arm), axis=1)
    J_inv = np.linalg.pinv(J_end)

    # Compute Wheel and arm control outputs
    controls = np.matmul(J_inv, v_twist)

    if debug:
        print(f"Vd: {Vd} \n adj_vd: {adj_vd} \n v_twist: {v_twist} \n x_error: {x_error}    \n  J_end: {J_end} \n  (u,theta dot): {controls} ))")  
    return v_twist, controls, x_error, ec



def main():
    np.set_printoptions(suppress=True)

    # Constants from MR wiki
    radius = 0.0475
    l = 0.235 
    w =  0.15

    # Initialize Transforms

    T_b0 = np.array([[1,0,0,0.1662],
                    [0,1,0,0],
                    [0,0,1,0.0026],
                    [0,0,0,1]])
    
    M_0e = np.array([[1,0,0,0.033],
                    [0,1,0,0],
                    [0,0,1,0.6546],
                    [0,0,0,1]])
    
    Blist =  np.array([[0,0,1,0,0.0330,0],
                    [0,-1,0,-0.5076,0,0],
                    [0,-1,0,-0.3526,0,0],
                    [0,-1,0,-0.2176,0,0],
                    [0,0,1,0,0,0]]).T

    # Initialize H matrix
    h = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])
    h = np.concatenate((np.zeros((2,4)),h*radius/4, np.zeros((1,4))), axis=0)


    # Init Test Variables
    X_d = np.array([[0,0,1,0.5],
                [0,1,0,0],
                [-1,0,0,0.5],
                [0,0,0,1]])
    X_dn = np.array([[0,0,1,0.6],
                [0,1,0,0],
                [-1,0,0,0.3],
                [0,0,0,1]])
    X = np.array([[0.170,0,0.985,0.387],
                [0,1,0,0],
                [-0.985,0,0.170,0.570],
                [0,0,0,1]])

    kp = 0
    ki = 0
    kp_mat = np.eye(6)*kp
    ki_mat = np.eye(6)*ki
    dt = 0.01
    current_config = np.zeros(13)
    np.put(current_config, [5,6], [0.2,-1.6])
    ec = np.zeros(6)

    FeedbackControl(X, X_d, X_dn, kp_mat, ki_mat, dt, h, T_b0, M_0e, Blist, current_config,ec,debug=True)

if __name__ == '__main__':
    main()