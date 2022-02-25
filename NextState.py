'''
This code uses current configuraton and controls (desired twist) to calculate the next configuration
All variables are initialied (and can be altered) in main.
Just run the file (python3 NextState.py) to generate the csv file (output.csv) in the working directory. 

Main function calls NextState function which generates the next state configurations.
'''
import modern_robotics as mr
import numpy as np
from math import sqrt, sin, cos, tan, pi

def h_generator(radius, gamma, beta, x,y):
    '''
    Generates the 3x1 vector for mobile robot for given radius, gamma, beta, x,y
    '''
    M1 = np.array([1,tan(gamma)])
    M2 = np.array([[cos(beta),sin(beta)],[-sin(beta),cos(beta)]])
    M3 = np.array([[-y,1,0], [x,0,1]])
    
    M23 = np.dot(M2,M3)
    M123 = np.dot(M1,M23)
    Output = (radius/4)*M123
    return Output

def h_matrix_generator(radius,l,w):
    '''
    Generates H(0) matrix for mobile robot
    '''
    h_1 =  h_generator(radius, gamma=-pi/4, beta = 0, x=l,y=w)
    h_2 =  h_generator(radius, gamma=pi/4, beta = 0, x=l,y=-w)
    h_3 =  h_generator(radius, gamma=-pi/4, beta = 0, x=-l,y=-w)
    h_4 =  h_generator(radius, gamma=pi/4, beta = 0, x=-l,y=w)
    h = np.array([h_1,h_2,h_3,h_4])
    
    return h

def u_matrix_generator(h,w,v_x,v_y):
    vb = np.array([w,v_x,v_y])
    u = np.dot(h,vb)
    return u

def max_speed_limiter(controls, max_velocity):
    '''
    Limits the max speed of the robot
    '''
    for i in range(len(controls)):
        if controls[i] > max_velocity:
            controls[i] = max_velocity
        elif controls[i] < -max_velocity:
            controls[i] = -max_velocity
    return controls

def NextState(current_config, controls, dt, max_velocity,h):
    '''
    Generates the next state configuration
    '''
    # Split config into individual variables
    q = current_config[0:3]
    joint_angles = current_config[3:8]
    wheel_angles = current_config[8:12]

    # Limit the max speed of the robot
    control = max_speed_limiter(controls, max_velocity)

    # Split control into individual variables
    joint_control = control[0:5]
    wheel_control = control[5:9]

    # Calculate the next joint angles and wheel angles
    new_joint_angles = joint_angles + joint_control*dt
    new_wheel_angles = wheel_angles + wheel_control*dt

    # MR page 546 where v_b = w, vx,vy
    w, vx, vy = h.T.dot((wheel_control*dt))
    
    # Get heading
    phi = q[0]
    
    # MR page 548
    if w == 0:
        dq_b = np.array([0,vx,vy])
    else:
        dq_b = np.array([w , (vx*sin(w) + vy*(cos(w) -1))/w, (vy*sin(w) + vx*(1-cos(w)))/w])

    T_sb = np.array([[1,0,0],[0,cos(phi),-sin(phi)],[0,sin(phi),cos(phi)]])
    dq_sb = np.dot(T_sb,dq_b)

    # Calculate the next q configuration
    new_q = q + dq_sb

    # Merge the new config and return
    
    return np.concatenate((new_q,new_joint_angles,new_wheel_angles))
    

def main():
    np.set_printoptions(suppress=True)
    
    # Constants from MR wiki
    radius = 0.0475
    l = 0.235 
    w =  0.15
    h = h_matrix_generator(radius,l,w)
    

    # Test values

    current_config = np.zeros(13)
    controls = np.zeros(9)
    np.put(controls, [5,6,7,8], [10,10,10,10])

    dt = 0.01
    max_velocity = 5
    total_time = 1
    

    # Init File
    filename = "output.csv"
    file = open(filename, "w")

    dataset  = []
    for i in range(int(total_time/dt)):
        current_config = NextState(current_config, controls, dt, max_velocity,h)
        dataset.append(current_config)
    dataset = np.array(dataset)
    np.savetxt(file, dataset, delimiter=",", fmt='%f')
    file.close()
    

if __name__ == '__main__':
    main()