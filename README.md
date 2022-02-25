# Mobile-Manipulation

# Code Structure:
The complete code is structured as follows:
- three helper functions are written in seperate files with their tests in their respecttive main() function.
    - NextState.py - contains helper function NextState() that computes next configuration of the robot
    - TrajectoryGenerator.py - contains helper function TrajectoryGenerator() that generates trajectory of the end-effector
    - FeedbackControl.py - contains helper function FeedbackControl() that computes kinematics and feedback FeedbackControl
- The main.py file uses above helper functions to generate a CoppeliaSim.csv and error.csv for any valid start and goal position of cube

- Please run python3 main.py to generate the csv files and plots
- The variables can be altered in the main() function
- A seperate README for respective results in available in results folder

Algo gist:
- Robot's initial configuration is defined wrt world frame
- Cube's initial and final position is defined wrt world frame
- TrajectoryGenerator() generates a end-effector trajectory configurations
- FeedbackControl() uses this configurations and determines the twise required and also implements PI feedback FeedbackControl
- NextState takes the current configuration and required twist to update the configuration
- All configuratins are saved in csv format to visualize in CoppeliaSim.
- Additionally x_error is also tracked over the time and plotted subsequently.