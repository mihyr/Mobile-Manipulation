# overshoot
-To test overshoot performance, following PI values were set.
    kp:2
    Ki: 36
- Due to overshoot, the robot experienced bit oscillations due to non-steady state error in the beginning. This can be verified from the x_error plot as well.

- The cube's initial configuration is as follows:
    [1,0,0,1.2],
    [0,1,0,0.5],
    [0,0,1,0.025],
    [0,0,0,1]

- The cube's final configuration is as follows:
    [0,1,0,1],
    [-1,0,0,-1],
    [0,0,1,0.025],
    [0,0,0,1]

- The initial robot configuration is as follows:
    (0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0)