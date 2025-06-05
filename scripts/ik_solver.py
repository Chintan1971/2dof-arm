from math import pi
import numpy as np
from scipy.optimize import minimize

def forward_kinematics(theta,L=(1.0,1.0)):
    t1,t2 = theta
    l1,l2 = L

    r1 = np.array([
        [np.cos(t1), 0 , np.sin(t1)],
        [0 , 1, 0,],
        [-np.sin(t1), 0, np.cos(t1)]
        ])   
    r2 = np.array([
        [np.cos(t2), -np.sin(t2), 0],
        [np.sin(t2), np.cos(t2), 0],
        [0, 0, 1]
        ])
    T1 = np.eye(4)
    T2 = np.eye(4)

    T1[:3,:3] = r1 
    T1[:3,3] = r1 @ np.array([ l1, 0, 0])
    T2[:3,:3] = r2 
    T2[:3,3] = r2 @ np.array([ l2, 0, 0])
    
    end_effector = T1 @ T2 @ np.array([0, 0, 0, 1])
    joint1 = T1 @ np.array([0, 0, 0, 1])

    return end_effector[:3], joint1[:3]


def objective_function(q,L, target):
    end_effector,_ = forward_kinematics(q,L)
    error = np.linalg.norm(end_effector - target)
    return error

def inverse_kinematics(target, L=(1.0,1.0),initial_guess=[0.0,0.0]):
    result =  minimize(
                    objective_function,
                    initial_guess,
                    args=(L, target),
                    bounds=[(-np.pi, np.pi), (-np.pi, np.pi)]
    )
    if not result.success:
        print("Inverse kinematics failed to converge", result.message)

    return result.x
                    
def plot(theta):
    import matplotlib.pyplot as plt

    end_effector_pos,joint1_pos = forward_kinematics(theta)
    origin = np.array([0,0,0])

    x_pos = [origin[0], joint1_pos[0], end_effector_pos[0]]
    y_pos = [origin[1], joint1_pos[1], end_effector_pos[1]]

    plt.figure()
    plt.plot(x_pos, y_pos, marker='o')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.title('2D Robot Arm')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid()
    plt.show()

if __name__ == "__main__":

    # target = np.array([1.707, 0.707, 0.0])
    # L = (1.0, 1.0)
    # initial_guess = [0.0, 0.0]

    # theta = inverse_kinematics(target, L, initial_guess)
    # print("Calculated joint angles:", theta)

    # end_effector_pos, joint1_pos = forward_kinematics(theta)
    # print("End effector position:", end_effector_pos)
    # print("Joint 1 position:", joint1_pos)
    theta = [5.80335503e-10, 7.85398157e-01]
    plot(theta)

    