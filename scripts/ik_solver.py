import math
import numpy as np
from scipy.optimize import minimize

def forward_kinematics(q,L):
    q1,q2 = q
    L1,L2 = L
    x = L1 *  np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 *  np.sin(q1) + L2 * np.sin(q1 + q2)
    return np.array([x, y])

def objective_function(q, L, target):
    end_effector = forward_kinematics(q, L)
    error = np.linalg.norm(end_effector - target)
    return error

def inverse_kinematics(target, L,initial_guess=[0.0,0.0]):
    result =  minimize(
                    objective_function,
                    initial_guess,
                    args=(L, target),
                    bounds=[(-np.pi, np.pi), (-np.pi, np.pi)]
    )

    return result.x
                    
