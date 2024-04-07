import numpy as np
import matplotlib.pyplot as plt

class PIDController:

    # kP, kI, kD - coefficients for PID formula
    def __init__ (self, kP = 1, kI = 0.5, kD = 0.01):
        self.prev_err = 0
        self.prev_prev_err = 0
        self.prev_move = 0
        self.kP = kP 
        self.kI = kI 
        self.kD = kD 
    
    # x0 - initial state 
    # x - final state
    def calculate (self, x0, x, dt):
        if dt == 0:
            return None
        
        curr_err = x - x0
        move = self.prev_move \
             + self.kI * curr_err * dt \
             + self.kP * (curr_err - self.prev_err) \
             + self.kD * (curr_err - 2 * self.prev_err + self.prev_prev_err) / dt
        
        # update prevs 
        self.prev_prev_err = self.prev_err
        self.prev_err = curr_err
        self.prev_move = move
        return move 

# visualization for testing different coeffs
if __name__ == "__main__":
    # paremetrs
    kp = 0.1
    ki = 0.05
    kd = 0.001 
    dt = 1
    x = 1
    count = 100
    
    controller = PIDController(kp, ki, kd)
    x_axis = []
    x_arr = [] 
    x0 = 0
    for i in range(count):
        print(x0)
        x_axis.append(i)
        x_arr.append(x0)
        move = controller.calculate (x0, x, dt)
        x0 += move

    x_axis = np.array(x_axis)
    x = np.array(x_arr)
    plt.plot(x_axis, x)
    plt.show()
    