import numpy as np
import matplotlib.pyplot as plt

class PIDController:

    # kP, kI, kD - coefficients for PID formula
    def __init__ (self, kP = 1, kI = 0.5, kD = 0.01):
        self.prevErr = 0
        self.prevPrevErr = 0
        self.prevMove = 0
        self.kP = kP 
        self.kI = kI 
        self.kD = kD 
    
    # x0 - initial state 
    # x - final state
    def Calculate (self, x0, x, dt):
        if dt == 0:
            return None
        
        currErr = x - x0
        move = self.prevMove \
             + self.kI * currErr * dt \
             + self.kP * (currErr - self.prevErr) \
             + self.kD * (currErr - 2 * self.prevErr + self.prevPrevErr) / dt
        
        # update prevs 
        self.prevPrevErr = self.prevErr
        self.prevErr = currErr
        self.prevMove = move
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
    xAxis = []
    xArr = [] 
    x0 = 0
    for i in range(count):
        print(x0)
        xAxis.append(i)
        xArr.append(x0)
        move = controller.Calculate (x0, x, dt)
        x0 += move

    xAxis = np.array(xAxis)
    x = np.array(xArr)
    plt.plot(xAxis, x)
    plt.show()
    