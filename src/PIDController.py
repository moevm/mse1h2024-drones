import numpy as np 
import matplotlib.pyplot as plt

class PIDController:

    # kP, kI, kD - coefficients for PID formula
    def __init__ (self, kP = 1, kI = 0, kD = 0):
        self.prevErr = [0,0,0]
        self.prevPrevErr = [0,0,0]
        self.prevMove = [0,0,0]
        self.kP = kP 
        self.kI = kI 
        self.kD = kD 
    
    # x0, y0, z0 - drone position 
    # x, y, z - destination
    # function calc, how much drone must move in three direction for one iteration
    def Calculate (self, x0, y0, z0, x, y, z):
        currErr = [x - x0, y - y0, z - z0]
        move = [0,0,0]
        for i in range(len(currErr)):
            move[i] = self.prevErr[i] + self.kI * currErr[i] + self.kP * (currErr[i] - self.prevErr[i]) + self.kD * (currErr[i] - 2 * self.prevErr[i] + self.prevPrevErr[i])
        
        # update prevs 
        self.prevMove = move
        self.prevPrevErr = self.prevErr
        self.prevErr = currErr
        print(currErr)
        print(move)
        return move
    

if __name__ == "__main__":

    controller = PIDController(1, 0.5, 0.01)

    x0, y0 = 0, 0
    x, y = 10, 10

    xAxis = []
    xArr = [] 
    yArr = []

    for i in range(20):
        print(x0, y0)
        xAxis.append(i)
        xArr.append(x0)
        yArr.append(y0)
        move = controller.Calculate (x0, y0, 0, x, y, 0)
        x0 += move[0] 
        y0 += move[1]
        # xArr.append(move[0])
        # yArr.append(move[1])
    
    x, y = 20, 34

    for i in range(20):
        print(x0, y0)
        xAxis.append(i + 20)
        xArr.append(x0)
        yArr.append(y0)
        move = controller.Calculate (x0, y0, 0, x, y, 0)
        x0 += move[0] 
        y0 += move[1]
        # xArr.append(move[0])
        # yArr.append(move[1])

    xAxis = np.array(xAxis)
    x = np.array(xArr)
    y = np.array(yArr)
    plt.plot(xAxis, x)
    plt.plot(xAxis, y)
    plt.show()
    
    





    