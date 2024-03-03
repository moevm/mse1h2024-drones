
class PIDController:

    # kP, kI, kD - coefficients for PID formula
    def __init__ (self, kP = 1, kI = 0.5, kD = 0.01):
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
        return move 
    
if __name__ == "__main__":
    pctr = PIDController()
    x0, y0, z0 = 0, 0, 0 
    x, y, z = 10, 10, 10
    move = pctr.Calculate(x0, y0, z0, x, y, z)
    x0 += move[0]
    y0 += move[1]
    z0 += move[2]
    move = pctr.Calculate(x0, y0, z0, x, y, z)
    
    print(move)
    