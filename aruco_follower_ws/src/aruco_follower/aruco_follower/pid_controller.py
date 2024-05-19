import numpy as np

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
