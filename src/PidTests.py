import unittest
import PIDController as pid


class TestPIDController(unittest.TestCase):

    def test_simple_case(self):
        pctr = pid.PIDController()
        x0, y0, z0 = 0, 0, 0 
        x, y, z = 10, 10, 10
        move = pctr.Calculate(x0, y0, z0, x, y, z)
        self.assertEqual(move, [15.1, 15.1, 15.1])

    def test_same_point(self):
        pctr = pid.PIDController()
        x0, y0, z0 = 0, 0, 0 
        x, y, z = 0, 0, 0
        move = pctr.Calculate(x0, y0, z0, x, y, z)
        self.assertEqual(move, [0, 0, 0])
    
    def test_negative_numbers(self):
        pctr = pid.PIDController()
        x0, y0, z0 = -1, -1, -1 
        x, y, z = -10, -10, -10
        move = pctr.Calculate(x0, y0, z0, x, y, z)
        self.assertEqual(move, [-13.59, -13.59, -13.59])

    def test_two_steps(self):
        pctr = pid.PIDController()
        x0, y0, z0 = 0, 0, 0 
        x, y, z = 10, 10, 10
        move = pctr.Calculate(x0, y0, z0, x, y, z)
        x0 += move[0]
        y0 += move[1]
        z0 += move[2]
        move = pctr.Calculate(x0, y0, z0, x, y, z)
        self.assertEqual(move, [-7.901, -7.901, -7.901])

if __name__ == "__main__":
    unittest.main()

