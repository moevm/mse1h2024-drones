import unittest
import PIDController as pid


class TestPIDController(unittest.TestCase):

    def test_simple_case(self):
        pctr = pid.PIDController()
        x0 = 0
        x = 1
        move = pctr.Calculate(x0, x, 1)
        self.assertEqual(move, 1.51)

    def test_same_point(self):
        pctr = pid.PIDController()
        x0 = 0
        x = 0
        move = pctr.Calculate(x0, x, 1)
        self.assertEqual(move, 0)
    
    def test_negative_numbers(self):
        pctr = pid.PIDController()
        x0 = -1
        x = -10
        move = pctr.Calculate(x0, x, 1) 
        self.assertEqual(move, -13.59)

    def test_two_steps(self):
        pctr = pid.PIDController()
        x0 = 0
        x = 10
        move = pctr.Calculate(x0, x, 1)
        x0 = move
        move = pctr.Calculate(x0, x, 1)
        self.assertEqual(move, -2.800999999999999)

if __name__ == "__main__":
    unittest.main()

