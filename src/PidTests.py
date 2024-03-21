from unittest import TestCase, main
from PIDController import PIDController
from math import isclose

class TestPIDController(TestCase):

    def test_simple_case(self):
        pctr = PIDController()
        move = pctr.calculate(0, 1, 1)
        self.assertTrue(isclose(move, 1.51, rel_tol=1e-7))

    def test_same_point(self):
        pctr = PIDController()
        move = pctr.calculate(0, 0, 1)
        self.assertEqual(move, 0)
    
    def test_negative_numbers(self):
        pctr = PIDController()
        move = pctr.calculate(-1, -10, 1) 
        self.assertTrue(isclose(move, -13.59, rel_tol=1e-7))

    def test_two_steps(self):
        pctr = PIDController()
        x0 = 0
        x = 10
        x0 = pctr.calculate(x0, x, 1)
        move = pctr.calculate(x0, x, 1)
        self.assertTrue(isclose(move, -2.800999999999999, rel_tol=1e-7))

    def test_dt_zero(self):
        pctr = PIDController()
        move = pctr.calculate(0, 1, 0)
        self.assertEqual(move, None)

if __name__ == "__main__":
    main()

