import pytest 

from aruco_follower.pid_controller import PIDController
from math import isclose

def test_simple_case():
    pctr = PIDController()
    move = pctr.calculate(0, 1, 1)
    assert (isclose(move, 1.51, rel_tol=1e-7))

def test_same_point():
    pctr = PIDController()
    move = pctr.calculate(0, 0, 1)
    assert move == 0

def test_negative_numbers():
    pctr = PIDController()
    move = pctr.calculate(-1, -10, 1) 
    assert (isclose(move, -13.59, rel_tol=1e-7))

def test_two_steps():
    pctr = PIDController()
    x0 = 0
    x = 10
    x0 = pctr.calculate(x0, x, 1)
    move = pctr.calculate(x0, x, 1)
    assert (isclose(move, -2.800999999999999, rel_tol=1e-7))

def test_dt_zero():
    pctr = PIDController()
    move = pctr.calculate(0, 1, 0)
    assert move == None



