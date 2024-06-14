import numpy as np
from utils import *
from copy import deepcopy
from scipy.optimize import minimize
from parameters import *

class MPC:
	def __init__(self, horizon):
		self.horizon = horizon
		self.R = np.diag([0.01, 0.01])                 # input cost matrix
		self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
		self.Q = np.diag([1.0, 1.0])                   # state cost matrix
		self.Qf = self.Q							   # state final matrix

	def cost(self, u_k, car, goal_x):
		goal_x = np.array(goal_x)
		controller_car = deepcopy(car)
		u_k = u_k.reshape(self.horizon, 2).T
		z_k = np.zeros((2, self.horizon+1))

		desired_state = goal_x

		cost = 0.0

		for i in range(self.horizon):
			controller_car.set_robot_velocity(u_k[0,i], u_k[1,i])
			controller_car.update(DELTA_T)
			x, _ = controller_car.get_state()
			z_k[:,i] = [x[0, 0], x[1, 0]]
			cost += np.sum(self.R@(u_k[:,i]**2))
			cost += np.sum(self.Q@((desired_state-z_k[:,i])**2))
			if i < (self.horizon-1):     
				cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))

		return cost

	def optimize(self, car, goal_x):
		bnd = [(MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY), (MIN_WHEEL_ROT_SPEED_RAD, MAX_WHEEL_ROT_SPEED_RAD)]*self.horizon
		result = minimize(self.cost, args=(car, goal_x), x0 = np.zeros((2*self.horizon)), method='SLSQP', bounds = bnd)
		return result.x[0],  result.x[1]