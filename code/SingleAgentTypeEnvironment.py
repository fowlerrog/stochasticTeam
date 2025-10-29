
from math import sqrt
from random import gauss
from scipy.spatial.distance import euclidean

class SingleAgentTypeEnvironment(object):
	"""Environmental model responsible for estimating and generating travel times for a single agent type"""

	def __init__(self, params):
		self.params = params

	def estimateMean(self, p1, p2):
		"""Estimates mean travel time from p1 to p2"""
		raise NotImplementedError("SingleAgentTypeEnvironment subclass must implement estimateMean(self, p1, p2)")

	def estimateVariance(self, p1, p2):
		"""Estimates mean travel variance from p1 to p2"""
		raise NotImplementedError("SingleAgentTypeEnvironment subclass must implement estimateVariance(self, p1, p2)")

	def evaluate(self, p1, p2):
		"""Evaluates actual random travel time from p1 to p2"""
		raise NotImplementedError("SingleAgentTypeEnvironment subclass must implement evaluate(self, p1, p2)")

class SpeedEnvironment(SingleAgentTypeEnvironment):
	"""Environment that uses a constant speed"""

	def __init__(self, params):
		super().__init__(params)
		self.agent_speed = params['SPEED']

	def estimateMean(self, p1, p2):
		return euclidean(p1, p2) / self.agent_speed

	def estimateVariance(self, p1, p2):
		return 0

	def evaluate(self, p1, p2):
		return self.estimateMean(p1, p2)

class GaussianEnvironment(SingleAgentTypeEnvironment):
	"""
	Environment that directly evaluates a gaussian multiplied by distance
	limited to +- 3 sigma and nonnegative
	"""

	def __init__(self, params):
		super().__init__(params)
		# make sure the required things are here
		self.weight = params['WEIGHT'] # edge weight, time/dist
		# edge weight standard deviation, time/dist
		if 'WEIGHT_VAR' in params:
			self.weight_std_dev = sqrt(params['WEIGHT_VAR'])
		elif 'WEIGHT_STD_DEV' in params:
			self.weight_std_dev = params['WEIGHT_STD_DEV']
		else:
			raise(ImportError('GaussianEnvironment requires WEIGHT_VAR or WEIGHT_STD_DEV'))

	def estimateMean(self, p1, p2):
		"""Mean = dist / speed"""
		return euclidean(p1, p2) * self.weight

	def estimateVariance(self, p1, p2):
		"""var(time) = var(dist / speed) = dist^2 * var(weight)"""
		return (euclidean(p1, p2) * self.weight_std_dev) ** 2

	def evaluate(self, p1, p2):
		return max(0, \
			min(3, max(-3, gauss(0, 1))) \
			* self.weight_std_dev + self.weight) \
			* euclidean(p1, p2)
