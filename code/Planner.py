
class Planner(object):
	"""Defines a Planner class which all other Planners should descend from"""
	params = {} # input parameters
	solution = {} # results of solving
	time_info = {} # solution timing information

	def __init__(self, params):
		"""
		Constructor:
		params = dict of parameters
		"""
		self.params = params

	def getRuntimeInfo(self):
		"""Returns runtime info dict"""
		raise NotImplementedError("Planner subclass must implement getRuntimeInfo(self)")

	def solve(self, points):
		"""Solves a """
		pass
