
import os

from .EnvUtils import envFromParamsOrFile

class Planner(object):
	"""Defines a Planner class which all other Planners should descend from"""
	params = {} # input parameters
	solution = {} # results of solving
	timeInfo = {} # solution timing information
	env = None # environment model, if necessary

	def __init__(self, params):
		"""
		Constructor:
		params = dict of parameters
		"""
		self.params = params

		if 'ENVIRONMENT' in params:
			envParams = params['ENVIRONMENT']
			if isinstance(envParams, str): # this is a path to another file, not params
				absFolder = params['RUN_FOLDER'] if 'RUN_FOLDER' in params else ''
				envParams = os.path.join(absFolder, envParams)
			self.env = envFromParamsOrFile(envParams)

	def solve(self, points):
		"""Solves the routing problem for a set of visited points"""
		pass
