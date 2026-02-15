
import os

from .EnvUtils import envFromParams
from .Constants import planPathResultsFilename
from .RunnerUtils import writeYaml

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
			self.env = envFromParams(envParams)

	def solve(self, points, startPoint=None, endPoint=None):
		"""Solves the routing problem for a set of visited points"""
		pass

	def standardizeSolution(self):
		"""Cleans up solution dict for yaml printing, if necessary"""
		return self.solution

	def printResultsToYaml(self, maxDecimals=2):
		"""Prints solution results to a yaml file"""
		absSavePath = os.path.join(self.params["RUN_FOLDER"], self.params["SAVE_PATH_FOLDER"], planPathResultsFilename)
		writeYaml(self.standardizeSolution(), absSavePath, maxDecimals=maxDecimals)
