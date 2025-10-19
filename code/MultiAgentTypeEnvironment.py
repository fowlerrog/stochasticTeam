
import sys

from SingleAgentTypeEnvironment import *

class MultiAgentTypeEnvironment(object):
	"""Environmental model responsible for estimating and generating travel times for various agent types"""

	def __init__(self, params):
		self.params = params

	def estimateMean(self, p1, p2, agentType):
		"""Estimates mean travel time from p1 to p2 for an agent of given type"""
		raise NotImplementedError("MultiAgentTypeEnvironment subclass must implement estimateMean(self, p1, p2, agentType)")

	def estimateVariance(self, p1, p2, agentType):
		"""Estimates mean travel variance from p1 to p2"""
		raise NotImplementedError("MultiAgentTypeEnvironment subclass must implement estimateVariance(self, p1, p2, agentType)")

	def evaluate(self, p1, p2, agentType):
		"""Evaluates actual random travel time from p1 to p2"""
		raise NotImplementedError("MultiAgentTypeEnvironment subclass must implement evaluate(self, p1, p2, agentType)")

class SplitEnvironment(MultiAgentTypeEnvironment):
	"""Environment type which has distinct SingleAgentTypeEnvironments for each agent type"""

	def __init__(self, params):
		super().__init__(params)

		self.environmentDict = {}
		for agentType, envParams in params['AGENT_ENVIRONMENTS'].items():
			envClass = getattr(sys.modules[__name__], envParams['TYPE'])
			self.environmentDict[agentType] = envClass(envParams)

	def estimateMean(self, p1, p2, agentType):
		return self.environmentDict[agentType].estimateMean(p1, p2)

	def estimateVariance(self, p1, p2, agentType):
		return self.environmentDict[agentType].estimateVariance(p1, p2)

	def evaluate(self, p1, p2, agentType):
		return self.environmentDict[agentType].evaluate(p1, p2)
