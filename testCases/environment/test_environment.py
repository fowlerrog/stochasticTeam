
# python imports
import os
import random
import pytest
import scipy.stats

# project imports
from pathPlanning.EnvUtils import envFromFile
from pathPlanning.Constants import environmentSettingsFilename

class TestEnvironment:

	def test_split(self):
		"""
		Tests whether a SplitEnvironment properly accepts agentType
		and whether estimating mean & variance works
		"""
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		env = envFromFile(os.path.join(thisScriptFolder, environmentSettingsFilename))
		assert env.estimateMean([0,5], [0,0], 'A') == 40 # mean 8 * length 5
		assert env.estimateMean([1,5], [1,3], 'B') == 14 # mean 7 * length 2
		assert env.estimateVariance([1,5], [1,3], 'A') == 1 # (stddev 0.5 * length 2) ^ 2
		assert env.estimateVariance([0,5], [0,0], 'B') == 400 # (stddev 4 * length 5) ^ 2

	@pytest.mark.parametrize("randomSeed", range(5))
	def test_gaussian(self, randomSeed):
		"""Tests whether gaussian evaluations are actually gaussian"""
		# load env_settings.yaml
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		env = envFromFile(os.path.join(thisScriptFolder, environmentSettingsFilename))
		# sample
		numSamples = 10000
		random.seed(randomSeed)
		samples = [env.evaluate([0,5], [0,0], 'A') for _ in range(numSamples)]
		# calculate statistics
		mean, stddev = scipy.stats.norm.fit(samples)
		# assert standard values
		#	note that these are relaxed, because the environment limits to +- 3 sigma
		assert abs(mean - 40) < 2e-1 # mean 8 * length 5
		assert abs(stddev - 2.5) < 2e-1 # (stddev 0.5 * length 5) ^ 2
