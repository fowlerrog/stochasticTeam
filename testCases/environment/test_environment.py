
# python imports
import os
from statistics import mean, variance

# project imports
from pathPlanning.EnvUtils import envFromParamsOrFile
from pathPlanning.Constants import environmentSettingsFilename

class TestEnvironment:

	def test_split(self):
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		env = envFromParamsOrFile(os.path.join(thisScriptFolder, environmentSettingsFilename))
		assert(env.estimateMean([0,5], [0,0], 'A') == 40) # mean 8 * length 5
		assert(env.estimateMean([1,5], [1,3], 'B') == 14) # mean 7 * length 2
		assert(env.estimateVariance([1,5], [1,3], 'A') == 1) # (stddev 0.5 * length 2) ^ 2
		assert(env.estimateVariance([0,5], [0,0], 'B') == 400) # (stddev 4 * length 5) ^ 2

	def test_gaussian(self):
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		env = envFromParamsOrFile(os.path.join(thisScriptFolder, environmentSettingsFilename))
		numSamples = 10000
		samples = [env.evaluate([0,5], [0,0], 'A') for _ in range(numSamples)]
		assert(abs(mean(samples) - 40) < 1e-1) # mean 8 * length 5
		assert(abs(variance(samples) - 6.25) < 1e-1) # (stddev 0.5 * length 5) ^ 2

	# todo test case with wind
