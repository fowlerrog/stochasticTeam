
import sys

from pathPlanning.RunnerUtils import loadYamlContents
from pathPlanning.MultiAgentTypeEnvironment import *
from pathPlanning.SingleAgentTypeEnvironment import *

def envFromParamsOrFile(params):
	if isinstance(params, str): # this is a filepath, not params
		params = loadYamlContents(params)
	envClass = getattr(sys.modules[__name__], params['TYPE'])
	return envClass(params)
