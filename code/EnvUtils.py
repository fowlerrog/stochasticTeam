
import sys

from RunnerUtils import loadYamlContents
from MultiAgentTypeEnvironment import *
from SingleAgentTypeEnvironment import *

def envFromParamsOrFile(params):
	if isinstance(params, str): # this is a filepath, not params
		params = loadYamlContents(params)
	envClass = getattr(sys.modules[__name__], params['TYPE'])
	return envClass(params)
