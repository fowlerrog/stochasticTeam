
import sys

from .RunnerUtils import loadYamlContents
from .MultiAgentTypeEnvironment import *
from .SingleAgentTypeEnvironment import *
from .Constants import environmentSettingsFilename

def envFromParams(params):
	envClass = getattr(sys.modules[__name__], params['TYPE'])
	return envClass(params)

def envFromFile(filepath, verbose=True):
	envParams = loadYamlContents(filepath, defaultFilename=environmentSettingsFilename, verbose=verbose)
	return envFromParams(envParams)
