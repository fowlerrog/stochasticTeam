# python imports
import os
import yaml
import traceback
import re
from itertools import product
from collections.abc import Iterable

# project imports
from .Constants import planPathResultsFilename

def appendDict(d1, d2):
	"""Appends the values in d2 to the values of d1, for matching keys"""
	for k in d2.keys():
		if k in d1:
			d1[k].append(d2[k])
		else:
			d1[k] = [d2[k]]
	return d1

def extendDict(d1, d2):
	"""Extends the values in d1 with the values of d2, for matching keys"""
	for k in d2.keys():
		if k in d1:
			d1[k].extend(d2[k])
		else:
			d1[k] = d2[k]
	return d1

def roundIterable(d, maxDecimals=1):
	"""Rounds all floats in sets, lists, tuples, or dict values"""
	if isinstance(d, float):
		return round(d, maxDecimals)
	elif isinstance(d, dict):
		return {k:roundIterable(v, maxDecimals) for k,v in d.items()}
	elif isinstance(d, list):
		return [roundIterable(e, maxDecimals) for e in d]
	elif isinstance(d, set):
		return {roundIterable(e, maxDecimals) for e in d}
	elif isinstance(d, tuple):
		return (roundIterable(e, maxDecimals) for e in d)
	return d

def loadYamlContents(settingsFile, defaultFilename = '', verbose=True):
	"""Returns the contents of a yaml file, if it exists"""
	# check if we were given the folder instead
	if os.path.isdir(settingsFile):
		settingsFile = os.path.join(settingsFile, defaultFilename)
	if verbose:
		print('Loading', settingsFile)
	absFile = os.path.abspath(settingsFile)

	# load run parameters from yaml
	params = {}
	with open(absFile, 'r') as f:
		try:
			params = yaml.safe_load(f)
		except Exception:
			print(traceback.format_exc())

	if len(params) == 0 and verbose:
		print('Params not found')
	return params

def writeYaml(dataDict, savePath, maxDecimals=1):
	"""Writes a data dictionary to a path"""
	# create folder if it doesn't exist
	absPath = os.path.abspath(savePath)
	absFolder = os.path.dirname(absPath)
	if not os.path.exists(absFolder):
		os.makedirs(absFolder)
	with open(savePath, 'w') as f:
		print('Writing to %s'%savePath)
		yaml.dump(roundIterable(dataDict, maxDecimals), f, default_flow_style=None)

def loadPlanResultsFromFolder(folderPath):
	"""Loads planning results from a yaml in a result folder"""

	absFile = os.path.abspath(os.path.join(folderPath, planPathResultsFilename))
	print('Loading planning results from', absFile)

	# load run parameters from yaml
	params = loadYamlContents(folderPath, planPathResultsFilename)
	return params

def toDir(path):
	"""Converts a folder or file path to a folder path"""
	return os.path.abspath(path) if os.path.isdir(path) else os.path.dirname(path)

def getChildFolders(parentFolder):
	results = []
	for resultsFolder in os.listdir(parentFolder):
		absResultsFolder = os.path.join(parentFolder, resultsFolder)
		if os.path.isdir(absResultsFolder):
			results.append(absResultsFolder)
	return results

def getVarFromString(folderName, varName):
	"""Extracts variable value from a string of the form 'var1_value1_var2_value2' """
	regexString = varName + '_([a-zA-Z\d\.\-]+)(?:_|$)' # varName_ followed by (letters, numbers, -, .) followed by _ or end of string
	stringMatch = re.search(regexString, folderName)
	return stringMatch.groups(1)[0]

def dictToString(d):
	"""Constructs 'var1_value1_var2_value2' from dict"""
	return '_'.join(['%s_%s'%(k,v) for k,v in d.items()])

def getIndependentValueCombos(params):
	"""
	Returns a list of dicts for all combinations of values of independent parameters in a dict
	e.g. {'INDEPENDENT_VARIABLES' : ['A','B'], 'A' : [1,2], 'B' : [3,4]}
		-> [ {'A':1,'B':3}, {'A':1,'B':4}, {'A':2,'B':3}, {'A':2,'B':4} ]
	or None if invalid
	"""

	#TODO this can only find top-level variables
	#TODO allow correlated values

	# make sure independent variables are present
	if 'INDEPENDENT_VARIABLES' not in params:
		return [{}]
	independentVars = params['INDEPENDENT_VARIABLES']

	if not isinstance(independentVars, Iterable): # one string
		independentVars = [independentVars]

	notPresent = [k not in params for k in independentVars]
	if any(notPresent):
		print('Did not find required independent variables:\n\t',
		'\n\t'.join([independentVars[i] for i in range(len(notPresent)) if notPresent[i]]),
		'\nStopping run', sep='')
		return None

	# generate all combinations of independent variables
	independentDict = {k:params[k] for k in independentVars}
	independentValueCombos = product(*[v for v in independentDict.values()])

	return [dict(zip(independentVars, combo)) for combo in independentValueCombos]
