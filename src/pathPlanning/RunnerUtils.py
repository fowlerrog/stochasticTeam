# python imports
import os
import yaml
import traceback
import re

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
	regexString = varName + '_([a-zA-Z\d\.\-]+)(?:_|$)' # varName_ followed by (letters, numbers, -, .) followed by _ or end of string
	stringMatch = re.search(regexString, folderName)
	return stringMatch.groups(1)[0]
