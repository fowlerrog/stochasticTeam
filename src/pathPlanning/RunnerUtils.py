# python imports
import os
import yaml
import traceback
import re
from itertools import product
from collections.abc import Iterable
from copy import deepcopy

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
	"""
	Returns the contents of a yaml file, if it exists
	Looks for another yaml file to load, for any value of the form $relative/path/to/filename.yaml
	"""

	# check if we were given the folder instead
	if os.path.isdir(settingsFile):
		settingsFile = os.path.join(settingsFile, defaultFilename)
	if verbose:
		print('Loading', settingsFile)
	absFile = os.path.abspath(settingsFile)
	absFolder = toDir(absFile)

	# load run parameters from yaml
	params = {}
	with open(absFile, 'r') as f:
		try:
			params = yaml.safe_load(f)
		except Exception:
			print(traceback.format_exc())

	# load any recursive yamls
	def replaceFilepaths(d):
		for k,v in d.items():
			if isinstance(v, str) and v[0] == '$':
				d[k] = loadYamlContents(os.path.join(absFolder, v[1:]), verbose=verbose)
			elif isinstance(v, dict):
				d[k] = replaceFilepaths(v)
		return d

	params = replaceFilepaths(params)

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

def fillIndependentVariablesFromString(dataDict, folderName):
	"""
	Fills a dict containing 'INDEPENDENT_VARIABLES'
	by pulling those values out of a string folder name
	"""
	if 'INDEPENDENT_VARIABLES' not in dataDict or len(dataDict['INDEPENDENT_VARIABLES']) == 0:
		return dataDict

	outDict = deepcopy(dataDict)
	for indVar in dataDict['INDEPENDENT_VARIABLES']:
		indVarList = indVar.split('.')
		choices = dictGetRecursive(outDict, indVarList)
		chosen = getVarFromString(folderName, indVar)
		correctInds = [i for i in range(len(choices)) if str(choices[i]) == chosen]
		assert len(correctInds) == 1, f'Found {len(correctInds)} matches for {indVar} == {chosen} in {choices}'
		outDict = dictSetRecursive(outDict, indVarList, choices[correctInds[0]])
	return outDict

def dictToString(d):
	"""Constructs 'var1_value1_var2_value2' from dict"""
	return '_'.join(['%s_%s' % (k if isinstance(k, str) else '.'.join(k), v) for k,v in d.items()])

def dictGetRecursive(dict, keyIter):
	"""Returns dict[keyList[0]][keyList[1]][...]"""
	if keyIter[0] in dict:
		if len(keyIter) == 1:
			return dict[keyIter[0]]
		else:
			return dictGetRecursive(dict[keyIter[0]], keyIter[1:])
	return None

def dictSetRecursive(dict, keyIter, value):
	"""Returns updated dict with dict[keyList[0]][keyList[1]][...] := value"""
	if keyIter[0] in dict:
		if len(keyIter) == 1:
			return dict | {keyIter[0] : value}
		else:
			newSubDict = dictSetRecursive(dict[keyIter[0]], keyIter[1:], value)
			if newSubDict is None:
				return None
			return dict | {keyIter[0] : newSubDict}
	return None

def getIndependentValueCombos(params):
	"""
	Returns a list of dicts for all combinations of values of independent parameters in a dict
		and the full param list updated by each combination
	e.g. {'INDEPENDENT_VARIABLES' : ['A','B'], 'A' : [1,2], 'B' : [3,4]}
		-> [ {'A':1,'B':3}, {'A':1,'B':4}, {'A':2,'B':3}, {'A':2,'B':4} ]
	or None if invalid
	Also, any variable name A.B would refer recursively to {'A':{'B':value}}
	"""

	#TODO allow correlated values e.g. [['A', 'B']] -> [ {A:1, B:1}, {A:2, B:2} ]

	# make sure independent variables are present
	if 'INDEPENDENT_VARIABLES' not in params:
		return [{}], [params]
	independentVars = params['INDEPENDENT_VARIABLES']

	if not isinstance(independentVars, Iterable): # one string
		independentVars = [independentVars]

	# split recursive names 'A.B' into ['A','B'], then get values
	keyTuples = [tuple(ind.split('.')) for ind in independentVars]
	valueLists = [dictGetRecursive(params, keyTuple) for keyTuple in keyTuples]

	# warn user if value not found
	notPresent = [v is None for v in valueLists]
	if any(notPresent):
		print('Did not find required independent variables:\n\t',
		'\n\t'.join(['.'.join(keyTuples[i]) for i in range(len(notPresent)) if notPresent[i]]),
		'\nStopping run', sep='')
		return None, None

	# generate all combinations of independent variables, and construct the full dict with those values replaced
	indVarComboList = []
	paramList = []
	independentValueCombos = product(*[v for v in valueLists])
	for combo in independentValueCombos:
		indVarCombo = dict(zip(keyTuples, combo))
		indVarComboList.append(indVarCombo)
		thisRunParams = deepcopy(params)
		for k,v in indVarCombo.items():
			thisRunParams = dictSetRecursive(thisRunParams, k, v)
		paramList.append(thisRunParams)

	return indVarComboList, paramList
