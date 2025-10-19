
import os
import json
import traceback

def appendDict(d1, d2):
	"""Appends the values in d2 to the values of d1, for matching keys"""
	for k in d2.keys():
		if k in d1:
			d1[k].append(d2[k])
		else:
			d1[k] = [d2[k]]
	return d1

def loadJsonContents(settingsFile, defaultFilename = ''):
	"""Returns the contents of a json file, if it exists"""
	# check if we were given the folder instead
	if os.path.isdir(settingsFile):
		settingsFile = os.path.join(settingsFile, defaultFilename)
	print('Loading', settingsFile)
	absFile = os.path.abspath(settingsFile)

	# load run parameters from json
	params = {}
	with open(absFile, 'r') as f:
		try:
			params = json.load(f)
		except Exception:
			print(traceback.format_exc())
	if len(params) == 0:
		print('Params not found')
		return {}

	return params

def writeJson(dataDict, savePath):
	"""Writes a data dictionary to a path"""
	# create folder if it doesn't exist
	absPath = os.path.abspath(savePath)
	absFolder = os.path.dirname(absPath)
	if not os.path.exists(absFolder):
		os.makedirs(absFolder)
	with open(savePath, 'w') as f:
		print('Writing to %s'%savePath)
		json.dump(dataDict, f)

def toDir(path):
	"""Converts a folder or file path to a folder path"""
	return path if os.path.isdir(path) else os.path.dirname(path)
