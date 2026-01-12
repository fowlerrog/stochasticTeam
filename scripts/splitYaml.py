# python imports
import yaml
import sys
import traceback
import os
from itertools import product

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 3 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python splitYaml.py /path/to/file.yaml var_name1 [var_name2 ...]')
		exit()

	# Load the yaml
	try:
		inFile = sys.argv[1]
		print('Loading', inFile)
		with open(inFile, 'r') as f:
			data = yaml.safe_load(f)
	except Exception:
		print(traceback.format_exc())
		exit()

	# Make sure all variable names are present
	varNames = sys.argv[2:]
	validVars = {k : k in data for k in varNames}
	if any(not v for v in validVars.values()):
		print('Variable(s) not found:', ' '.join([k for k in validVars.keys() if not validVars[k]]))
		exit()

	# Find all unique comboes of values of given variable names
	uniques = {k : set(data[k]) for k in varNames}
	combos = product(*[v for v in uniques.values()])

	# Get data for each combination of values
	for combo in combos:
		labeledCombo = {varNames[i] : combo[i] for i in range(len(varNames))}
		print('Looking for', ' && '.join(' == '.join([str(k),str(v)]) for k,v in labeledCombo.items()))

		# Construct logical mask
		mask = [True] * len(data[varNames[0]])
		for k,v in labeledCombo.items():
			mask = [mask[i] and data[k][i] == v for i in range(len(mask))]

		# Check for empty result
		numValues = sum(mask)
		print('\tFound', numValues, 'values')
		if numValues == 0:
			continue

		# Construct new filename
		fileparts = os.path.splitext(inFile)
		outFile = fileparts[0] + '_' + \
			'_'.join('_'.join([str(k), str(v)]) for k,v in labeledCombo.items()) + \
			fileparts[1]

		# Write new file
		newData = {k : [v[i] for i in range(len(mask)) if mask[i]] for k,v in data.items()}
		try:
			print('Writing to', outFile)
			with open(outFile, 'w') as f:
				yaml.dump(newData, f, default_flow_style=None)
		except Exception:
			print(traceback.format_exc())
