# python imports
import yaml
import sys
import traceback
import os

# project imports
from pathPlanning.RunnerUtils import extendDict

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python combineYamls.py /path/to/file1.yaml [/path/to/file2.yaml ...] /path/to/output.yaml')
		exit()
	
	# Load each yaml and append to output
	data = {}
	for s in sys.argv[1:len(sys.argv)-1]:
		try:
			print('Loading', s)
			with open(s, 'r') as f:
				data = extendDict(data, yaml.safe_load(f))
		except Exception:
			print(traceback.format_exc())

	outFile = sys.argv[-1]
	if os.path.isfile(outFile):
		print('Overwrite', outFile, '? (Y/n) ')
		x = input()
		if x.lower() != 'y':
			print('Canceling')
			exit()

	try:
		print('Writing to', outFile)
		with open(outFile, 'w') as f:
			yaml.dump(data, f, default_flow_style=None)
	except Exception:
		print(traceback.format_exc())
