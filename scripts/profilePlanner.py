# python imports
import sys
import traceback
import cProfile
import pstats
import io

# project imports
from pathPlanning.Constants import planSettingsFilename
from pathPlanning.PlannerUtils import runPlannerFromSettings

if __name__ == '__main__':
	# print help message if necessary
	if len(sys.argv) < 2 or any([s == '--help' or s == '-h' for s in sys.argv[1:]]):
		print('Usage: python profilePlanner.py /path/to/%s [/path/to/another/%s ...]'%tuple(2*[planSettingsFilename]))
		exit()

	# create profiler
	profiler = cProfile.Profile()
	profiler.enable()

	# for each provided settings file, run planner
	for s in sys.argv[1:]:
		try:
			runPlannerFromSettings(s)
		except Exception:
			print(traceback.format_exc())

	# sort results
	profiler.disable()
	stream = io.StringIO()
	stats = pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE)

	# print
	numFuncs = 20
	stats.print_stats(numFuncs)
	print(f'\nPrinting top {numFuncs} functions by cumulative time')
	print(stream.getvalue())
