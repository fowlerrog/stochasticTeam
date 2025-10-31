
import sys

from .OurPlanner import *

def plannerFromParams(params):
	plannerClass = getattr(sys.modules[__name__], params['PLANNER_TYPE'])
	return plannerClass(params)
