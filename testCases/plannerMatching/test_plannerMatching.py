# python imports
import os

# project imports
from pathPlanning.PlannerUtils import runPlannerFromSettings
from pathPlanning.RunnerUtils import loadPlanResultsFromFolder
from pathPlanning.Constants import planSettingsFilename

class TestPlannerMatching:

	def test_plannerMatching(self):
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))

		# Run the deterministic and stochastic versions of the same case
		runPlannerFromSettings(os.path.join(thisScriptFolder, planSettingsFilename))

		# Load results
		detPlanResults = loadPlanResultsFromFolder(os.path.join(thisScriptFolder, 'results_PLANNER_TYPE_OurPlannerDeterministic'))
		stochPlanResults = loadPlanResultsFromFolder(os.path.join(thisScriptFolder, 'results_PLANNER_TYPE_OurPlannerStochastic'))

		# Under the assumptions:
		#	Both planners have a correct model of the environment
		#	Deterministic delta time = 0
		#	Stochastic risk = 0.5
		# both planners should produce the same plan
		assert(detPlanResults['uav_points'] == stochPlanResults['uav_points'])
		assert(detPlanResults['uav_cycles'] == stochPlanResults['uav_cycles'])
		assert(detPlanResults['ugv_mapping_to_points'] == stochPlanResults['ugv_mapping_to_points'])
		assert(detPlanResults['ugv_path'] == stochPlanResults['ugv_path'])
