
# python imports
import os

# project imports
from pathPlanning.RunnerUtils import loadYamlContents, getIndependentValueCombos

class TestYamlLoader:

	def test_independentVariables(self):
		"""Tests whether indepndent variable combinations are produced correctly"""

		# load yaml and generate independent value combinations
		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		params = loadYamlContents(os.path.join(thisScriptFolder, 'independent_variables.yaml'))
		indVarCombos, fullDicts = getIndependentValueCombos(params)

		# manually generate independent value combinations
		manualCombos = []
		manualDicts = []
		for A in ['A1', 'A2']:
			for B in ['B1', 'B2']:
				for CA in ['CA1', 'CA2']:
					manualCombos.append({
						('A',) : A, ('B',) : B, ('C','A') : CA
					})
					manualDicts.append({
						'INDEPENDENT_VARIABLES' : ['A', 'B', 'C.A'],
						'A' : A, 'B' : B, 'D' : 'D',
						'C' : {'A' : CA, 'B' : 'CB'}
					})

		for indVarCombo in indVarCombos:
			# test if this combo is in manualCombos
			ind = manualCombos.index(indVarCombo)
			manualCombos.pop(ind)

			# test there is not another copy in manualCombos
			try:
				ind = manualCombos.index(indVarCombo)
			except ValueError:
				pass

		assert len(manualCombos) == 0

		for fullDict in fullDicts:
			# test if this dict is in manualDicts
			ind = manualDicts.index(fullDict)
			manualDicts.pop(ind)

			# test there is not another copy in manualCombos
			try:
				ind = manualDicts.index(fullDict)
			except ValueError:
				pass

		assert len(manualDicts) == 0

	def test_nesting(self):
		"""Tests whether yaml paths are properly followed"""

		thisScriptFolder = os.path.dirname(os.path.abspath(__file__))
		
		# child is in folder
		parent1 = loadYamlContents(os.path.join(thisScriptFolder, 'nested_parent1.yaml'))

		assert parent1 == {
			'PARENT_VALUE' : 'PARENT1',
			'CHILD' : {
				'VALUE' : 'CHILD1'
			}
		}

		# child is in the folder above
		#	this requires the path to be followed relative to parent, not to this script
		parent2 = loadYamlContents(os.path.join(thisScriptFolder, 'folder', 'nested_parent2.yaml'))

		assert parent2 == {
			'PARENT_VALUE' : 'PARENT2',
			'CHILD' : {
				'VALUE' : 'CHILD2'
			}
		}
