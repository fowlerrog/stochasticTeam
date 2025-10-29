
from pathPlanning.OurPlanner import Cost

class TestCost:
	def test_add(self):
		c1 = Cost(1,2,3)
		c2 = Cost(4,5,6)
		c3 = c1 + c2
		assert(c3.values == [5,7,9])
