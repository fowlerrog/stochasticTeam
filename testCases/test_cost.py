
from pathPlanning.OurPlanner import Cost

class TestCost:

	def test_add(self):
		c1 = Cost(1,2,3)
		c2 = Cost(4,5,6)
		c3 = c1 + c2
		assert(c3.value == [5,7,9])

	def test_compare(self):
		c1 = Cost(0,2)
		assert(c1 == c1)

		c2 = Cost(0,3)
		assert(c1 < c2)
		assert(c1 <= c2)
		assert(c2 > c1)
		assert(c2 >= c1)
		assert(c1 != c2)

		c3 = Cost(1,2)
		assert(c1 < c3)
		assert(c2 < c3)

		c4 = Cost(0,4)
		assert(c4 > c1)
		assert(c4 > c2)
		assert(c4 < c3)
