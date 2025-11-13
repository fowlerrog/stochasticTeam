
# python imports
import pytest
import random

# project imports
from pathPlanning.OurPlanner import Cost

class TestCost:

	@pytest.mark.parametrize("randomSeed", range(5))
	@pytest.mark.parametrize("length", [3])
	def test_add(self, randomSeed, length):
		"""Tests whether costs add elementwise"""
		random.seed(randomSeed)
		a = [random.randint(-100, 100) for _ in range(length)]
		b = [random.randint(-100, 100) for _ in range(length)]
		c1 = Cost(*a)
		c2 = Cost(*b)
		c3 = c1 + c2
		assert c3.value == [a[i] + b[i] for i in range(length)]

	def test_compare(self):
		"""Tests whether costs compare like tuples"""
		c1 = Cost(0,2)
		assert c1 == c1

		c2 = Cost(0,3)
		assert c1 < c2
		assert c1 <= c2
		assert c2 > c1
		assert c2 >= c1
		assert c1 != c2

		c3 = Cost(1,2)
		assert c1 < c3
		assert c2 < c3

		c4 = Cost(0,4)
		assert c4 > c1
		assert c4 > c2
		assert c4 < c3
