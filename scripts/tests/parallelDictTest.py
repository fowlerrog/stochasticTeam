# python imports
import multiprocessing
import traceback
import random
from itertools import repeat
# from functools import lru_cache
import time

# function that relies on dict
def hashLookup(v, d):
	found = True
	if v not in d:
		found = False
		d[v] = v + 1
		time.sleep(0.1)
	return [d[v], found]

# # function that relies on lru_cache
# @lru_cache(maxsize=None)
# def hashLookupCached(v):
# 	return v + 1

if __name__ == '__main__':
	try:

		seed = 0
		n = 50 # which numbers we pass
		m = 10 # how many times we pass each number

		# generate argument vector
		vVec = []
		for i in range(n):
			vVec += [i] * m
		random.shuffle(vVec) # leaving in order will make things incorrectly faster because [..., i, i, i, i, ...] are likely to be given to the same process because pool.starmap breaks vVec into chunks, and each process builds its own version of the direct dict

		# try to pass a dict directly
		pool1 = multiprocessing.Pool()
		fCache = {}
		t1Start = time.perf_counter()
		results1 = pool1.starmap(hashLookup, zip(vVec, repeat(fCache)))
		t1End = time.perf_counter()

		# try to pass a managed dict
		pool2 = multiprocessing.Pool()
		manager = multiprocessing.Manager()
		d = manager.dict()
		t2Start = time.perf_counter()
		results2 = pool2.starmap(hashLookup, zip(vVec, repeat(d)))
		t2End = time.perf_counter()

		# print results
		print('Direct dict:')
		print('\tdict =', fCache)
		print('\ttime =', t1End - t1Start)
		print('\thits =', len([a for a in results1 if a[1]]))
		print('\tprocesses =', pool1._processes)

		print('Managed dict:')
		print('\tdict =', d)
		print('\ttime =', t2End - t2Start)
		print('\thits =', len([a for a in results2 if a[1]]))
		print('\tprocesses =', pool2._processes)

	except Exception:
		print(traceback.format_exc())
