# python imports
import multiprocessing
import traceback
import random
from itertools import repeat
from functools import lru_cache
import time

# function that relies on dict
def hashLookup(v, d):
	found = True
	value = d.get(v, None)
	if value is None:
		found = False
		value = v + 1
		d[v] = value
		time.sleep(0.1)
	return [value, found]

# function that relies on lru_cache
@lru_cache(maxsize=None)
def hashLookupCached(v):
	time.sleep(0.1)
	return v + 1

# class that builds its own cache
class cacheFunction():
	fCache: dict

	def __init__(self):
		self.fCache = {}

	def f(self, v):
		time.sleep(0.1)
		return v + 1

	def fCached(self, v):
		found = True
		value = self.fCache.get(v, None)
		if value is None:
			found = False
			value = self.f(v)
			self.fCache[v] = value
		return [value, found]

	def fCachedExplicit(self, v, fCache):
		found = True
		value = fCache.get(v, None)
		if value is None:
			found = False
			value = self.f(v)
			fCache[v] = value
		return [value, found]

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
		print('Pool 1 processes =', pool1._processes)
		fCache = {}
		t1Start = time.perf_counter()
		results1 = pool1.starmap(hashLookup, zip(vVec, repeat(fCache)))
		t1End = time.perf_counter()
		pool1.close()

		# try to pass a managed dict
		pool2 = multiprocessing.Pool()
		print('Pool 2 processes =', pool2._processes)
		manager = multiprocessing.Manager()
		d = manager.dict()
		t2Start = time.perf_counter()
		results2 = pool2.starmap(hashLookup, zip(vVec, repeat(d)))
		t2End = time.perf_counter()
		pool2.close()

		# try to pass an lru_cache function directly
		pool3 = multiprocessing.Pool()
		print('Pool 3 processes =', pool3._processes)
		t3Start = time.perf_counter()
		results3 = pool3.map(hashLookupCached, vVec)
		t3End = time.perf_counter()
		pool3.close()

		# try to pass a class with dict cache directly
		pool4 = multiprocessing.Pool()
		print('Pool 4 processes =', pool4._processes)
		cf4 = cacheFunction()
		t4Start = time.perf_counter()
		results4 = pool4.map(cf4.fCached, vVec)
		t4End = time.perf_counter()
		pool4.close()

		# try to pass a class with managed cache
		pool5 = multiprocessing.Pool()
		print('Pool 5 processes =', pool5._processes)
		cf5 = cacheFunction()
		cf5.fCache = manager.dict(cf5.fCache)
		t5Start = time.perf_counter()
		results5 = pool5.map(cf5.fCached, vVec)
		t5End = time.perf_counter()
		pool5.close()

		# try to pass a class with an external managed cache
		pool6 = multiprocessing.Pool()
		print('Pool 6 processes =', pool6._processes)
		cf6 = cacheFunction()
		fCache6 = manager.dict()
		t6Start = time.perf_counter()
		results6 = pool6.starmap(cf6.fCachedExplicit, zip(vVec, repeat(fCache6)))
		t6End = time.perf_counter()
		pool6.close()

		# print results
		print('Direct dict:')
		print('\tdict =', fCache)
		print('\ttime =', t1End - t1Start)
		print('\thits =', len([a for a in results1 if a[1]]))

		print('Managed dict:')
		print('\tdict =', d)
		print('\ttime =', t2End - t2Start)
		print('\thits =', len([a for a in results2 if a[1]]))

		print('Lru_cache:')
		print('\tcachesize =', hashLookupCached.cache_info().currsize)
		print('\ttime =', t3End - t3Start)
		print('\thits =', hashLookupCached.cache_info().hits)

		print('Direct class cache:')
		print('\tdict =', cf4.fCache)
		print('\ttime =', t4End - t4Start)
		print('\thits =', len([a for a in results4 if a[1]]))

		print('Managed class cache:')
		print('\tdict =', cf5.fCache)
		print('\ttime =', t5End - t5Start)
		print('\thits =', len([a for a in results5 if a[1]]))

		print('Managed external class cache:')
		print('\tdict =', fCache6)
		print('\ttime =', t6End - t6Start)
		print('\thits =', len([a for a in results6 if a[1]]))

	except Exception:
		print(traceback.format_exc())
