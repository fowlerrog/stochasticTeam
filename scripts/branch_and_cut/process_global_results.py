
import numpy as np

filename = './results_bonferroni.txt'
print('reading', filename)
with open(filename,'r') as f:
	lines = f.readlines()

print('Found', len(lines), 'lines')

data = {}
for line in lines:
	try:
		nums = [float(f) for f in line.split(',')]
		indVars = (nums[0]) # N
		if indVars in data:
			data[indVars].append([nums[4], nums[3]]) # compute time, mission time [objective time]
		else:
			data[indVars] = [[nums[4], nums[3]]]
	except:
		print('skipping line', line)

# for k, v in data.items():
# 	meanM, stdM = np.mean([e[0] for e in v]), np.std([e[0] for e in v])
# 	meanT, stdT = np.mean([e[1] for e in v]), np.std([e[1] for e in v])
# 	print(f'steps = {k[0]}, tmax = {k[1]} : mission time {meanM:.2f} +- {stdM:.2f} | comp time {meanT:.2f} +- {stdT:.2f}')

def sigfigs(i, n):
	return '{:g}'.format(float('{:.{p}g}'.format(i, p=n)))

# print latex table
nsigfig = 2
ns = list(data.keys())
ns = sorted(ns[i] for i in range(len(ns)) if ns[i] not in ns[:i])
for n in ns:
	strings = []
	for i in range(len(data[n][0])):
		v = [d[i] for d in data[n]]
		mean, std = np.nanmean(v), np.nanstd(v)
		strings.append(str(sigfigs(mean, nsigfig)) + ' \pm ' + str(sigfigs(std, nsigfig)))
	print(n, '$ & $'.join(strings))
