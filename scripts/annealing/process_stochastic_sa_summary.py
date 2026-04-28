
import numpy as np

filename = './stochastic_sa_summary.txt'
print('reading', filename)
with open(filename,'r') as f:
	lines = f.readlines()

print('Found', len(lines), 'lines')

data = {}
for line in lines:
	try:
		nums = [float(f) for f in line.split(', ')]
		indVars = (nums[1], nums[2]) # steps, tmax
		if indVars in data:
			data[indVars].append(nums[3:]) # mission time, compute time
		else:
			data[indVars] = [nums[3:]]
	except:
		print('skipping line', line)

for k, v in data.items():
	meanM, stdM = np.mean([e[0] for e in v]), np.std([e[0] for e in v])
	meanT, stdT = np.mean([e[1] for e in v]), np.std([e[1] for e in v])
	print(f'steps = {k[0]}, tmax = {k[1]} : mission time {meanM:.2f} +- {stdM:.2f} | comp time {meanT:.2f} +- {stdT:.2f}')

def sigfigs(i, n):
	return '{:g}'.format(float('{:.{p}g}'.format(i, p=n)))

# print latex table
tmaxs = [k[0] for k in data.keys()]
tmaxs = sorted(tmaxs[i] for i in range(len(tmaxs)) if tmaxs[i] not in tmaxs[:i])
steps = [k[1] for k in data.keys()]
steps = sorted(steps[i] for i in range(len(steps)) if steps[i] not in steps[:i])
for t in tmaxs:
	strings = []
	for s in steps:
		v = data[(t,s)]
		meanT, stdT = np.mean([e[1] for e in v]), np.std([e[1] for e in v])
		strings.append(str(sigfigs(meanT, 2)) + ' \pm ' + str(sigfigs(stdT, 2)))
	for s in steps:
		v = data[(t,s)]
		meanM, stdM = np.mean([e[0] for e in v]), np.std([e[0] for e in v])
		strings.append(str(sigfigs(meanM, 2)) + ' \pm ' + str(sigfigs(stdM, 2)))
	print('$ & $'.join(strings))
