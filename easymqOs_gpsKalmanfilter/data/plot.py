import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
# plot the gps data
def loadData(flieName):
	inFile = open(flieName, 'r')

	x = []
	y = []
	cnt = 1;
	for line in inFile:
		cnt = cnt+1
		pair = line.split()
		x.append(pair[0])
		y.append(pair[1])
		if cnt==100:
			break

	x = map(float,x)
	y = map(float,y)
	return	(x, y)

(x,y) = loadData('raw_data.txt')
(u,v) = loadData('filtered_data.txt')




plt.figure(figsize=(8, 4))
plt.scatter(np.linspace(1,len(x),len(x)),[(a-118) for a in x],label="$lon data$", color="blue");
plt.scatter(np.linspace(1,len(u),len(u)),[(a-118.09) for a in x], label="$lonf data$", color="red")
plt.legend()
plt.show()

# plt.plot(u,v, label="$filtered data$", color="red", linewidth=2)
# plt.scatter(x,y,label="$raw data$", color="blue");

# plt.xlabel("lon(deg)")
# plt.ylabel("lat(deg)")
# plt.title("lat-lon plot")

# plt.legend()
# plt.show()


