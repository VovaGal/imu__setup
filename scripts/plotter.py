import matplotlib.pyplot as plt
import sys

plotter_len = 200

data: list[list[float]] = list()

for line in sys.stdin:
    plt.clf()
    vals = line.split()

    if len(data) < len(vals):
        for i in range(len(vals)):
            data.append(list())

    for i in range(len(vals)):
        data[i].append(float(vals[i]))
        if len(data[i]) > plotter_len:
            data[i].pop(0)
        
        plt.plot(data[i])


    plt.pause(0.001)
