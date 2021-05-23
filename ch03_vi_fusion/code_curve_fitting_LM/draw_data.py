#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

f = open("data_mu.txt")

x = []
y = []
for line in f:
    data = line.split()
    x.append( float(data[0] ) )
    y.append( float(data[1] ) )

plt.plot(x,y,marker='o')

plt.xlim(xmin = -1)
plt.ylim(ymin = -100)
plt.xticks(np.arange(min(x), max(x)+1, 1.0))
# for xy in zip(x, y):
#     plt.annotate("(%s,%s)" % xy, xy=xy, xytext=(-20, 10), textcoords='offset points')
plt.xlabel("iter")
plt.ylabel("lambda")
plt.title('LM damping factor plot')

plt.show()
