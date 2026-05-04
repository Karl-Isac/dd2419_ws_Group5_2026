import numpy as np
import matplotlib.pyplot as plt


pts = np.loadtxt("/Users/ki/Desktop/Skola/Robot/dd2419_ws_Group5_2026/map_points.csv", delimiter=",", skiprows=1)
plt.scatter(pts[:,0], pts[:,1], s=1)
plt.axis("equal")
plt.show()