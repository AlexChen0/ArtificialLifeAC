import matplotlib.pyplot as pt
import numpy as np

bLSV = np.load('backLegVal.npy')
fLSV = np.load('frontLegVal.npy')
pt.plot(bLSV, label="frontleg", linewidth=3)
pt.plot(fLSV, label="backleg")
pt.legend(loc="upper left")
pt.show()
