from matplotlib import pyplot;
from pylab import genfromtxt;  
mat0 = genfromtxt("log.txt");
pyplot.plot(mat0[:,0], mat0[:,1], label = "altitude (m)");
pyplot.figure()
pyplot.plot(mat0[:,0], mat0[:,2], label = "rate (m/s)");
pyplot.legend();
pyplot.show();
