from matplotlib import pyplot as plt
import matplotlib.pylab as pylab
from math import pi


pylab.rcParams['figure.figsize'] = 10, 10

def plot_zy_flight_path(z_path, y_path, history):
	plt.plot( y_path, z_path,linestyle='-',marker='.',color='red')
	plt.plot(history[:,1],history[:,0],linestyle='-',color='blue')
	plt.title('Flight trajectory').set_fontsize(20)
	plt.xlabel('$y$ [$m$]').set_fontsize(20)
	plt.ylabel('$z$ [$m$]').set_fontsize(20)
	plt.xticks(fontsize = 14)
	plt.yticks(fontsize = 14)
	plt.legend(['Planned path','Executed path'],fontsize = 14)
	plt.gca().invert_yaxis()
	plt.axis('equal')
	plt.show()

def plot_phi(history, t):
	plt.plot(t, history[:,2] * 180 / pi)
	plt.show()


def compare_flight_paths(z_path, y_path, history_1, history_2, title_1, title_2):
	plt.plot( y_path, z_path,linestyle='-',marker='.',color='red')
	plt.plot(history_1[:,1],history_1[:,0],linestyle='-',color='blue',alpha=0.5)
	plt.plot(history_2[:,1],history_2[:,0],linestyle='-',color='green',alpha=0.5)
	plt.title('Flight trajectory').set_fontsize(20)
	plt.xlabel('$y$ [$m$]').set_fontsize(20)
	plt.ylabel('$z$ [$m$]').set_fontsize(20)
	plt.xticks(fontsize = 14)
	plt.yticks(fontsize = 14)
	plt.gca().invert_yaxis()
	plt.legend(['Planned path',title_1, title_2 ],fontsize = 14)
	plt.axis('equal')
	plt.show()
