import rospy
import tf
import numpy as np
from matplotlib import pyplot as plt

class VSCaleCalibrator(object):
	def __init__(self):
		rospy.init_node('vscale_calibrator')
		self._tfl = tf.TransformListener()
		self._data = [] # (timestamp, distance)

		self._t0 = rospy.Time.now()

	def step(self):
		try:
			t, q = self._tfl.lookupTransform('map', 'camera_link', rospy.Time(0))
			x, y = t[0], t[1]
			d    = np.sqrt(x**2 + y**2)
			time = rospy.Time.now()
			print (time - self._t0).to_sec()
			self._data.append( ((time - self._t0).to_sec(), d) )
		except Exception:
			#print 'life is terrible'
			pass

	def show(self):
		data = np.asarray(self._data, dtype=np.float32)
		#print data[:,0]

		plt.plot(data[:,0] - data[0,0], data[:,1])
		plt.show()

		#print self._data
		#print 'shutdown'

	def run(self):
		rate = rospy.Rate(100)
		rospy.on_shutdown(self.show)

		while self._t0.to_sec() == 0:
			self._t0 = rospy.Time.now()

		while not rospy.is_shutdown():
			self.step() # << where stuff happens
			rate.sleep()

def main():
	app = VSCaleCalibrator()
	app.run()

if __name__ == "__main__":
	main()
