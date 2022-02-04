import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan

import numpy as np

def callback(data):
	cp_publisher.publish(np.min(data.ranges))
	fp_publisher.publish(np.max(data.ranges))

def listener():

	# rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('scan', LaserScan, callback)

	rospy.spin()


if __name__ == "__main__":
	cp_publisher = rospy.Publisher('closest_point', Float64, queue_size=10)
	fp_publisher = rospy.Publisher('farthest_point', Float64, queue_size=10)
	rospy.init_node('listak', anonymous=True)
	listener()