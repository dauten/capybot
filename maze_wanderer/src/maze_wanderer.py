import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan
#init
rospy.init_node("maze_wanderer")
#publish
frw = rospy.Publisher("/front_right_wheel_controller/command", Float64, queue_size=0)
flw = rospy.Publisher("/front_left_wheel_controller/command", Float64, queue_size=0)
brw = rospy.Publisher("/back_right_wheel_controller/command", Float64, queue_size=0)
blw = rospy.Publisher("/back_left_wheel_controller/command", Float64, queue_size=0)
def drive(left, right):
    flw.publish(left)
    blw.publish(left)

    frw.publish(right)
    brw.publish(right)

rospy.sleep(1)
drive(1,1)


def callback(data):
	print(data.ranges[len(data.ranges)/2])
	if data.ranges[len(data.ranges)/2] < .25:
		drive(2,-2)
		rospy.sleep(1)
		drive(2,2)

def listener():
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.spin()



if __name__ == '__main__':

	listener()


