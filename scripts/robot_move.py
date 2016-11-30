 #!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(hello_str)
        rate.sleep()
	rospy.Subscriber("chatter", String, callback)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
