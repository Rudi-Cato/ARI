import rospy

from pal_web_msgs.msg import WebGoTo



if __name__ == "__main__":
  rospy.init_node("publisher")
  pub = rospy.Publisher("/web/go_to", WebGoTo, queue_size=10)
  rate = rospy.Rate(10) # 10Hz
  msg = WebGoTo()
  while not rospy.is_shutdown():


     # check http://docs.ros.org/en/api/pal_web_msgs/html/msg/WebGoTo.html

     # for the msg structure

     # msg.data = ...

     pub.publish(msg)


     rate.sleep()