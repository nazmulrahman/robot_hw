import rospy
import math

from geometry_msgs.msg import Point
from ublox_msgs.msg import NavSTATUS, NavPVT
from sensor_msgs.msg import NavSatFix

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

status_rear=0
status_front=0


lat_org = 90.35303250
long_org = 23.76030833


def callback_rear_status(msg):
    status_rear = msg.fixStat
    print("status_rear")
    print(status_rear)

def callback_front_status(msg):
    status_front = msg.fixStat 
    print("status_front")
    print(status_front)


def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  # rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
  azimuth = g['azi1']
  # rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = -math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  # rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
  # print hypotenuse
  # print x  ,  y
  # return x, y  flipped so that it corresponds to ros coordinate system  
  return x,y


def callback_rear_NavPVT(msg):
    rtk_msg   = Point()

    rtk_msg.x,rtk_msg.y = calc_goal(lat_org,long_org, msg.lat*1e-7,msg.lon*1e-7)
    rtk_msg.z = 0

    pub_rear.publish(rtk_msg)




def callback_front_NavPVT(msg):
    rtk_msg   = Point()

    rtk_msg.x,rtk_msg.y = calc_goal(lat_org,long_org, msg.lat*1e-7,msg.lon*1e-7)
    rtk_msg.z = 0

    pub_front.publish(rtk_msg)




if __name__ == '__main__':

    rospy.init_node('rtk_fix_publisher', anonymous=True)

    pub_rear = rospy.Publisher('/rtk_rear_fix', Point ,queue_size=10)
    pub_front = rospy.Publisher('/rtk_front_fix', Point ,queue_size=10)

    while not rospy.is_shutdown():
        rospy.Subscriber("/ublox_rear/navStatus", NavSTATUS, callback_rear_status, queue_size=1)
        rospy.Subscriber("/ublox_front/navStatus", NavSTATUS, callback_front_status, queue_size=1)
        rospy.Subscriber("/ublox_rear/navpvt", NavPVT, callback_rear_NavPVT, queue_size=1)
        rospy.Subscriber("/ublox_front/navpvt", NavPVT, callback_front_NavPVT, queue_size=1)
        
        rospy.spin()
