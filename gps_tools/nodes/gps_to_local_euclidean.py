#!/usr/bin/env python
import rospy, math
import tf, tf2_ros
import nvector as nv

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from geometry_msgs.msg import (
  TransformStamped,
  Transform,
  Quaternion,
  Vector3,
  Point,
  PointStamped
)

WGS84 = nv.FrameE(name='WGS84')

class StampedGeoPoint(object):

  def __init__(self, geopoint, stamp):
    self.geopoint = geopoint

    """ TODO(asiron) remove when OSDK is fixed! """
    if stamp == rospy.Time(0):
      self.stamp = rospy.get_rostime()
    else:
      self.stamp = stamp

  def __format__(self, fmt_spec):
    gp = self.to_vec()
    return ("<StampedGeoPoint \n\tstamp: {} \n\tlat: {}"
           "\n\tlon: {}\n\talt: {}\n> ".format(self.stamp, *gp))

  def __eq__(self, other):
    if isinstance(other, StampedGeoPoint):
      return (self.geopoint.latitude == other.geopoint.latitude) \
        and (self.geopoint.longitude == other.geopoint.longitude) \
        and (self.geopoint.z == other.geopoint.z)
    return False

  def __ne__(self, other):
    if isinstance(other, StampedGeoPoint):
      return not (self == other)
    return False

  def to_msg(self):
    h = Header()
    h.stamp = self.stamp
    h.frame_id = earth_frame_id

    msg = NavSatFix()
    msg.header = h
    msg.latitude = math.degrees(self.geopoint.latitude)
    msg.longitude = math.degrees(self.geopoint.longitude)
    msg.altitude = self.geopoint.z
    return msg

  def to_vec(self, degrees=True):
    vec = [self.geopoint.latitude, self.geopoint.longitude, self.geopoint.z]
    return map(math.degrees, vec[:2]) + [vec[2]] if degrees else vec

reference_stamped_geopoint = None
last_gps_message = None

earth_frame_id = None
gps_no_orientation_frame_id = None

def gps_reference_callback(msg):

  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude

  reference_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  global reference_stamped_geopoint
  reference_stamped_geopoint = StampedGeoPoint(reference_geopoint, msg.header.stamp)

  rospy.loginfo(('{}: GPS Reference was set to: '
                 '{}').format(node_name, reference_stamped_geopoint))

def gps_callback(msg):
  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude

  if 0.0 in [lat, lon, alt]:
    rospy.logwarn_throttle(2, ('{}: Received a msg with lat, lon, alt'
                              ': 0, 0, 0'.format(node_name)))
    return

  current_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)
  current_stamped_geopoint = StampedGeoPoint(current_geopoint, msg.header.stamp)

  global last_gps_message
  if last_gps_message is None:
    last_gps_message = current_stamped_geopoint
  else:
    if last_gps_message != current_stamped_geopoint:
      last_gps_message = current_stamped_geopoint
    else:
      return

  if reference_stamped_geopoint is None:
    rospy.logwarn_throttle(2, '{}: GPS Reference is not set!'.format(node_name))
    return

  '''get transformation from intial to current in Earth coordinate frame'''
  transform_in_E = nv.diff_positions(
    reference_stamped_geopoint.geopoint,
    current_stamped_geopoint.geopoint)

  '''get transformation in NED frame'''
  reference_frame = nv.FrameN(reference_stamped_geopoint.geopoint)
  transform_in_NED = transform_in_E.change_frame(reference_frame)
  transform_in_NED = transform_in_NED.pvector.ravel()

  '''get transformation in ENU frame (ROS compatible)'''
  transform_in_ENU = [
    transform_in_NED[1],
    transform_in_NED[0],
    transform_in_NED[2]
  ]

  h = Header()
  h.stamp = current_stamped_geopoint.stamp
  h.frame_id = earth_frame_id

  p_stamped = PointStamped()
  p_stamped.header = h
  p_stamped.point.x = transform_in_ENU[0]
  p_stamped.point.y = transform_in_ENU[1]
  p_stamped.point.z = transform_in_ENU[2]
  local_euclidean_position_pub.publish(p_stamped)

  if(publish_tf):
    t_stamped = TransformStamped()
    t_stamped.header = h
    t_stamped.child_frame_id = gps_no_orientation_frame_id
    t_stamped.transform = Transform()
    t_stamped.transform.translation = Vector3(*transform_in_ENU)
    t_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
    tf_broadcaster.sendTransform(t_stamped)


rospy.init_node('gps_to_local_euclidean')

node_name = rospy.get_name()

publish_tf = rospy.get_param('~publish_tf')

earth_frame_id = rospy.get_param('~earth_frame_id')
gps_no_orientation_frame_id = rospy.get_param('~gps_no_orientation_frame_id')

rospy.loginfo('{}: Assuming map frame id to be: {}'.format(node_name, earth_frame_id))
rospy.loginfo(('{}: Assuming baselink translation (no orientation) frame id'
               'to be: {}'.format(node_name, gps_no_orientation_frame_id)))

if(publish_tf):
  tf_broadcaster = tf2_ros.TransformBroadcaster()

rospy.Subscriber('gps_reference', NavSatFix, gps_reference_callback)

rospy.Subscriber('gps_position', NavSatFix, gps_callback)
local_euclidean_position_pub  = rospy.Publisher('local_euclidean_position',
                                              PointStamped,
                                              queue_size = 1)

r = rospy.Rate(0.1)
while not rospy.is_shutdown():
  if reference_stamped_geopoint is None:
    rospy.logwarn(('{}: No GPS message received and no reference '
                   'was set!'.format(node_name)))
  try:
    r.sleep()
  except rospy.exceptions.ROSTimeMovedBackwardsException:
    pass
  except rospy.exceptions.ROSInterruptException:
    break
