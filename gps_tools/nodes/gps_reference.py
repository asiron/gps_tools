#!/usr/bin/env python
import rospy, math, collections, time, os, yaml
import nvector as nv
import numpy as np

from functools import wraps, partial

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse

from gps_tools_srvs.srv import LoadGPSReference, LoadGPSReferenceRequest, LoadGPSReferenceResponse

WGS84 = nv.FrameE(name='WGS84')

""" TODO(asiron) convert into parameter """
HISTORY_LENGTH = 30
EXPECTED_FREQ = 5.0
LAST_MEASUREMENT_DELAY_THRESHOLD = 0.5
MEASUREMENT_UPDATE_FREQUENCY_TOLERANCE = 1.0

LAT_VAR_TH = 1e-10
LON_VAR_TH = 1e-10
ALT_VAR_TH = 1e-3

def logged_service_callback(f, throttled = True):

  if throttled:
    loginfo = partial(rospy.loginfo_throttle, 1)
    logwarn = partial(rospy.logwarn_throttle, 1)
  else:
    loginfo = rospy.loginfo
    logwarn = rospy.logwarn

  @wraps(f)
  def wrapper(*args, **kwds):
    ret_val = f(*args, **kwds)
    message = "{}: {}".format(node_name, ret_val.message)
    loginfo(message) if ret_val.success else logwarn(message)
    return ret_val

  return wrapper

def msg_dict_to_NavSatFix(msg_dict):
  msg = NavSatFix()
  msg.header = Header()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = msg_dict['header']['frame_id']
  msg.latitude = msg_dict['latitude']
  msg.longitude = msg_dict['longitude']
  msg.altitude = msg_dict['altitude']
  msg.status.status = msg_dict['status']['status']
  msg.status.service = msg_dict['status']['service']
  msg.position_covariance = msg_dict['position_covariance']
  msg.position_covariance_type = msg_dict['position_covariance_type']
  return msg

class StampedGeoPoint(object):

  def __init__(self, geopoint, stamp):
    self.geopoint = geopoint

    if stamp == rospy.Time(0):
      rospy.logwarn('Received msg contained empty stamp!')
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

  @staticmethod
  def from_msg(msg):
    geopoint = WGS84.GeoPoint(
      latitude=msg.latitude,
      longitude=msg.longitude,
      z=msg.altitude,
      degrees=True)
    return StampedGeoPoint(geopoint, msg.header.stamp)

  def to_vec(self, degrees=True):
    vec = [self.geopoint.latitude, self.geopoint.longitude, self.geopoint.z]
    return map(math.degrees, vec[:2]) + [vec[2]] if degrees else vec

  def save(self, output_directory):
    filename = time.strftime('gps_reference_%Y%m%d-%H%M%S.yml')
    filepath = os.path.join(output_directory, filename)
    rospy.loginfo('{}: Saving to a file: {}'.format(node_name, filepath))
    with open(filepath, 'wt') as f:
      f.write(str(self.to_msg()))

measurement_history = collections.deque(maxlen=HISTORY_LENGTH)

reference_stamped_geopoint = None

def gps_callback(msg):
  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude

  if 0.0 in [lat, lon, alt]:
    rospy.logwarn_throttle(2, ('{}: Received a GPS msg with lat, lon, alt'
                              ': 0, 0, 0'.format(node_name)))
    return

  current_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  current_stamped_geopoint = StampedGeoPoint(current_geopoint, msg.header.stamp)

  if len(measurement_history) == 0:
    measurement_history.append(current_stamped_geopoint)
  else:
    last_measurement = measurement_history[-1]
    if last_measurement != current_stamped_geopoint:
      measurement_history.append(current_stamped_geopoint)


  if reference_stamped_geopoint is None:
    if autoset_geo_reference_file:
      rospy.logwarn(("{}: Auto-setting GPS reference from "
                     "loaded file {}!").format(node_name, autoset_geo_reference_file))
      load_gps_reference(LoadGPSReferenceRequest(autoset_geo_reference_file))

    elif autoset_geo_reference:
      rospy.logwarn_throttle(1, "{}: Auto-setting GPS reference!".format(node_name))
      set_geo_reference()

@logged_service_callback
def load_gps_reference(request=None):

  filename = request.filename
  if not (filename.endswith('.yml') or filename.endswith('.yaml')):
    filename = filename + '.yml'

  filepath = os.path.join(gps_reference_output_directory, filename)
  with open(filepath, 'rt') as f:
    try:
      msg_dict = yaml.load(f)
    except yaml.YAMLError as exc:
      return LoadGPSReferenceResponse(False, 'YAML parse exception!')

  try:
    msg = msg_dict_to_NavSatFix(msg_dict)
    global reference_stamped_geopoint
    reference_stamped_geopoint = StampedGeoPoint.from_msg(msg)
  except:
    return LoadGPSReferenceResponse(False, ('Exception during msg creation ',
                                            'probably a malformed YAML file!'))
  reference_geopoint_pub.publish(msg)
  return LoadGPSReferenceResponse(True, 'GPS reference loaded correctly')

@logged_service_callback
def set_geo_reference(request=None):
  if len(measurement_history) == 0:
    return TriggerResponse(False, 'Did not receive any GPS message so far!')
  elif len(measurement_history) != HISTORY_LENGTH:
    return TriggerResponse(False, ('Did not receive all {} needed '
                                   'GPS messages!'.format(HISTORY_LENGTH)))

  last_measurement = measurement_history[-1]
  duration = (rospy.Time.now() - last_measurement.stamp).to_sec()
  if duration > LAST_MEASUREMENT_DELAY_THRESHOLD:
    string_response = 'Last message was received {:.3f}s ago'.format(duration)
    return TriggerResponse(False, string_response)

  oldest_measurement = measurement_history[0]
  total_duration = (last_measurement.stamp - oldest_measurement.stamp).to_sec()
  actual_freq = HISTORY_LENGTH / total_duration
  frequency_err = abs(EXPECTED_FREQ - actual_freq)
  if frequency_err > MEASUREMENT_UPDATE_FREQUENCY_TOLERANCE:
    string_response = ('Last {N} messages were received with a'
                       'freqency of {actual:.3f} instead of '
                       '{expected:.3f}'.format(N=HISTORY_LENGTH,
                                              actual=actual_freq,
                                              expected=EXPECTED_FREQ))
    return TriggerResponse(False, string_response)

  measurements = np.array(map(lambda m: m.to_vec(), measurement_history))
  measurement_covariance = np.cov(measurements, rowvar=False).diagonal()
  measurement_mean = np.mean(measurements, axis=0)

  lat_var, lon_var, alt_var = measurement_covariance
  if (lat_var > LAT_VAR_TH) or (lon_var > LON_VAR_TH) or (alt_var > ALT_VAR_TH):
    string_response = ('Calculated diagnoal covariance matrix '
                       '(lat, lon, alt) was {} and the limits '
                       'are {} {} {}'.format(measurement_covariance,
                                             LAT_VAR_TH,
                                             LON_VAR_TH,
                                             ALT_VAR_TH))
    return TriggerResponse(False, string_response)

  lat, lon, alt = measurement_mean
  reference_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  global reference_stamped_geopoint
  reference_stamped_geopoint = StampedGeoPoint(reference_geopoint,
                                             last_measurement.stamp)
  reference_geopoint_pub.publish(reference_stamped_geopoint.to_msg())

  reference_stamped_geopoint.save(gps_reference_output_directory)

  string_response = 'GPS Reference was set to: {}'.format(reference_stamped_geopoint)
  return TriggerResponse(True, string_response)

rospy.init_node('gps_reference')

node_name = rospy.get_name()

earth_frame_id = rospy.get_param('~earth_frame_id')
autoset_geo_reference = rospy.get_param('~autoset_geo_reference')
gps_reference_output_directory = rospy.get_param('~gps_reference_output_directory')
autoset_geo_reference_file = rospy.get_param('~autoset_geo_reference_file')

if not os.path.exists(gps_reference_output_directory):
  os.makedirs(gps_reference_output_directory)

rospy.Subscriber('gps_position', NavSatFix, gps_callback)

rospy.Service('set_geo_reference', Trigger, set_geo_reference)
rospy.Service('load_gps_reference', LoadGPSReference, load_gps_reference)

reference_geopoint_pub = rospy.Publisher('gps_reference',
                                         NavSatFix,
                                         queue_size = 1,
                                         latch = True)

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
