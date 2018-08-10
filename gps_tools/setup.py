## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  packages=['gps_tools'],
  package_dir={'': 'src'},
  requires=[
    'rospy',
    'std_msgs',
    'std_srvs',
    'geometry_msgs',
    'rospy',
    'tf2_ros',
    'tf'
  ]
)

setup(**setup_args)