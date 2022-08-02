# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['boat_control'],
    package_dir={'': 'src'},
    requires=['mavros', 'mavros_msgs', 'rospy']
)

setup(**setup_args)