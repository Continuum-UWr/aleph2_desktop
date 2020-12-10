# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'aleph2_gui.aleph2_drivetrain_controller',
        'aleph2_gui.aleph2_manip_controller',
        'aleph2_gui.aleph2_power_controller',
        'aleph2_gui.joystick_selector',
        'aleph2_gui.resources',
        'aleph2_gui.rubi_controller',
    ],
    package_dir={'': 'src'}
)

setup(**setup_args)
