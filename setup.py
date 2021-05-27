from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import os

# Export source python files
setup_args = generate_distutils_setup(
    packages=['astrobee_ros_demo'],
    package_dir={'': 'src'},
)

thelibFolder = os.path.dirname(os.path.realpath(__file__))
requirementPath = thelibFolder + '/requirements.txt'
install_requires = []
if os.path.isfile(requirementPath):
    with open(requirementPath) as f:
        install_requires = f.read().splitlines()
setup_args.update({'install_requires': install_requires})

setup(**setup_args)
