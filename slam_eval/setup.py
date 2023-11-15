from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['slam_eval'],
    package_dir={'': 'src'},
    scripts=['scripts/slam_eval.py']
)

setup(**setup_args)