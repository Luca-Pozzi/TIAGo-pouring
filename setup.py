from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['tiagp_pouring'],
    package_dir={'': 'scripts'}
)
setup(**d)