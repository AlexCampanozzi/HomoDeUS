from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['HomoDeUS_common_py'],
    package_dir={'': 'src'}
)
setup(**d)