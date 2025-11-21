from glob import glob
import os
from setuptools import find_packages, setup

PACKAGE_NAME = 'udp_example'
SHARE_DIR = os.path.join('share', PACKAGE_NAME)

setup(
    name = PACKAGE_NAME,
    version = '0.1.0',
    packages = find_packages(exclude = ['test']),
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        (SHARE_DIR, ['package.xml']),
        (os.path.join(SHARE_DIR, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    maintainer = 'Marco Walther',
    maintainer_email = 'marco@sonic.net',
    description = 'TODO: Package description',
    license = 'Apache-2.0',
    extras_require = {
        'test': [
            'pytest',
        ],
    },
    entry_points = {
        'console_scripts': [
            'udp_test_server = udp_example.test_server:main',
            'udp_node = udp_example.udp_node:main',
        ],
    },
)
