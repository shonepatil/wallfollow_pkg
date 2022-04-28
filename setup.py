from setuptools import setup
import os
from glob import glob

package_name = 'wallfollow_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wallfollow = wallfollow_pkg.wallfollow:main',
            'my_action_server = wallfollow_pkg.my_action_server:main',
            'my_action_client = wallfollow_pkg.my_action_client:main'
        ],
    },
)
