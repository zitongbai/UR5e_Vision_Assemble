from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # install rviz folder
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaobaige',
    maintainer_email='zitongbai@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_listerner = vision.get_image_test:main',
            'img_process = vision.image_process:main'
        ],
    },
)
