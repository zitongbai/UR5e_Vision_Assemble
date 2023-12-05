from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'
submodule_model_name = 'vision/yolov5/models'
submodule_utils_name = 'vision/yolov5/utils'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, submodule_model_name, submodule_utils_name],
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
            'obj_detect = vision.obj_detect:main',
            'test_sync = vision.test_sync:main',
            'test_rs = vision.test_rs:main',
            'obj_seg = vision.obj_seg:main',
            'record = vision.record:main',
            'det_tf = vision.det_tf:main',
        ],
    },
)
