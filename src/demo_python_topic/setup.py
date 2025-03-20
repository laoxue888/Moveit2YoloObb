from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'demo_python_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 加入以下这一行识别launch文件。
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='xsf@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'novel_pub_node = demo_python_topic.novel_pub_node:main',
            'novel_sub_node = demo_python_topic.novel_sub_node:main',
            'handle_ctrl_node = demo_python_topic.handle_ctrl_node:main',
            'keyboard_ctrl_topic = demo_python_topic.keyboard_ctrl_topic:main',
            'read_joint_states = demo_python_topic.read_joint_states:main'
        ],
    },
)
