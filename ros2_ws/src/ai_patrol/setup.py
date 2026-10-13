from glob import glob

from setuptools import setup

package_name = 'ai_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai-student',
    maintainer_email='noreply@example.com',
    description='AI 捜索ドローン: PX4 + Gazebo + YOLO による要救助者の自律捜索',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector = ai_patrol.detector_node:main',
            'mission = ai_patrol.mission_node:main',
            'dashboard = ai_patrol.dashboard_node:main',
        ],
    },
)
