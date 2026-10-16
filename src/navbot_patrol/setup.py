from setuptools import setup

package_name = 'navbot_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/launch', ['launch/patrol.launch.py']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=False,
    author='hejiurui',
    author_email='hejiurui@users.noreply.github.com',
    maintainer='hejiurui',
    maintainer_email='hejiurui@users.noreply.github.com',
    description='navbot 多航点巡检任务节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'patrol = navbot_patrol.patrol_node:main',
        ],
    },
)
