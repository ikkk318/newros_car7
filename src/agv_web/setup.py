"""agv_web —— AGV 网页控制台（后端网关 + 前端静态页）。"""

from setuptools import setup
from glob import glob
import os

package_name = 'agv_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('agv_web/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV',
    maintainer_email='agv@example.com',
    description='AGV 网页控制台：浏览器内完成导航/搬运/任务，并显示地图与实时位姿',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gateway=agv_web.web_gateway:main',
        ],
    },
)
