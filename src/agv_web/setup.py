"""agv_web —— AGV 网页控制台（后端网关 + 前端静态页）。"""

from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'agv_web'

setup(
    name=package_name,
    version='0.1.0',
    # ★ 必须 find_packages()：网关实现拆到了 agv_web/gateway/ 子包里，
    #   写死 packages=[package_name] 只会装顶层包，运行时报
    #   ModuleNotFoundError: No module named 'agv_web.gateway'。
    packages=find_packages(exclude=['test', 'test.*']),
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
