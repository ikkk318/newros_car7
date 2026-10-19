from glob import glob                              # use 通配符匹配 launch 文件
from setuptools import find_packages, setup        # use 打包工具与包发现

package_name = 'agv_waypoint'                      # def 功能包名称

setup(
    name=package_name,                             # def 包名
    version='0.1.0',                               # def 版本号
    packages=find_packages(exclude=['test']),      # use 自动发现 Python 包
    data_files=[                                   # def 安装时拷贝的资源清单
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),         # io 注册 ament 资源索引标记
        ('share/' + package_name, ['package.xml']),    # io 安装 package.xml
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),   # io 安装 launch
    ],
    install_requires=['setuptools'],               # def Python 依赖
    zip_safe=True,                                 # def 允许 zip 安装
    maintainer='yuki',                             # def 维护者
    maintainer_email='3030067800@qq.com',          # def 维护者邮箱
    description='AGV 点位/路线管理：统一增删查列 + YAML 持久化（服务节点）',   # def 包描述
    license='MIT',                                 # def 许可证
    tests_require=['pytest'],                      # def 测试依赖
    entry_points={                                 # def 可执行入口注册表
        'console_scripts': [
            'waypoint_node=agv_waypoint.waypoint_node:main',   # io 注册导航点服务节点
        ],
    },
)
