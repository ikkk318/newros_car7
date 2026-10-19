from glob import glob                              # use 通配符匹配 launch 文件
from setuptools import find_packages, setup        # use 打包工具与包发现函数

package_name = 'agv_task'                          # def 功能包名称

setup(
    name=package_name,                             # def 包名
    version='0.1.0',                               # def 版本号
    packages=find_packages(exclude=['test']),      # use 自动发现 Python 包（排除 test 目录）
    data_files=[                                   # def 安装时拷贝的资源文件清单
        # ament 资源索引标记（让 ros2 能找到本包）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),         # io 注册 ament 资源索引标记
        # 包描述文件
        ('share/' + package_name, ['package.xml']),      # io 安装 package.xml
        # 启动文件
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),   # io 安装全部 launch
    ],
    install_requires=['setuptools'],               # def Python 依赖声明
    zip_safe=True,                                 # def 允许以 zip 形式安装
    maintainer='yuki',                             # def 维护者
    maintainer_email='3030067800@qq.com',          # def 维护者邮箱
    description='AGV 任务层：指令总线 + 导航 + 搬运 + 安全',   # def 包描述
    license='MIT',                                 # def 许可证
    entry_points={                                 # def 可执行入口注册表
        'console_scripts': [
            # 任务层唯一执行者：指令路由 + 搬运状态机 + 安全层 + 查询服务
            'task_manager_node=agv_task.task_manager_node:main',   # io 注册任务层节点
        ],
    },
)
