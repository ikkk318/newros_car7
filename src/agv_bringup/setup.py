from glob import glob                              # use 通配符匹配 launch 文件
from setuptools import find_packages, setup        # use 打包工具与包发现

package_name = 'agv_bringup'                       # def 功能包名称

setup(
    name=package_name,                             # def 包名
    version='0.1.0',                               # def 版本号
    packages=find_packages(exclude=['test']),      # use 自动发现 Python 包
    data_files=[                                   # def 安装时拷贝的资源清单
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),         # io 注册 ament 资源索引标记
        ('share/' + package_name, ['package.xml']),    # io 安装 package.xml
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),   # io 安装 launch
        # 参数中心：default 打底，real/ 为实车覆盖层（sim 无差异，故不设该层）
        ('share/' + package_name + '/config/default', glob('config/default/*.yaml')),
        ('share/' + package_name + '/config/real', glob('config/real/*.yaml') + glob('config/real/*.md')),
    ],
    install_requires=['setuptools'],               # def Python 依赖
    zip_safe=True,                                 # def 允许 zip 安装
    maintainer='yuki',                             # def 维护者
    maintainer_email='3030067800@qq.com',          # def 维护者邮箱
    description='AGV 参数中心 + 总启动',            # def 包描述
    license='MIT',                                 # def 许可证
    entry_points={'console_scripts': []},          # def 无节点入口（只提供 launch 与配置）
)
