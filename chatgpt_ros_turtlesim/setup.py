from setuptools import setup

package_name = 'chatgpt_ros_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/demo.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hy',
    maintainer_email='810130242@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_turtle = chatgpt_ros_turtlesim.chat_turtle:main',
        ],
    },
)
