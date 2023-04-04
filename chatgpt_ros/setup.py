from setuptools import setup

package_name = 'chatgpt_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
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
            'chatgpt_action_server = chatgpt_ros.chatgpt_action_server:main',
            'chatgpt_action_client = chatgpt_ros.chatgpt_action_client:main'
        ],
    },
)
