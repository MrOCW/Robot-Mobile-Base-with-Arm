from setuptools import setup

package_name = 'ESP32_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ocw',
    maintainer_email='ocw_97@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_mqtt = ESP32_control.ros2_mqtt:main',
            'ros2_serial = ESP32_control.ros2_serial:main'
        ],
    },
)
