from setuptools import find_packages, setup

package_name = 'ai_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gvrose8192@gmail.com',
    description='Obstacle detection with Lidar and AI deciding which way to turn',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_driver_node = ai_driver.ai_driver_node:main'
        ],
    },
)
