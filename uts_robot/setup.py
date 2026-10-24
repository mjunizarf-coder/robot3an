from setuptools import find_packages, setup

package_name = 'uts_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mukhammad Junizar Firmansyah',
    maintainer_email='m.junizar.f@gmail.com',
    description='UTS Robotika: node penggerak robot robin melewati track UTS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uts_node = uts_robot.uts_node:main'
        ],
    },
)
