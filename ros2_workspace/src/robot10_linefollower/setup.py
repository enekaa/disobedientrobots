from setuptools import find_packages, setup

package_name = 'robot10_linefollower'
submodules = 'robot10_linefollower.submodules'

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
    maintainer='disobedientrobot2',
    maintainer_email='disobedientrobot2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'start_robot10 = robot10_linefollower.LineFollower:main'
        ],
    },
)
#find_packages(exclude=['test'])
