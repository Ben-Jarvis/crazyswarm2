from setuptools import find_packages, setup

package_name = 'symbiotic_crazyswarm'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/swarm_launch.py',
    ]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jarvis',
    maintainer_email='ben_jarvis@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_utils = symbiotic_crazyswarm.swarm_utils:main',
            'my_vel_mux = symbiotic_crazyswarm.my_vel_mux:main',
            'rc_pilot = symbiotic_crazyswarm.rc_pilot:main',
            'swarm_pilot = symbiotic_crazyswarm.swarm_pilot:main',
            'multi_stream = symbiotic_crazyswarm.multi_stream:main',
       ],
    },
)
