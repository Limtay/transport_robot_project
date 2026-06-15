from setuptools import setup

package_name = 'carrier_teleop'

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
    maintainer='swarm',
    maintainer_email='rudxo970@gmail.com',
    description='키보드 멀티키 teleop (가상 조이스틱) for carrier robot cmd_vel.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = carrier_teleop.keyboard_teleop:main',
        ],
    },
)
