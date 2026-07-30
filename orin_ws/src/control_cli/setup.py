from setuptools import setup

package_name = 'control_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm',
    maintainer_email='swarm@todo.todo',
    description='트랙 테스트베드 CLI (testbed_spec.md §5.1)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_cli = control_cli.cli:main',
        ],
    },
)
