from setuptools import setup

package_name = 'control_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # UI 는 파이썬 문자열이 아니라 파일로 둔다 — 편집·검토가 쉽고 diff 가 읽힌다.
        # ⚠ 새 파일을 여기 안 적으면 개발 트리에서는 되고 설치본에서만 404 가 난다.
        ('share/' + package_name + '/www',
         ['www/index.html',
          'www/regmap.js', 'www/regmap.json', 'www/regmap.dpc.json',
          'www/profile.js', 'www/project.js']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm',
    maintainer_email='swarm@todo.todo',
    description='control 모드 웹 조작 UI (redesign/01 §6.1.3)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_web = control_web.server:main',
        ],
    },
)
