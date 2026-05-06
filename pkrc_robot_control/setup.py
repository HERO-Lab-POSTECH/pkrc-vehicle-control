from setuptools import setup, find_packages

package_name = 'pkrc_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    package_data={
        f'{package_name}.gui': ['templates/*.html'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pkrc.yaml']),
        ('share/' + package_name + '/launch', ['launch/pkrc_main.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='luckkim123@gmail.com',
    description='PKRC robot main control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control = pkrc_robot_control.main:main',
        ],
    },
)
