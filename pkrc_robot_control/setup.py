from setuptools import setup

package_name = 'pkrc_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
