from setuptools import setup

package_name = 'erp_control'

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
    maintainer='mds2',
    maintainer_email='mds2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ERP42_ros2 = erp_control.ERP42_ros2:main',
        'test_steer = erp_control.test_steer:main'
        ],
    },
)
