from setuptools import setup

package_name = 'fastlap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'deltri'],
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
        'odom = fastlap.odom:main',
        'odom_path_gen_viz = fastlap.odom_path_gen_viz:main',
        'stanley_fast_control = fastlap.stanley_fast_control:main',
        'delaunay_tri_path = fastlap.delaunay_tri_path:main',
        'sensing_perception = fastlap.sensing_perception:main'
        ],
    },
)
