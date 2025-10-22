from setuptools import find_packages, setup

package_name = 'a105_leash_top'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", ["launch/depth_proc.launch.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mobile',
    maintainer_email='mobile@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'edge_depth_to_scan = a105_leash_top.edge_depth_to_scan:main',
            'rgbd_to_cmd = a105_leash_top.rgbd_to_cmd:main',
            'stenka = a105_leash_top.stenka:main',
        ],
    },
)
