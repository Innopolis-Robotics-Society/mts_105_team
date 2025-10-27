from setuptools import setup
package_name = 'a105_mpu6500_spi_py'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mpu6500_spi.launch.py']),
        ('share/' + package_name + '/config', ['config/mpu6500_spi.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team105',
    maintainer_email='team105@example.com',
    description='MPU6500 over spidev publisher',
    license='MIT',
    entry_points={'console_scripts': [
        'mpu6500_spi_node = a105_mpu6500_spi_py.mpu6500_spi_node:main',
    ]},
)
