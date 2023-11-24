from setuptools import setup

package_name = 'cyberdog_bluetooth_network'

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
    maintainer='dingsong1',
    maintainer_email='dingsong1@xiaomi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyberdog_bluetooth_main = cyberdog_bluetooth_network.cyberdog_bluetooth_main:main'
        ],
    },
)
