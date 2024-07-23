from setuptools import setup

package_name = 'osrm_server'

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
    maintainer='wjh',
    maintainer_email='wjh_9696@163.com',
    description='Providing route service of osm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'osrm_server = osrm_server.osrm_server:main',
        ],
    },
)
