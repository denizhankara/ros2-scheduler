from setuptools import setup
import os
import glob

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kara4@illinois.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = my_python_pkg.my_python_node:main',
            'publisher = my_python_pkg.publisher:main',
            'subscriber = my_python_pkg.subscriber:main',
            'sub_sch = my_python_pkg.subscriber_scheduled:main',
            'pub_sch = my_python_pkg.publisher_scheduled:main',
        ],
    },
)
