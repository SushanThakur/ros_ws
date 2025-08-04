from setuptools import find_packages, setup

package_name = 'self_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sushant',
    maintainer_email='mail@sushant.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = self_py_pkg.simple_publisher:main',
            'simple_subscriber = self_py_pkg.simple_subscriber:main',
            'simple_parameter = self_py_pkg.simple_parameter:main',
        ],
    },
)
