from setuptools import find_packages, setup

package_name = 'controller_ps4'

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
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_ps4 = controller_ps4.controller_ps4:main',
            'test_controller = controller_ps4.test_controller:main'
        ],
    },
)
