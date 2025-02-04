from setuptools import find_packages, setup

package_name = 'conveyor_pkg'

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
    maintainer='supreme',
    maintainer_email='aj1234@hotmail.co.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor = conveyor_pkg.conveyor:main',
            'conveyor_gui = conveyor_pkg.conveyor_gui:main',
            'camera = conveyor_pkg.camera:main'
        ],
    },
)
