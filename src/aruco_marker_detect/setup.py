from setuptools import find_packages, setup

package_name = 'aruco_marker_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/calibration_params.yaml']),  # Ensure the file is included
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey12',
    maintainer_email='rokey12@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_marker_detector = aruco_marker_detect.aruco_marker_detector:main',
            'aruco_move_test = aruco_marker_detect.aruco_move_test:main',
        ],
    },
    package_data={
        package_name: ['config/calibration_params.yaml'],  # Include the YAML file
    },
)
