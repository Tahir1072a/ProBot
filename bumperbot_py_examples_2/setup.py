from setuptools import find_packages, setup

package_name = 'bumperbot_py_examples_2'

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
    maintainer='tahir',
    maintainer_email='tahirifdn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_parameters = bumperbot_py_examples_2.simple_parameters:main",
            "simple_kinematics = bumperbot_py_examples_2.simple_kinematics:main",
            "simple_service_server = bumperbot_py_examples_2.simple_service_server:main",
            "simple_service_client = bumperbot_py_examples_2.simple_service_client:main"
        ],
    },
)
