from setuptools import setup

package_name = 'examples_rclpy_minimal_publisher'

setup(
    name=package_name,
    version='0.9.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rx_joy_pub = examples_rclpy_minimal_publisher.rx_joy_pub:main',
        ],
    },
)
