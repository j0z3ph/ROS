from setuptools import find_packages, setup

package_name = 'py_turtle_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j0z3ph',
    maintainer_email='jlcruz@ipn.mx',
    description='Simple publisher for Turtle-Car Example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_turtle_publisher.turtle_publisher:main',
        ],
    },
)
