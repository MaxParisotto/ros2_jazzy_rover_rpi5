from setuptools import setup

package_name = 'my_custom_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'save_map_node',  # Ensure this matches the name of your Python file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='Custom ROS2 nodes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_map_node = save_map_node:main',  # Ensure this matches the name of your Python file and function
        ],
    },
)