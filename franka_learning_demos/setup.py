from glob import glob
from setuptools import find_packages, setup

setup(
    name='franka_learning_demos', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/franka_learning_demos']),
        ('share/franka_learning_demos', ['package.xml']),
        ('share/franka_learning_demos/launch', glob('launch/*.launch.py')),
        ('share/franka_learning_demos/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Ming', maintainer_email='ming@example.com', license='Apache-2.0',
    description='Panda simulation learning demos and measured trajectory reports.',
    entry_points={'console_scripts': [
        'circle_demo = franka_learning_demos.circle_demo:main',
        'plot_circle = franka_learning_demos.report:main',
    ]},
)
