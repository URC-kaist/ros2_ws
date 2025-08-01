from setuptools import setup

package_name = 'mr2_rover_auto'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # The inner Python module directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/estimator.launch.py']),
        (f'share/{package_name}/config', ['config/ekf.yaml', 'config/navsat.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomy launch and config package for MR2 Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
