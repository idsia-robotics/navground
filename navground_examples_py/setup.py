from setuptools import setup

package_name = 'navground_examples_py'

setup(
    name=package_name,
    version='0.0.6',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jerome Guzzi',
    maintainer_email='jerome@idsia.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior = navground_examples_py.behavior:main',
            'controller = navground_examples_py.controller:main',
            'run = navground_examples_py.run:main',
            'experiment = navground_examples_py.experiment:main',
            'rt_decorate = navground_examples_py.rt_decorate:main',
        ],
        'navground_behaviors':
        ['idle = navground_examples_py.my_behavior:PyIdleBehavior'],
    },
)
