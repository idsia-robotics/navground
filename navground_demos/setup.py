from setuptools import setup
import glob

package_name = 'navground_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
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
            'ros = navground_demo.ros:main',
            'enki = navground_demo.enki:main',
            'python = navground_demo.python:main',
        ],
    },
)
