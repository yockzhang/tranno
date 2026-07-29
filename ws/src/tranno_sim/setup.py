from setuptools import setup
import os
from glob import glob

package_name = 'tranno_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yock Zhang',
    maintainer_email='yockzhang@gotranno.com',
    description='Tranno T-01 sim',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'delivery_demo = tranno_sim.delivery_demo:main',
        ],
    },
)
