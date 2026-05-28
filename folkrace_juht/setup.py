import os
from glob import glob
from setuptools import setup

package_name = 'folkrace_juht'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@tktk.ee',
    description='MOBROS moodul 4b: Iseseisev folkrace sõitmine',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'folkrace_juht = folkrace_juht.folkrace_juht:main',
        ],
    },
)
