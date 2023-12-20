from setuptools import find_packages, setup

package_name = 'gnssrecv'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannu Hakalahti',
    maintainer_email='hannu.hakalahti@seamk.fi',
    description='Read position from u-blox GNSS receiver',
    license='CC-BY-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = gnssrecv.main:main',
        ],
    },
)
