from setuptools import find_packages, setup

package_name = 'ir_tracking_pub'
submodules = 'ir_tracking_pub/module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['ir_tracking_pub/pts1.npy']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryo',
    maintainer_email='rtasansax@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ir_tracking_pub.publisher_member_function:main',
        ],
    },
)
