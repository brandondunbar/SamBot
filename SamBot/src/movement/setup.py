from setuptools import setup

package_name = 'movement'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brandon Dunbar',
    maintainer_email='brandon.dunbar97@gmail.com',
    description='Handles movement of the bot.',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = movement.move:main',
            'client = movement.movement_client:main',
        ],
    },
)
