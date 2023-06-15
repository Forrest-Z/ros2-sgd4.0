from setuptools import setup

package_name = 'sdtl_action_server'

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
    maintainer='Helmer Barcos',
    maintainer_email='helmer@barcos.co',
    description='Python action server for Shared Dog Traffic Lights project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdtl_action_server = sdtl_action_server.sdtl_action_server:main',
        ],
    },
)