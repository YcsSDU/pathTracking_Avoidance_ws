from setuptools import find_packages, setup

package_name = 'car_nmpc_cbf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycs',
    maintainer_email='ycs@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "nmpc_cbf_node = car_nmpc_cbf.nmpc_cbf_node:main",
            "controller_node = car_nmpc_cbf.controller_node:main",
            "publish_rectangle_cmdvel_node = car_nmpc_cbf.publish_rectangle_cmdvel_node:main",
        ],
    },
)
