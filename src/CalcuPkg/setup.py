from setuptools import find_packages, setup

package_name = 'CalcuPkg'

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
    maintainer='hotaka',
    maintainer_email='hhhotaka2004@gmail.com',
    description='TODO: Package description',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pos_node = CalcuPkg.pos_node:main',
            'calcu_node = CalcuPkg.calcu_node:main',
            'debug_node = CalcuPkg.debug_node:main'
        ],
    },
)
