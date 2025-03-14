from setuptools import find_packages, setup

package_name = 'py02_service'

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
    maintainer='oshacker',
    maintainer_email='1500438364@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo01_client_py = py02_service.demo01_client_py:main',
            'demo02_server_py = py02_service.demo02_server_py:main'
        ],
    },
)
