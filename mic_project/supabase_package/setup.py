from setuptools import find_packages, setup

package_name = 'supabase_package'

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
    maintainer='parallels',
    maintainer_email='me23b018@smail.iitm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supabase_reader = supabase_package.supabase_reader:main',
            'map_node = supabase_package.map_node:main',
            'plotter = supabase_package.plotter_node:main',
        ],
    },
)
