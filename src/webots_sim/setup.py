from setuptools import find_packages, setup
import os

package_name = 'webots_sim'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# List of folders you want to install recursively
resource_folders = ['protos', 'projects',
                    'worlds', 'launch', 'config', 'controllers']

for folder in resource_folders:
    for root, _, files in os.walk(folder):
        for file in files:
            file_path = os.path.join(root, file)
            install_path = os.path.join('share', package_name, root)
            data_files.append((install_path, [file_path]))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jannen',
    maintainer_email='thyman_bathlo@yahoo.com',
    description='Webots simulation for ROS 2',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = webots_sim.obs_avoidance:main',
            'state_machine = webots_sim.featured_state_machine:main',

        ],
    },
)
