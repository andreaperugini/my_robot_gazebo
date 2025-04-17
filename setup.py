from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'my_robot_gazebo'


# Percorso relativo alla directory dei mesh
meshes_dir = 'meshes'
mesh_files = []

# Aggiungi tutti i file nella directory meshes
def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
     #   (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.[urdf][xacro]*'))),
       
  
        ('target_directory_2', glob('nested_source_dir/**/*', recursive=True)), # includes sub-folders - recursive
    #    (os.path.join('share', package_name), glob('scripts/*')),

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + generate_data_files('share/' + package_name + '/', 'meshes') 
    + generate_data_files('share/' + package_name + '/', 'config')
    + generate_data_files('share/' + package_name + '/', 'urdf')
    + generate_data_files('share/' + package_name + '/', 'scripts'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_entity = my_robot_gazebo.spawn_entity:main',
            'controller_joint = my_robot_gazebo.controller_joint:main'
        ],
    },

    # scripts = [
    #     'scripts/spawn_robot_xacro.py'
    # ]
)
