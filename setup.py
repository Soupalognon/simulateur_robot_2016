from setuptools import find_packages, setup
from glob import glob, iglob
import os 

package_name = 'simulateur_robot_2016'

def package_files(directory):
    """Retourne une liste de tous les fichiers sous 'directory'."""
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(path, filename)
            paths.append(filepath)
    return paths

# Récupération récursive des ressources
resource_files = package_files('resource')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name, glob('launch/*.*')),
        ('share/' + package_name + '/resource/', glob('resource/*.*')),
        ('share/' + package_name + '/resource/proto/', glob('resource/proto/*.*')),
        ('share/' + package_name + '/resource/meshes/collision/', glob('resource/meshes/collision/*.*')),
        ('share/' + package_name + '/resource/meshes/visual/', glob('resource/meshes/visual/*.*')),
        ('share/' + package_name + '/resource/urdf/', glob('resource/urdf/*.*')),
        ('share/' + package_name + '/resource/worlds/', glob('resource/worlds/*.*') + glob('resource/worlds/.*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gdu',
    maintainer_email='gabriel.durand@hotmail.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
