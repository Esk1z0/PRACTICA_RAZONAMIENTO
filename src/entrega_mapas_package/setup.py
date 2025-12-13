from setuptools import setup
import os
from glob import glob

package_name = 'entrega_mapas_package'

setup(
    # ========== METADATOS ==========
    name=package_name,
    version='0.0.1',

    # ========== PAQUETES PYTHON ==========
    packages=[package_name],

    # ========== ARCHIVOS DE DATOS ==========
    data_files=[
        # Registro del paquete
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files (todos los *.launch.py)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],

    # ========== DEPENDENCIAS ==========
    install_requires=['setuptools'],
    zip_safe=True,

    # ========== INFO AUTOR ==========
    maintainer='juanes',
    maintainer_email='tu@email.com',
    description='Sistema Bug2 con CoppeliaSim para la entrega de mapas',
    license='Apache License 2.0',

    tests_require=['pytest'],

    # ========== NODOS EJECUTABLES ==========
    entry_points={
        'console_scripts': [
            # ros2 run entrega_mapas_package coppelia_interface_node
            'coppelia_interface_node = entrega_mapas_package.coppelia_interface_node:main',

            # ros2 run entrega_mapas_package bug2_controller_node
            'bug2_controller_node = entrega_mapas_package.bug2_controller_node:main',

            # ros2 run entrega_mapas_package goal_manager_node
            'goal_manager_node = entrega_mapas_package.goal_manager_node:main',

            # ros2 run entrega_mapas_package occupancy_mapper_node
            'occupancy_mapper_node = entrega_mapas_package.occupancy_mapper_node:main',
        ],
    },
)
