from setuptools import setup
from glob import glob

package_name = 'final_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a final_v2 Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker = final_v2.marker:main',
            'test2 = final_v2.test2:main',
            'marker3 = final_v2.marker3:main',
            'test3 = final_v2.test3:main',
            'reach_target_control = final_v2.reach_target_control:main',
            'target_marker = final_v2.target_marker:main',
            'test6 = final_v2.test6:main',
            'test7 = final_v2.test7:main',
            'testk = final_v2.test_knot:main',
            'testk2 = final_v2.test_knot_2:main',
            
        ],
    },
)
