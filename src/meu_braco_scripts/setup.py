from setuptools import find_packages, setup

package_name = 'meu_braco_scripts'

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
    maintainer='luis',
    maintainer_email='luisjacksonjr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_to_detected_object = meu_braco_scripts.point_to_detected_object:main',
            'pick_and_place_task = meu_braco_scripts.pick_and_place_task:main',
        ],
    },
)
