from setuptools import find_packages, setup

package_name = 'reto_final_laptop'

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
    maintainer='abraham',
    maintainer_email='abrahamortizcastro1229@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sign_detection = reto_final_laptop.sign_detection:main',
            'vertical_detection = reto_final_laptop.vertical_detection:main'
        ],
    },
)
