from setuptools import setup

package_name = 'rostron_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etienne',
    maintainer_email='contact@etienne-schmitz.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation = rostron_nav.localisation:main',
            'nav2order = rostron_nav.nav2order:main'

        ],
    },
)
