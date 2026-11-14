from setuptools import setup
package_name='rl_nav_pkg'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/'+package_name]),
        ('share/'+package_name,['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course project team',
    maintainer_email='course-project@example.invalid',
    description='BEV PPO inference for the course project',
    license='TODO',
    entry_points={'console_scripts':['policy_node = rl_nav_pkg.policy_node:main']},
)
