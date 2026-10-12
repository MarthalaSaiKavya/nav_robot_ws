from setuptools import find_packages, setup

package_name = 'my_bot_autonomy'

# Every entry below becomes a command:  ros2 run my_bot_autonomy <name>
NODES = {
    # simulator + robot drivers
    'lite_sim': 'sim.lite_sim',
    'odometry_processor': 'robot.odometry_processor',
    'twist_stamper': 'robot.twist_stamper',
    'sim_world': 'common.sim_world_cli',
    'wait_for_nav2': 'common.wait_for_nav2',
    # mapping tools
    'save_map': 'mapping.save_map_node',
    'make_reference_map': 'mapping.reference_map',
    'compare_maps': 'mapping.compare_maps',
    # exploration
    'frontier_explorer': 'exploration.frontier_explorer',
    'reactive_explorer': 'exploration.reactive_explorer',
    # mission
    'mission_runner': 'mission.mission_runner',
    'scenario_director': 'mission.scenario_director',
    'supervisor': 'mission.supervisor',
    # evaluation
    'evaluator': 'evaluation.evaluator',
    'run_report': 'evaluation.report',
    # from-scratch learning implementations
    'learning_mapper': 'learning.learning_mapper_node',
    'learning_localizer': 'learning.learning_localizer_node',
    'learning_planner': 'learning.learning_planner_node',
    'learning_controller': 'learning.learning_controller_node',
}

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python nodes for the autonomous navigation robot project',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [f'{name} = {package_name}.{module}:main' for name, module in NODES.items()],
    },
)
