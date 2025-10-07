import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    for dep in ["skill_explain",
                "component_explain_planner",
                "component_explain_navigation",
                "component_explain_ask_human_for_help",
                "component_explain_say",
                "component_explain_recommend_pizza",
                ]:
        ld.add_action(IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(dep), 'launch'), f'/{dep}.launch.py'])
        ))

    return ld
