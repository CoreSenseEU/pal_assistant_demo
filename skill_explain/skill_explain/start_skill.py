import rclpy

from rclpy.executors import MultiThreadedExecutor

import skill_explain.skill_impl


def main():
    rclpy.init()

    skill = skill_explain.skill_impl.GenerateExplanationSkillImpl()
    skill_executor = MultiThreadedExecutor()
    skill_executor.add_node(skill)

    try:
        skill_executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Goodbye!")
        skill.destroy_node()


if __name__ == '__main__':
    main()
