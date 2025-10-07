import rclpy

from rclpy.executors import MultiThreadedExecutor

import component_explain_navigation.component_explainer_impl


def main():
    rclpy.init()

    explainer = component_explain_navigation.component_explainer_impl.explainerImpl()
    explainer_executor = MultiThreadedExecutor()
    explainer_executor.add_node(explainer)

    try:
        explainer_executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Goodbye!")
        explainer.destroy_node()


if __name__ == '__main__':
    main()
