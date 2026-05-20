#!/usr/bin/env python3

import re
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from interfaces.action import BaseCommand


class Mission_Node(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.base_client = ActionClient(
            self,
            BaseCommand,
            "/control/chassis/base_command"
        )

        self.action_running = False
        self.active_goal_handle = None

        # Workstation coordinates.
        # For now these are simulated coordinates.
        self.workstations = {
            "workstation1": [2.0, 1.0, 0.0],
            "workstation2": [-2.0, -1.0, 0.0],
            "workstation3": [-3.0, 2.5, 0.0],
            "workstation4": [-0.5, -0.5, 0.0],
            "workstation5": [7.0, 0.0, 0.0],
            "workstation6": [0.0, -0.5, 0.0],
            "workstation7": [-4.5, -3.0, 0.0],
            "workstation8": [0.5, -3.0, 0.0],
            "workstation9": [5.0, -3.0, 0.0],
        }

        # Experiments are just workstation sequences.
        self.experiments = {
            "experiment1": ["workstation1", "workstation2"],
            "experiment2": ["workstation2", "workstation1"],
            "experiment3": ["workstation1", "workstation2", "workstation1"],
            "experiment4": ["workstation2", "workstation1", "workstation2"],
        }

        # Experiment tracking
        self.experiment_active = False
        self.current_route = []
        self.current_route_index = 0
        self.returning_home_after_experiment = False

        # Timer used between experiment steps
        self.wait_timer = None

        # Time to wait at each workstation before continuing
        self.wait_at_station_s = 5.0

        self.get_logger().info("Mission node started.")

        # Run terminal input in a separate thread so ROS callbacks still work.
        self.input_thread = threading.Thread(
            target=self.terminal_input_loop,
            daemon=True
        )
        self.input_thread.start()

    def terminal_input_loop(self):
        while rclpy.ok():
            try:
                command_text = input("> ").strip().lower()
            except EOFError:
                return
            except KeyboardInterrupt:
                return

            if command_text == "":
                continue

            # Stop must always be allowed, even during an experiment.
            if command_text == "stop":
                self.send_stop_command()
                continue

            # During experiments, only stop is allowed as a manual command.
            if self.experiment_active:
                self.get_logger().warn(
                    "Ignoring command because an experiment is already running. Use 'stop' to interrupt."
                )
                continue

            if command_text == "manual":
                self.send_manual_command()
                continue

            if command_text == "home":
                self.send_movement_command([0.0, 0.0, 0.0])
                continue

            experiment_name = self.parse_experiment_command(command_text)

            if experiment_name is not None:
                self.start_experiment(experiment_name)
                continue

            target_pose = self.parse_goto_command(command_text)

            if target_pose is not None:
                self.send_movement_command(target_pose)
                continue

            self.get_logger().warn(
                "Unknown command. Try: 'goto ws 1', 'exp 1', 'home', or 'stop'"
            )

    def parse_goto_command(self, command_text):
        # Accepts:
        #   goto ws 1

        workstation_pattern = r"^goto\s+ws\s*(\d+)$"
        workstation_match = re.match(workstation_pattern, command_text)

        if not workstation_match:
            return None

        workstation_number = workstation_match.group(1)
        workstation_name = f"workstation{workstation_number}"

        target_pose = self.workstation_coordinates(workstation_name)

        if target_pose is None:
            self.get_logger().warn(f"Unknown workstation: {workstation_name}")
            return None

        return target_pose

    def parse_experiment_command(self, command_text):
        # Accepts:
        #   exp 1
        #   exp1

        experiment_pattern = r"^exp\s*(\d+)$"
        experiment_match = re.match(experiment_pattern, command_text)

        if not experiment_match:
            return None

        experiment_number = experiment_match.group(1)
        experiment_name = f"experiment{experiment_number}"

        return experiment_name

    def start_experiment(self, experiment_name):
        if experiment_name not in self.experiments:
            self.get_logger().warn(f"Unknown experiment: {experiment_name}")
            return

        self.current_route = self.experiments[experiment_name]
        self.current_route_index = 0
        self.experiment_active = True
        self.returning_home_after_experiment = False

        self.get_logger().info(
            f"Starting {experiment_name}: {' -> '.join(self.current_route)}"
        )

        self.send_current_experiment_step()

    def send_current_experiment_step(self):
        if not rclpy.ok():
            return

        # Important: if stop was pressed during the 5 second wait,
        # the old timer may still fire. This prevents the experiment from continuing.
        if not self.experiment_active:
            self.get_logger().warn("No active experiment. Not sending next step.")
            return

        if self.action_running:
            self.get_logger().warn("Cannot send next step because action is still running.")
            return

        # If all workstations are completed, return home.
        if self.current_route_index >= len(self.current_route):
            self.get_logger().info("Experiment route completed. Returning home.")

            self.current_route = []
            self.current_route_index = 0
            self.returning_home_after_experiment = True

            self.send_movement_command([0.0, 0.0, 0.0], internal_command=True)
            return

        workstation_name = self.current_route[self.current_route_index]
        target_pose = self.workstation_coordinates(workstation_name)

        if target_pose is None:
            self.get_logger().error(f"Unknown workstation in experiment: {workstation_name}")
            self.abort_experiment()
            return

        self.get_logger().info(
            f"Experiment step {self.current_route_index + 1}: going to {workstation_name}"
        )

        sent = self.send_movement_command(target_pose, internal_command=True)

        if not sent:
            self.get_logger().error("Failed to send experiment step.")
            self.abort_experiment()

    def send_movement_command(self, target_pose, internal_command=False):
        if self.action_running:
            self.get_logger().warn("Ignoring command because robot is already moving.")
            return False

        if self.experiment_active and not internal_command:
            self.get_logger().warn("Ignoring manual command because experiment is running.")
            return False

        if not self.base_client.server_is_ready():
            self.get_logger().warn("Base action server is not ready yet.")
            return False

        goal_msg = BaseCommand.Goal()
        goal_msg.command = "goto"
        goal_msg.target_pose = target_pose

        self.get_logger().info(f"Sending goto command: {goal_msg.target_pose}")

        self.action_running = True

        send_goal_future = self.base_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        try:
            self.active_goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Goal response failed: {error}")
            self.action_running = False
            self.active_goal_handle = None
            self.abort_experiment_if_active()
            return

        if not self.active_goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            self.action_running = False
            self.active_goal_handle = None
            self.abort_experiment_if_active()
            return

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = self.active_goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as error:
            self.get_logger().error(f"Goal result failed: {error}")
            self.action_running = False
            self.active_goal_handle = None
            self.abort_experiment_if_active()
            return

        self.action_running = False
        self.active_goal_handle = None

        if result.success:
            self.get_logger().info(f"Movement succeeded: {result.message}")

            if self.returning_home_after_experiment:
                self.returning_home_after_experiment = False
                self.experiment_active = False
                self.get_logger().info("Experiment finished. Robot is home.")
                return

            if self.experiment_active:
                finished_station = self.current_route[self.current_route_index]

                self.get_logger().info(
                    f"Finished {finished_station}. Waiting {self.wait_at_station_s} seconds."
                )

                self.current_route_index += 1

                self.wait_timer = threading.Timer(
                    self.wait_at_station_s,
                    self.send_current_experiment_step
                )
                self.wait_timer.daemon = True
                self.wait_timer.start()

        else:
            self.get_logger().error(f"Movement failed: {result.message}")
            self.abort_experiment_if_active()


    def send_stop_command(self):
        self.get_logger().warn("STOP command received.")

        # Stop any experiment sequence first.
        self.abort_experiment_if_active()

        # Cancel the current goto goal if possible.
        if self.active_goal_handle is not None:
            self.get_logger().warn("Cancelling active goto goal.")
            self.active_goal_handle.cancel_goal_async()

        self.action_running = False
        self.active_goal_handle = None

        if not self.base_client.server_is_ready():
            self.get_logger().warn("Base action server is not ready yet. Could not send stop.")
            return False

        goal_msg = BaseCommand.Goal()
        goal_msg.command = "stop"
        goal_msg.target_pose = []

        self.get_logger().warn("Sending stop command to base.")

        stop_future = self.base_client.send_goal_async(goal_msg)
        stop_future.add_done_callback(self.stop_response_callback)

        return True
    
    def stop_response_callback(self, future):
        try:
            stop_goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Stop response failed: {error}")
            return

        if not stop_goal_handle.accepted:
            self.get_logger().error("Stop command was rejected.")
            return

        self.get_logger().warn("Stop command accepted. Waiting for stop result...")

        result_future = stop_goal_handle.get_result_async()
        result_future.add_done_callback(self.stop_result_callback)

    def stop_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as error:
            self.get_logger().error(f"Stop result failed: {error}")
            return

        if result.success:
            self.get_logger().warn(f"Stop succeeded: {result.message}")
        else:
            self.get_logger().error(f"Stop failed: {result.message}")

    def workstation_coordinates(self, location_name):
        return self.workstations.get(location_name.lower(), None)

    def abort_experiment_if_active(self):
        if self.experiment_active or self.returning_home_after_experiment:
            self.abort_experiment()

    def abort_experiment(self):
        self.get_logger().error("Experiment aborted.")

        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

        self.experiment_active = False
        self.current_route = []
        self.current_route_index = 0
        self.returning_home_after_experiment = False

    def send_manual_command(self):
        if self.action_running or self.experiment_active:
            self.get_logger().warn("Cannot enter manual mode because robot is already moving.")
            return False

        if not self.base_client.server_is_ready():
            self.get_logger().warn("Base action server is not ready yet.")
            return False

        goal_msg = BaseCommand.Goal()
        goal_msg.command = "manual"
        goal_msg.target_pose = []

        self.get_logger().info("Sending manual command to base.")

        manual_future = self.base_client.send_goal_async(goal_msg)
        manual_future.add_done_callback(self.manual_response_callback)

        return True


    def manual_response_callback(self, future):
        try:
            manual_goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Manual response failed: {error}")
            return

        if not manual_goal_handle.accepted:
            self.get_logger().error("Manual command was rejected.")
            return

        self.get_logger().info("Manual command accepted. Waiting for result...")

        result_future = manual_goal_handle.get_result_async()
        result_future.add_done_callback(self.manual_result_callback)


    def manual_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as error:
            self.get_logger().error(f"Manual result failed: {error}")
            return

        if result.success:
            self.get_logger().info("Manual mode activated.")
            self.get_logger().info("Use 'stop' to exit manual mode.")
        else:
            self.get_logger().error(f"Manual command failed: {result.message}")





def main(args=None):
    rclpy.init(args=args)

    node = Mission_Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()