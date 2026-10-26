"""Configuration and ROS node lifecycle management.

Provides the data-side counterpart to ``configuration.py``:

- :class:`UserConfigFileManager` persists arbitrary app settings to a JSON file.
- :class:`StaticParamsConfigLoader` pushes a YAML file straight onto the ROS
  parameter server.
- :class:`ConfigModel` reads/writes a specific set of ROS params backed by a
  YAML file (e.g. mapping parameters) and their valid ranges.
- :class:`NodesManager` builds ``roslaunch`` ``Node`` instances from
  dictionaries describing ROS nodes and starts/stops/restarts them, plus a
  few helpers for bring-up, map saving and rosbag recording.
"""
import json
import os
import subprocess
import time
from sys import executable
from typing import List

import rospy
import yaml
from PyQt5.QtCore import QObject, QSettings, QThread, pyqtSignal
from roslaunch import configure_logging, parent, rlutil
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch


class UserConfigFileManager:
    """Reads and writes a JSON file used to persist user-level app settings."""

    def __init__(self, filename: str = "./config/app_config.json"):
        self.filename = filename
        self.ensure_file_exists()

    def ensure_file_exists(self):
        """Create file with empty structure if it doesn't exist"""
        if not os.path.exists(self.filename):
            initial_data = {}
            self.write_data(initial_data)

    def read_data(self) -> dict:
        """Read data from JSON file"""
        try:
            with open(self.filename, "r") as file:
                return json.load(file)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error reading file: {e}")
            return {}

    def write_data(self, data):
        """Write data to JSON file"""
        try:
            with open(self.filename, "w") as file:
                json.dump(data, file, indent=4)
            return True
        except Exception as e:
            print(f"Error writing file: {e}")
            return False

    def update_value(self, key_path, new_value):
        """Update a specific value using key path"""
        data = self.read_data()

        # Navigate to the nested key
        current_level = data
        keys = key_path.split(".")

        for key in keys[:-1]:
            if key not in current_level:
                current_level[key] = {}
            current_level = current_level[key]

        # Update the final key
        current_level[keys[-1]] = new_value

        return self.write_data(data)

    def add_to_array(self, array_path, new_item):
        """Add item to an array in the JSON structure"""
        data = self.read_data()

        # Navigate to the array
        current_level = data
        keys = array_path.split(".")

        for key in keys:
            if key not in current_level:
                current_level[key] = []
            current_level = current_level[key]

        # Add new item
        current_level.append(new_item)

        return self.write_data(data)


class StaticParamsConfigLoader:
    """Loads a YAML file and pushes every key/value onto the ROS parameter
    server under ``workspace``, on construction."""

    def __init__(self, param_file: str, workspace: str = "/calamardo_loader/") -> None:
        with open(param_file, "r") as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)

        for key, value in data.items():
            rospy.set_param(workspace + key, value)


class ConfigModel:
    """Syncs a named set of ROS parameters with a YAML file on disk.

    ``param_file`` holds the default values; :meth:`get_params` reconciles
    them with whatever is already on the ROS parameter server, and
    :meth:`set_params` writes updated values back to both.
    """

    def __init__(
        self, param_file: str = "./config/mapping_params.yml", workspace: str = "elpepe"
    ) -> None:
        self.workspace = workspace
        self.param_file = param_file
        self.all_config_params = {}

        with open(self.param_file, "r") as file:
            self.data = yaml.load(file, Loader=yaml.SafeLoader)
            # print(self.data)

    def get_params(self) -> dict:
        """Return current param values, seeding the ROS server from the YAML
        defaults for any key not already set."""
        params = {}
        for key, value in self.data.items():
            if rospy.has_param(key):
                # if not isinstance(value, str):
                params.update({str(key): rospy.get_param(self.workspace + key)})
            else:
                rospy.set_param(self.workspace + key, value)
                params.update({str(key): value})

        # print(params)
        self.all_config_params.update(params)
        return params

    def get_ranges(self) -> dict:
        """Load the ``<param_file>_range.yml`` sibling describing valid
        min/max ranges for each parameter."""
        filename_with_extension = self.param_file.split("/")[-1]
        [filename, extension] = filename_with_extension.split(".")
        # filename = 'mapping_params'
        print(filename)
        param_file = f"./config/{filename}_range.yml"
        with open(param_file, "r") as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            # print('RANGE', data)
        return data

    def set_params(self, configs: dict) -> None:
        """Merge ``configs`` into the tracked params, push them to the ROS
        server and persist them to ``param_file``."""
        self.all_config_params.update(configs)
        for key, value in self.all_config_params.items():
            rospy.set_param(self.workspace + key, value)

        with open(self.param_file, "w") as file:
            yaml.dump(self.all_config_params, file, sort_keys=False)

    def restore_values(self):
        """Unimplemented hook for restoring previous parameter values."""
        pass


# class NodeWorker(QThread):
#     # finished = pyqtSignal()
#     # progress = pyqtSignal(int)
#
#     def __init__(self, node):
#         super().__init__()
#         self.node = node
#         self.subprocess = None
#
#     def run(self):
#         package, exec, name, arg, respawn = self.node['package'], self.node['exec'], self.node['name'], self.node.get('arg'), self.node.get('respawn')
#         if arg:
#             self.subprocess = subprocess.Popen(['rosrun', package, exec, arg, f'__name:={name}'])
#         else:
#             self.subprocess = subprocess.Popen(['rosrun', package, exec, f'__name:={name}'])

#         # self.subprocess = subprocess.Popen(['rosrun', package, exec, arg, f'__name:={name}'])
#         # self._launcher = ROSLaunch()
#         # self._launcher.start()
#         # self.subprocess = self._launcher.launch(self.node)
#         # for i in range(1, 101):
#         #     time.sleep(0.1)  # Simulate work
#         #     self.progress.emit(i)
#         # self.finished.emit()

#     def stop(self):
#         if self.subprocess:
#             self.subprocess.terminate()  # Send SIGTERM
#             # self.subprocess.stop()
#         if  self.thread.isRunning():
#             self.stop()


class NodesManager(QObject):
    """Creates, starts, stops and restarts ROS nodes on behalf of the GUI.

    Node descriptions (package/exec/name/args) come from the configuration
    panel; this class turns them into ``roslaunch`` ``Node`` objects (remapped
    to the currently selected robot's topics), launches them via a shared
    ``ROSLaunch`` instance and keeps track of the running subprocesses so
    they can be stopped or restarted later. Also exposes helpers to bring up
    the robot driver stack, save maps and record/stop rosbag sessions.
    """

    def __init__(self) -> None:
        super().__init__()
        self.nodes_subprocess = {}
        self.bringup = None
        self._launcher = ROSLaunch()
        self.rosbag = None
        self.started_nodes = {}
        self.user_configs = UserConfigFileManager()
        self._config = QSettings("MyCompany", "MyApp")

    def initNodes(self, nodes: List[dict] = []) -> dict:
        """Build a ``{name: roslaunch.Node}`` map from node descriptor dicts,
        remapping cmd_vel/odom/scan/imu topics to the selected robot."""
        node_instances = {}
        for node in nodes:
            node_values = node.values()
            package, exec, name, args, respawn = (
                node["package"],
                node["exec"],
                node["name"],
                node.get("arg"),
                node.get("respawn"),
            )
            if not respawn:
                respawn = False

            robots = self._config.value(
                "robots",
                {
                    "turtlebot3": {
                        "name": "turtlebot3",
                        "URDF_path": "empty",
                        "odom_topic": "odom",
                        "cmd_vel_topic": "cmd_vel",
                        "lidar_topic": "scan",
                        "imu_topic": "imu",
                        "baseframe": "base_link",
                    }
                },
            )
            robot_name = self._config.value("robot", "turtlebot3")
            robot = robots[robot_name]

            # "URDF_path": "",
            # "odom_topic": "",
            # "cmd_vel_topic": "",
            # "lidar_topic": ""

            node = Node(
                package=package,
                node_type=exec,
                name=name,
                args=args,
                respawn=False,
                remap_args=[
                    ("/cmd_vel", robot["cmd_vel_topic"]),
                    ("/odom", robot["odom_topic"]),
                    ("/scan", robot["lidar_topic"]),
                    ("/imu", robot["imu_topic"]),
                    ("/move_base_simple/goal", "/move_base_simple/goal2"),
                ],
            )
            node_instances.update(
                # {name: Node(package=package, node_type=exec, name=name, args=arg, output='screen',  respawn=respawn)}
                {name: node}
            )
            self.started_nodes.update(node_instances)
        return node_instances

    # def initNodes(self, nodes: List[dict] = []) -> List[dict]:
    # node_instances = {}
    # for node in nodes:
    #     node_values = node.values()
    #     package, exec, name, arg, respawn = node['package'], node['exec'], node['name'], node.get('arg'), node.get('respawn')
    #     if not respawn:
    #         respawn = False
    #
    #     node_instances.update(
    #         # {name: Node(package=package, node_type=exec, name=name, args=arg, output='screen',  respawn=respawn)}
    #         {name: Node(package=package, node_type=exec, name=name, args=arg,  respawn=respawn)}
    # )
    # return nodes

    def startNodes(self, node_instances: dict = {}) -> dict:
        """Launch every not-yet-running node in ``node_instances`` and track
        its subprocess handle."""
        self._launcher.start()
        # subprocesses = {name: self._launcher.launch(node) for name, node in node_instances.items()}
        for name, node in node_instances.items():
            # time.sleep(7)
            if not self.nodes_subprocess.get(name):
                _subprocess = {name: self._launcher.launch(node)}
                print(_subprocess)
                self.nodes_subprocess.update(_subprocess)
                # self.started_nodes.update({name: node})
        return {}

    # def startNodes(self, node_instances: List[dict] = []) -> dict:

    #     # subprocesses = {name: self._launcher.launch(node) for name, node in node_instances.items()}
    #     for node in node_instances :
    #         package, exec, name, arg, respawn = node['package'], node['exec'], node['name'], node.get('arg'), node.get('respawn')
    #         _subprocess = {name: NodeWorker(node)}
    #         _subprocess.get(name).start()
    #         # if arg:
    #         #     _subprocess = {name: subprocess.Popen(['rosrun', package, exec, arg, f'__name:={name}'])}
    #         # else:
    #         #     _subprocess = {name: subprocess.Popen(['rosrun', package, exec, f'__name:={name}'])}
    #         # _subprocess.get(name).start()
    #         print(_subprocess)
    #         self.nodes_subprocess.update(_subprocess)

    #     # for name, node in node_instances.items():
    #     #     if not self.nodes_subprocess.get(name):
    #     #         _subprocess = {name: NodeWorker(node)}
    #     #         _subprocess.get(name).start()
    #     #         print(_subprocess)
    #     #         self.nodes_subprocess.update(_subprocess)
    #     return {}

    def stopNodes(self, node_names: List[str]) -> None:
        """Stop the subprocesses for ``node_names`` and drop them from the
        tracked node maps."""
        # self.bringup.shutdown()
        # [subprocess.stop() for name, subprocess in self.nodes_subprocess.items()]
        # subprocess = self.get(node_name)
        # subprocess.stop()
        nodeToDelete = None
        nodeToDeleteArray = []

        for node_name in node_names:
            for name, _subprocess in self.nodes_subprocess.items():
                if name == node_name:
                    _subprocess.stop()
                    nodeToDeleteArray.append(name)
                    nodeToDelete = name

        # def stopNodes(self, node_names: List[str]) -> None:
        #     # self.bringup.shutdown()
        #     # [subprocess.stop() for name, subprocess in self.nodes_subprocess.items()]
        #     # subprocess = self.get(node_name)
        #     # subprocess.stop()
        #     nodeToDeleteArray = []

        #     for node_name in node_names:
        #         for name, _subprocess in self.nodes_subprocess.items():
        #             if name == node_name:
        #                 _subprocess.stop()
        #                 # _subprocess.terminate()  # Send SIGTERM
        #                 # _subprocess.wait()
        #                 nodeToDeleteArray.append(name)

        # print('node manager node to stop', nodeToDelete)
        # if nodeToDelete:
        #     self.nodes_subprocess.pop(nodeToDelete)
        for node_name in nodeToDeleteArray:
            self.nodes_subprocess.pop(node_name)
            self.started_nodes.pop(node_name)

    def restartNodes(self):
        """Stop every running node and relaunch the full ``started_nodes`` set."""
        for name, _subprocess in self.nodes_subprocess.items():
            _subprocess.stop()

        self.nodes_subprocess = {}
        self.startNodes(self.started_nodes)

    def bringUpStart(self):
        """Launch the turtlebot3 bring-up launch file (robot driver stack)."""
        # rospy.init_node("harold_start_launch", anonymous=True)

        uuid = rlutil.get_or_generate_uuid(None, False)
        configure_logging(uuid)
        self.bringup = parent.ROSLaunchParent(
            uuid,
            [
                "/pico-sdk/harold_ws/src/turtlebot3/turtlebot3_bringup/launch/turtlebot3_remote.launch"
            ],
        )
        self.bringup.start()
        # rospy.loginfo("started")

    def bringUpStop(self):
        """Shut down the robot bring-up launch, if it was started."""
        if self.bringup is not None:
            self.bringup.shutdown()

    def save_map(self, mapname):
        """Run ``map_server``'s ``map_saver`` to dump the current map to
        ``mapname`` and return the spawned subprocess."""
        x = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", f"{mapname}"])
        # x.wait()
        return x

    def topicHasPublisher(self, topic, publisher=None):
        """Return True if ``topic`` appears among currently published ROS topics."""
        for available_topic, _type in rospy.get_published_topics():
            if available_topic == topic:
                return True
            print("from topci :", available_topic, _type)
        return False

    def nodeIsRunning(self, nodename):
        """Return True if ``nodename`` has a tracked running subprocess."""
        if self.nodes_subprocess.get(nodename):
            return True
        return False

    def recordSensordata(self, topicslist: list):
        """Start a ``rosbag record`` subprocess capturing ``topicslist``."""
        topics = " ".join(topicslist)
        filename = "haroldjeje"
        self.rosbag = subprocess.Popen(["rosbag", "record", "-O", filename, topics])
        print("ROSBAG: ", topics)

    def stopRecordSensorData(self):
        """Terminate the active rosbag recording, if any."""
        if self.rosbag:
            self.rosbag.terminate()
        pass


if __name__ == "__main__":
    t = ConfigModel()
    t.get_params()
    # t.set_params([], [])
    pass
