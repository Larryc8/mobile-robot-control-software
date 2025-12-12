import json
import os
import subprocess
import time
from sys import executable
from typing import List

import rospy
import yaml
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from roslaunch import configure_logging, parent, rlutil
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch


class UserConfigFileManager:
    def __init__(self, filename: str = "./config/app_config.json"):
        self.filename = filename
        self.ensure_file_exists()

    def ensure_file_exists(self):
        """Create file with empty structure if it doesn't exist"""
        if not os.path.exists(self.filename):
            initial_data = {}
            self.write_data(initial_data)

    def read_data(self):
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
    def __init__(self, param_file: str, workspace: str = "/calamardo_loader/") -> None:
        with open(param_file, "r") as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)

        for key, value in data.items():
            rospy.set_param(workspace + key, value)


class ConfigModel:
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
        self.all_config_params.update(configs)
        for key, value in self.all_config_params.items():
            rospy.set_param(self.workspace + key, value)

        with open(self.param_file, "w") as file:
            yaml.dump(self.all_config_params, file, sort_keys=False)

    def restore_values(self):
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
    def __init__(self) -> None:
        super().__init__()
        self.nodes_subprocess = {}
        self.bringup = None
        self._launcher = ROSLaunch()
        self.rosbag = None
        self.started_nodes = {}

    def initNodes(self, nodes: List[dict] = []) -> dict:
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

            node_instances.update(
                # {name: Node(package=package, node_type=exec, name=name, args=arg, output='screen',  respawn=respawn)}
                {
                    name: Node(
                        package=package,
                        node_type=exec,
                        name=name,
                        args=args,
                        respawn=False,
                    )
                }
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
        for name, _subprocess in self.nodes_subprocess.items():
            _subprocess.stop()

        self.nodes_subprocess = {}
        self.startNodes(self.started_nodes)

    def bringUpStart(self):
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
        if self.bringup is not None:
            self.bringup.shutdown()

    def save_map(self, mapname):
        x = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", f"{mapname}"])
        # x.wait()
        return x

    def topicHasPublisher(self, topic, publisher=None):
        for available_topic, _type in rospy.get_published_topics():
            if available_topic == topic:
                return True
            print("from topci :", available_topic, _type)
        return False

    def nodeIsRunning(self, nodename):
        if self.nodes_subprocess.get(nodename):
            return True
        return False

    def recordSensordata(self, topicslist: list):
        topics = " ".join(topicslist)
        filename = "haroldjeje"
        self.rosbag = subprocess.Popen(["rosbag", "record", "-O", filename, topics])
        print("ROSBAG: ", topics)

    def stopRecordSensorData(self):
        if self.rosbag:
            self.rosbag.terminate()
        pass


if __name__ == "__main__":
    t = ConfigModel()
    t.get_params()
    # t.set_params([], [])
    pass
