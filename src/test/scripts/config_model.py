import subprocess
from sys import executable
from typing import List
import rospy
import yaml
import subprocess
import time

from roslaunch.core import Node
from roslaunch import rlutil
from roslaunch import parent
from roslaunch import configure_logging
from roslaunch.scriptapi import ROSLaunch


class StaticParamsConfigLoader:
    def __init__(self, param_file: str, workspace: str = "/calamardo_loader/") -> None:
        with open(param_file, "r") as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)

        for key, value in data.items():
            rospy.set_param(workspace + key, value)


class ConfigModel:
    def __init__(
        self, param_file: str = "./mapping_params.yml", workspace: str = "elpepe"
    ) -> None:
        self.workspace = workspace
        self.param_file = param_file

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
        return params

    def get_ranges(self) -> dict:
        filename_with_extension = self.param_file.split("/")[-1]
        [filename, extension] = filename_with_extension.split(".")
        # filename = 'mapping_params'
        print(filename)
        param_file = f"./{filename}_range.yml"
        with open(param_file, "r") as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)
            # print('RANGE', data)
        return data

    def set_params(self, configs: dict) -> None:
        for key, value in configs.items():
            rospy.set_param(self.workspace + key, value)

        with open(self.param_file, "w") as file:
            yaml.dump(configs, file, sort_keys=False)


class NodesManager:
    def __init__(self) -> None:
        self.nodes_subprocess = {}
        self.bringup = None

    def initNodes(self, nodes: List[dict] = []) -> dict:
        self._launcher = ROSLaunch()
        node_instances = {}
        for node in nodes:
            node_values = node.values()
            package, exec, name, arg, respawn = node['package'], node['exec'], node['name'], node.get('arg'), node.get('respawn')
            if not respawn:
                respawn = False
            
            node_instances.update(
                {name: Node(package=package, node_type=exec, name=name, args=arg, output='screen',  respawn=respawn)}
            )
        return node_instances

    def startNodes(self, node_instances: dict = {}) -> dict:

        self._launcher.start()
        # subprocesses = {name: self._launcher.launch(node) for name, node in node_instances.items()}
        for name, node in node_instances.items():
            # time.sleep(7)
            if not self.nodes_subprocess.get(name):
                _subprocess = {name: self._launcher.launch(node)}
                print(_subprocess)
                self.nodes_subprocess.update(_subprocess)
        return {}

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

        print('node manager node to stop', nodeToDelete)
        # if nodeToDelete:
        #     self.nodes_subprocess.pop(nodeToDelete)
        for node in nodeToDeleteArray:
            self.nodes_subprocess.pop(node)



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
        if  self.bringup is not None:
            self.bringup.shutdown()

    def save_map(self):
        subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', 'mymap'])

    def topicHasPublisher(self, topic):
        for available_topic, type in rospy.get_published_topics():
            if available_topic == topic:
                return True
        return False



if __name__ == "__main__":
    t = ConfigModel()
    t.get_params()
    # t.set_params([], [])
    pass
