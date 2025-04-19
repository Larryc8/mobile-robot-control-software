import rospy
import yaml


class ConfigModel:
    def __init__(self, param_file: str = "./mapping_params.yml") -> None:
        self.workspace = "/turtlebot3_slam_gmapping/"
        self.param_file = param_file
        with open(param_file, "r") as file:
            self.data = yaml.load(file, Loader=yaml.SafeLoader)
            # print(self.data)

    def get_params(self) -> dict:
        params = {}
        for key, value in self.data.items():
            if rospy.has_param(key):
                if not isinstance(value, str):
                    params.update({str(key): rospy.get_param(self.workspace + key)})
            else:
                params.update({str(key): value})

        print(params)
        return params 

    def set_params(self, names: list, values: list) -> None:
        params = {}
        for name, value in zip(names, values):
            params.update({name: value})
            rospy.set_param(name, value)

        with open(self.param_file, "w") as file:
            yaml.dump(params, file, sort_keys=False)


if __name__ == "__main__":
    t = ConfigModel()
    t.get_params()
    # t.set_params([], [])
    pass
