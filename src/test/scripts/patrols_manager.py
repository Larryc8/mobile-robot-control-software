
class NodesManager():
    def __init__(self) -> None:
        self.patrols = []
        self.dispatch_patrol = None

    def add(self, patrol = None):
        self.patrols.append(patrol)
        pass
    def remove(self):
        pass
    def dispatch(self):
        pass
