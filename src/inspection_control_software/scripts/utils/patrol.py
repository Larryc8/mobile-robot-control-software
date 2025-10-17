from enum import Enum

class PatrolEndState(Enum):
    ACTIVE = 0
    CANCELLED = 1
    SUCCESS = 2
    IMCOMPLETE = 3
    FINISHED = 4

class userOperation(Enum):
    CREATEMAP = 0
    LOADMAP = 1
    IDLE = 2

class operationMode(Enum):
    AUTO = 0
    MANUAL = 1

class checkpointEndState(Enum):
    PENDING = "<span style='color: red; font-weight: bold'>Pendiente</span>"
    CHECKED = "<span style='color: green; font-weight: bold'>Revisado</span>"
    NEXT = "<span style='color: yellow; font-weight: bold'>Siguiente</span>"
    NONE = "<span style='color: black; font-weight: bold'>NoA</span>"
