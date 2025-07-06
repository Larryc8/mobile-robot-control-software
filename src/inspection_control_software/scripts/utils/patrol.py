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
