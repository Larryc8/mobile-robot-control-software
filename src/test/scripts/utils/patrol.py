from enum import Enum

class PatrolEndState(Enum):
    ACTIVE = 0
    CANCELLED = 1
    SUCCESS = 2
    IMCOMPLETE = 3
    FINISHED = 4

