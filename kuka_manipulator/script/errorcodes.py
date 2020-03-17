from enum import Enum
class ErrorCodes(Enum):

    NO_OBJECT = "No object found while closing"
    OBJECT_FOUND = "Object found while closing"
    COLLISION = "Collision"
    OPEN = "Open: Requested Position"