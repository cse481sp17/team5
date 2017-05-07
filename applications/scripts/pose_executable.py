

class PoseExecutable(object):
    OPEN = 1
    CLOSE = 2
    MOVETO = 3

    def __init__(self, actionType, frame, pose):
        self.actionType = actionType
        self.frame = frame
        self.pose = pose