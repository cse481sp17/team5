class PoseExecutable(object):
    OPEN = 1
    CLOSE = 2
    MOVETO = 3

    def __init__(self, actionType, frame, pose = None, arPose = None):
        self.actionType = actionType
        self.relativeFrame = frame
        self.pose = pose
        self.arPose = arPose
        