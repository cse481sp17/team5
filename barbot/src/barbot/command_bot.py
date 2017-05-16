class Commander(object):
    OPEN_GRIPPER = 1
    CLOSE_GRIPPER = 2
    GO_TO = 3
    MOVE_ARM_TO = 4

    def __init__(self, actionType, relativeFrame = None, pose = None, arPose = None):
        self.actionType = actionType
        self.relativeFrame = frame
        self.pose = pose
        self.arPose = arPose
