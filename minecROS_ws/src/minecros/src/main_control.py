import rospy

# communicates with the discord and movement node to control what happens
# reponsible for all the checks to not get banned
# read ocr coords and verify we have not been teleported and are still moving
# assert angle is within tolerance
# decide when to take breaks
# make sure crops are being broken

MAX_COORDS_TO_KEEP = 100
MAX_ANGLES_TO_KEEP = 100
ALLOWED_MOVEMENT_THRESHOLD = 3 # if we move more than this on X or Z, big problems and time to disconnect

class PrimaryController:

    def __init__(self):
        self.coord_sub = rospy.Subscriber("/minecros/coords", geometry_msgs.msg.Point, self.xyz_callback)
        self.angle_sub =
        self.coord_history = []
        self.angle_history = []
        self.movement_node_client = 
    

    # check X and Z which should not move t
    def xyz_callback(self, msg):
        most_recent = self.coord_history[-1]
        if abs(msg.x - most_recent.x) > ALLOWED_MOVEMENT_THRESHOLD or abs(msg.z - most_recent.z) > ALLOWED_MOVEMENT_THRESHOLD:
            rospy.logerr("Coord jump detected! Disconnecting")
            # call discord bot here
            
            # 

        self.coord_history.append(msg)
        




if __name__ == '__main__':
    rospy.logwarn("Initalizing main control node")
    rospy.init_node('main_control_node')
    name = rospy.get_name()
    movement_server = MovementController(name)
    rospy.spin()