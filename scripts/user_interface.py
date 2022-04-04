import rospy
import time
from rt2_assignment1.srv import Command

# Interface for getting the user inputs
def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    # Getting the input and assigning to the x variable
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():                                           
        # Checking the x value and sending it to the user interface server
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("\nThe robot is stopping")
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
