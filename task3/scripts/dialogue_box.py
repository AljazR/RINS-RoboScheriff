#!/usr/bin/python3

import rospy
import os
from std_msgs.msg import String
from task3.srv import Dialogue, DialogueResponse

global colors_pub

def chat(req):
    global colors_pub
    os.system('clear')

    res = DialogueResponse()
    cylinder_colors = ["red", "green", "blue", "yellow"]
    ring_colors = ["red", "green", "blue", "black"]

    cylinders = []
    ring = ""

    print("SHERIFF: Hello! Do you know where the robber is hiding?")
    user_input = input("YOU:     ")
    user_input = user_input.lower()

    error = 0
    if "no" not in user_input:
        while error != 2:
            error = 0
            for color in cylinder_colors:
                if color in user_input:
                    cylinders.append(color)
                    error += 1
            if error != 2:
                print("SHERIFF: You need to tell me two of these colors: red, green, blue, yellow.")
                user_input = input("YOU:     ")
                user_input = user_input.lower()

        print("SHERIFF: To which prison do I have to take him?")
        user_input = input("YOU:     ").lower()
        user_input = user_input.lower()

        error = True
        while error:
            for color in ring_colors:
                if color in user_input:
                    ring = color
                    error = False
            if error:
                print("SHERIFF: You need to tell me one of these colors: red, green, blue, black.")
                user_input = input("YOU:     ")
                user_input = user_input.lower()


    print("SHERIFF: Thank you, bye.")
    
    # clear console
    rospy.sleep(3)
    os.system('clear')

    if len(cylinders) < 2 or ring == "":
        res.success = False
    else:
        res.success = True
        print(f"cylinders: {cylinders} ring: {ring}")
        

        # Publish the colors
        colors_pub.publish(f"{cylinders[0]} {cylinders[1]} {ring}")

    return res

def main():
    rospy.init_node('dialogue_box_service')

    # make publisher for colors
    global colors_pub
    colors_pub = rospy.Publisher('/colors', String, queue_size=10)

    # make service for dialogue
    rospy.Service('dialogue_box', Dialogue, chat)
    
    rospy.spin()

if __name__ == '__main__':
    main()