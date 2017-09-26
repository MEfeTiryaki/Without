import os


if __name__=="__main__":
	os.system("python VirtualEngine.py &")
	os.system("python RobotControl_Two_player.py &")
	os.system("python Hub.py &")
	os.system("python GM_example_Predator_Prey.py &")
