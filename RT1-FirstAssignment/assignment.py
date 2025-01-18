from __future__ import print_function
import time
from sr.robot import *

a_th = 2.0                                   # float: Threshold for the control of the orientation
d_th_slow = 0.4                              # float: Threshold for the control of the linear distance (for the slowest velocity for the silver token)
d_th_fast = 1                                # float: Threshold for the control of the linear distance (for the higest velocity of both token)
d_th_2 = 0.55                                # float: Threshold for the control of the linear distance (for the slowest velocity of the golden token)
silver_boxes = []                            # array of the taken silver token
gold_boxes = []                              # array of the taken golden token
markers = []                                 # array of the token that are seen by the robot
i = 1

R = Robot()                                  # instance of the class Robot

def drive(speed, seconds): # Function for setting a linear velocity - Args: speed (int): the speed of the wheels - seconds (int): the time interval
	R.motors[0].m0.power = speed
	R.motors[0].m1.power = speed
	time.sleep(seconds)
	R.motors[0].m0.power = 0
	R.motors[0].m1.power = 0

def turn(speed, seconds): # Function for setting an angular velocity - Args: speed (int): the speed of the wheels - seconds (int): the time interval
	R.motors[0].m0.power = speed
	R.motors[0].m1.power = -speed
	time.sleep(seconds)
	R.motors[0].m0.power = 0
	R.motors[0].m1.power = 0

def find_token_silver(): # Function for finding the closest silver token - Returns: dist (float): distance of the closest token (-1 if no token is detected) - rot_y (float): angle between the robot and the token (-1 if no token is detected) - code: id of the token
	dists = []
	dist = 0
	markers = []
	markers = R.see()
	if markers is not None:
		for token in markers:
			if token.info.marker_type == MARKER_TOKEN_SILVER:
				dists.append(token.dist)
		if len(dists) == 0:
			return -1, -1, -1
		dist = min(dists)
		for token in markers:
			if token.dist == dist:
				return token.dist, token.rot_y, token.info.code
	else:
		return -1,-1,-1

def find_token_gold(): # Function for finding the closest golden token - Returns: dist (float): distance of the closest token (-1 if no token is detected) - rot_y (float): angle between the robot and the token (-1 if no token is detected) - code: id of the token
	dists = []
	dist = 0
	markers = []
	markers = R.see()
	if markers is not None:
		for token in markers:
			if token.info.marker_type == MARKER_TOKEN_GOLD:
				dists.append(token.dist)
		if len(dists) == 0:
			return -1, -1, -1
		dist = min(dists)
		for token in markers:
			if token.dist == dist:
				return token.dist, token.rot_y, token.info.code
	else:
		return -1,-1,-1
			
def dist_of_token(code, color):  # Function for knowing the current distance of the token - Returns the dist of the token with that particular code and color
	if color == "silver":
		for token in R.see():
			if token.info.code == code and token.info.marker_type == MARKER_TOKEN_SILVER:
				return token.dist
	else:
		for token in R.see():
			if token.info.code == code and token.info.marker_type == MARKER_TOKEN_GOLD:
				return token.dist

def goto_and_grab_silver(dist, rot_y, code):  # Function for reach and grab the silver token
	if dist < d_th_slow: 
		print("Found it!")
		R.grab()                      # if we are close to the token, we grab it.
		print("Gotcha!")
		silver_boxes.append(code)
		for i in range(len(silver_boxes)):
			print(" I have the ", silver_boxes[i]," box! \n")
	else:
		if -a_th <= rot_y <= a_th:    # if the robot is well aligned with the token, we go forward
			print("Ah, here we are!")
			while dist > d_th_fast:
				drive(40, 0.5)
				dist = dist_of_token(code,"silver")
			while dist > d_th_slow:
				drive(20, 0.5)
				dist = dist_of_token(code,"silver")
			print("Found it!")
			R.grab()
			print("Gotcha!")
			silver_boxes.append(code)
			for i in range(len(silver_boxes)):
				print(" I have the ", silver_boxes[i]," box! \n")
		elif rot_y < -a_th:           # if the robot is not well aligned with the token, we move it on the left or on the right
			print("Left a bit...")
			turn(-1, 0.5)
			dist_temp, rot_y_temp, code = find_token_silver()
			goto_and_grab_silver(dist_temp, rot_y_temp, code)
		elif rot_y > a_th:
			print("Right a bit...")
			turn(+1, 0.5)
			dist_temp, rot_y_temp, code = find_token_silver()
			goto_and_grab_silver(dist_temp, rot_y_temp, code)

def goto_and_put_gold(dist, rot_y, code):  # Function for reach and put the golden token
	if dist < d_th_2: 
		print("Found it!")
		R.release()                      # if we are close to the token, we grab it.
		print("Gotcha!")
		gold_boxes.append(code)
		for i in range(len(gold_boxes)):
			print(" I have put on the ", gold_boxes[i]," box! \n")
	else:
		if -a_th <= rot_y <= a_th:    # if the robot is well aligned with the token, we go forward
			print("Ah, here we are!")
			while dist > d_th_fast:
				drive(40, 0.5)
				dist = dist_of_token(code,"gold")
			while dist > d_th_2:
				drive(20, 0.5)
				dist = dist_of_token(code,"gold")
			print("Found it!")
			R.release()
			print("Gotcha!")
			gold_boxes.append(code)
			for i in range(len(gold_boxes)):
				print(" I have put on the ", gold_boxes[i]," box! \n")
		elif rot_y < -a_th:           # if the robot is not well aligned with the token, we move it on the left or on the right
			print("Left a bit...")
			turn(-1, 0.5)
			dist_temp, rot_y_temp, code = find_token_gold()
			goto_and_put_gold(dist_temp, rot_y_temp, code)
		elif rot_y > a_th:
			print("Right a bit...")
			turn(+1, 0.5)
			dist_temp, rot_y_temp, code = find_token_gold()
			goto_and_put_gold(dist_temp, rot_y_temp, code)
	
def check_if_inside(code, color):  # Function for checking if the token is already taken (it specifies the color of the token because we have token with different color but same code id)
	if color == "silver":
		if len(silver_boxes) == 0:
			return True
		for i in range(len(silver_boxes)):
			if silver_boxes[i] == code or code == -1:
				return False       # return false if the marker is already taken/put
		return True                # return true if the marker is not already taken/put, or if it's the first token beeing taken/put
	else:
		if len(gold_boxes) == 0:
			return True
		for i in range(len(gold_boxes)):
			if gold_boxes[i] == code or code == -1:
				return False
		return True
	
def random_turn_and_search_s():  # Function for random turning and driving: after that, it calls the "find_token_silver()" function
	turn(10,0.4)
	dist, rot_y, code = find_token_silver()
	if dist!=-1:
		return dist, rot_y, code
	turn(-20,0.4)
	dist, rot_y, code = find_token_silver()
	if dist!=-1:
		return dist, rot_y, code
	turn(10,0.4)
	drive(10,0.4)
	return -1, -1, -1

def random_turn_and_search_g():  # Function for random turning and driving: after that, it calls the "find_token_gold()" function
	turn(10,0.4)
	dist, rot_y, code = find_token_gold()
	if dist!=-1:
		return dist, rot_y, code
	turn(-20,0.4)
	dist, rot_y, code = find_token_gold()
	if dist!=-1:
		return dist, rot_y, code
	turn(10,0.4)
	drive(10,0.4)
	random_turn_and_search_g()

def golden(dist, rot_y, code):  # Function that combine all the functions relates to the golden token..
	goto_and_grab_silver(dist, rot_y, code)
	dist, rot_y, code = find_token_gold()
	if dist==-1:
		dist, rot_y, code = random_turn_and_search_g()
	if check_if_inside(code,"gold") == True:
		goto_and_put_gold(dist, rot_y, code)
		drive(-10,1.5)
		turn(15,3)
	else:
		dist = -1
		while dist == -1:
			dist, rot_y, code = random_turn_and_search_g()
			if check_if_inside(code,"gold") == True:
				goto_and_put_gold(dist, rot_y, code)
				drive(-10,1.5)
				turn(15,3)
			else:
				dist = -1

while i==1:
	cont = 0 
	dist, rot_y, code = find_token_silver()
	while dist ==-1:
		dist, rot_y, code = random_turn_and_search_s()
		if cont >= 2:
			turn(8,3)
		cont = cont + 1
	if check_if_inside(code,"silver") == True:
		golden(dist, rot_y, code)
	else:
		drive(10,1)
		dist = -1
		while dist ==-1:
			dist, rot_y, code = random_turn_and_search_s()
			if check_if_inside(code,"silver") == True:
				golden(dist, rot_y, code)
			if cont >= 2:
				turn(8,3)
			cont = cont + 1
	if len(silver_boxes)==6 and len(gold_boxes)==6:
		print("\n\n Goodbye!\n\n")
		drive(10,2)
		i = 0
pass
