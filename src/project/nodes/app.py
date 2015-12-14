#!/usr/bin/env python
import tf
import rospy
import sys
import time
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf
import path_planning as pp
import atexit

def setUp():

	rospy.init_node('followTag')
	atexit.register(exit_handler)

	global originTag
	global zumy1_name
	global zumy2_name
	global zumy1Tag
	global zumy2Tag
	global obstaclesARTagsWithScanARTags
	global listener
	global zumy1_vel
	global zumy2_vel
	global gridXDim
	global gridYDim
	global unit
	global offset
	global error
	global rate
	global ar_tags_grid_location
	global prevAbsoluteTheta
	global lastCommand
	global targetFound
	global scanning
	global currentScanTag
	global rightTurn
	global leftTurn
	global forward
	global idle

	originTag = 'ar_marker_8'
	zumy1_name = 'zumy1b'
	zumy2_name = 'zumy5a'
	zumy1Tag = 'ar_marker_4'
	zumy2Tag = 'ar_marker_5'
	# obstaclesARTagsWithScanARTags = [('ar_marker_7','ar_marker_15',0), ('ar_marker_0','ar_marker_5',0), ('ar_marker_10','ar_marker_11',0)]
	obstaclesARTagsWithScanARTags = [('ar_marker_7','ar_marker_15',0), ('ar_marker_0','ar_marker_5',0)]
	listener = tf.TransformListener()
	zumy1_vel = rospy.Publisher('%s/cmd_vel' % zumy1_name, Twist, queue_size=1)
	zumy2_vel = rospy.Publisher('%s/cmd_vel' % zumy2_name, Twist, queue_size=1)
	ar_tags = {}
	rospy.Subscriber('/tf', TFMessage, callback, ar_tags)

	# STATIC VARIABLES
	linSpeed = 0.10
	rotSpeed = 0.18

	gridXDim = 20
	gridYDim = 20
	unit = 0.05 # In meters
	offset = unit/2.0
	error = 0.6 # Acceptable error in radians
	rate = rospy.Rate(2) # In hertz

	# GLOBAL VARIABLES
	ar_tags_grid_location = [(0,0), (0,0), (0,0)] # Origin, zumy1, zumy2
	# Add all obstacles
	for i in range(0, len(obstaclesARTagsWithScanARTags)):
		ar_tags_grid_location.append((0,0))

	prevAbsoluteTheta = 0
	targetFound = False
	scanning = False
	currentScanTag = 0

	# Twists
	rightTurn = Twist()
	rightTurn.linear.x = 0
	rightTurn.linear.y = 0
	rightTurn.linear.z = 0
	rightTurn.angular.x = 0
	rightTurn.angular.y = 0
	rightTurn.angular.z = -rotSpeed

	leftTurn = Twist()
	leftTurn.linear.x = 0
	leftTurn.linear.y = 0
	leftTurn.linear.z = 0
	leftTurn.angular.x = 0
	leftTurn.angular.y = 0
	leftTurn.angular.z = rotSpeed

	forward = Twist()
	forward.linear.x = linSpeed
	forward.linear.y = 0
	forward.linear.z = 0
	forward.angular.x = 0
	forward.angular.y = 0
	forward.angular.z = 0

	idle = Twist()
	idle.linear.x = 0
	idle.linear.y = 0
	idle.linear.z = 0
	idle.angular.x = 0
	idle.angular.y = 0
	idle.angular.z = 0

	lastCommand = idle


def getMap():
	print 'Getting map'



	global rate

	while not rospy.is_shutdown():

		print 'originTag'
		print originTag

		print 'Locations'
		print ar_tags_grid_location

		# Find position of Zumy 1
		objectName = zumy1Tag
		if ar_tags_grid_location[1] == (0,0):
			ar_tags_grid_location[1] = getGridPosition(objectName)

		# # Find position of Zumy 2
		objectName = zumy2Tag
		if ar_tags_grid_location[2] == (0,0):
			ar_tags_grid_location[2] = getGridPosition(objectName)

		# Find position of Obstacles
		for i in range(0, len(obstaclesARTagsWithScanARTags)):
			objectName = obstaclesARTagsWithScanARTags[i][0]
			if ar_tags_grid_location[i+3] == (0,0):
				ar_tags_grid_location[i+3] = getGridPosition(objectName)


		gotAllInfo = True

		for i in range(0, len(obstaclesARTagsWithScanARTags)):
			if (ar_tags_grid_location[3+i] == (0,0)):
				gotAllInfo = False

		if (ar_tags_grid_location[1] == (0,0)) or (ar_tags_grid_location[2] == (0,0)):
			gotAllInfo = False

		if gotAllInfo == True:
			print 'GOT ALL THE INFO!'
			break
		else:
			print 'MISSING INFO!'
			time.sleep(1)

		print 'Locations'
		print ar_tags_grid_location

def getGridPosition(objectName):

	gridX = 0
	gridY = 0
	trans = (0,0)
	try:
		(trans, rot) = listener.lookupTransform(originTag, objectName, rospy.Time(0))
	except:
		print 'Excepting, we cant see: '  + objectName
		e = sys.exc_info()[0]
		e2 = sys.exc_info()[1]
		print "<p>Error: %s</p>" % e
		print "<p>Error: %s</p>" % e2
		
	print 'X-Y from origin of: ' + objectName
	print trans[0], trans[1]
	gridX = max(math.floor(trans[0]/unit), 0)
	gridY = max(math.floor(trans[1]/unit), 0)
	return (gridX, gridY)




def getAbsoluteThetaAbsoluteLocationAndTrans(zumy_tag, transform, prevAbsoluteTheta):

	trans = transform[0]
	rot = transform[1]

	# GET ZUMY position on grid
	gridX = max(math.floor(trans[0]/unit), 0)
	gridY = max(math.floor(trans[1]/unit), 0)
	# print 'Zumy 1 is at:'
	# print gridX, gridY

	absoluteTheta1 = eqf.quaternion_to_exp(rot)[1]
	absoluteTheta2 = -absoluteTheta1 + 2* math.pi

	print "1. absoluteTheta1"
	print absoluteTheta1
	print "2. absoluteTheta2"
	print absoluteTheta2
	print "3. prevAbsoluteTheta"
	print prevAbsoluteTheta

	absoluteTheta = 0

	absoluteThetaDifference1 = abs(prevAbsoluteTheta - absoluteTheta1)
	absoluteThetaDifference2 = abs(prevAbsoluteTheta - absoluteTheta2)
	thetaDifference = abs(absoluteTheta1-absoluteTheta2)

	closestTheta = 0
	closestThetaDifference = 0
	if absoluteThetaDifference1 < absoluteThetaDifference2:
		closestTheta = absoluteTheta1
		closestThetaDifference = absoluteThetaDifference1
	if absoluteThetaDifference1 > absoluteThetaDifference2:
		closestTheta = absoluteTheta2
		closestThetaDifference = absoluteThetaDifference2
	print "4. closestTheta"
	print closestTheta

	precision = 0.5

	if lastCommand == rightTurn:
		print 'LAST COMMAND WAS RIGHT'
		if absoluteTheta1 > 2*math.pi - precision or absoluteTheta1 < precision:
			absoluteTheta = 0
		elif prevAbsoluteTheta == 0:
			absoluteTheta = absoluteTheta2
		elif closestThetaDifference < 0.3:
			absoluteTheta = prevAbsoluteTheta
		elif prevAbsoluteTheta - closestTheta > 0:
			print 'CHOOSING THE CLOSEST'
			absoluteTheta = closestTheta
			prevAbsoluteTheta = closestTheta
		else:
			print 'CHOOSING ABSOLUTETHETA1'
			absoluteTheta = absoluteTheta1
	elif lastCommand == leftTurn:
		print 'LAST COMMAND WAS LEFT'
		if absoluteTheta1 > 2*math.pi - precision or absoluteTheta1 < precision:
			absoluteTheta = 0
		elif prevAbsoluteTheta == 0:
			absoluteTheta = absoluteTheta1
		elif closestThetaDifference < 0.5:
			absoluteTheta = prevAbsoluteTheta
		elif prevAbsoluteTheta - closestTheta < 0:
			print 'CHOOSING THE CLOSEST'
			absoluteTheta = closestTheta
			prevAbsoluteTheta = closestTheta
		else:
			print 'CHOOSING ABSOLUTETHETA1'
			absoluteTheta = absoluteTheta1
	elif lastCommand == forward:
		print 'LAST COMMAND WAS FORWARD'
		if absoluteThetaDifference2 < absoluteThetaDifference1:
			absoluteTheta = absoluteTheta2
		else:
			absoluteTheta = absoluteTheta1
	elif lastCommand == idle:
		print 'LAST COMMAND WAS IDLE'
	else:
		print 'ERROR, LAST COMMAND NOT RECOGNIZED, SLEEPING FOR 10 SECONDS'
		# time.sleep(10)


	print "5. CHOSEN absoluteTheta: "
	print absoluteTheta


	return absoluteTheta, (gridX, gridY), trans

def getInnerThetaAndAbsoluteLocation(targetPoint, zumy_tag):


	# Get transform and absoluteTheta

	global prevAbsoluteTheta
	transform = listener.lookupTransform(originTag, zumy_tag, rospy.Time(0))
	absoluteThetaAndAbsoluteLocation = getAbsoluteThetaAbsoluteLocationAndTrans(zumy_tag, transform, prevAbsoluteTheta)
	absoluteTheta = absoluteThetaAndAbsoluteLocation[0]
	prevAbsoluteTheta = absoluteTheta
	print 'absoluteTheta IS: ' + str(absoluteTheta)
	# print "Zumy absoluteTheta"
	# print absoluteTheta
	trans = absoluteThetaAndAbsoluteLocation[2]
	currentPoint = trans

	# Get vectors

	orientationVector = getOrientationVector(absoluteTheta)
	# print 'orientationVector'
	# print orientationVector
	orientationVectorUnit = getUnitVector(orientationVector)
	# print 'orientationVectorUnit'
	# print orientationVectorUnit
	pointVector = getPointVector(targetPoint=targetPoint, currentPoint=currentPoint)
	# print 'pointVector'
	# print pointVector
	pointVectorUnit = getUnitVector(pointVector)
	# print 'pointVectorUnit'
	# print pointVectorUnit

	# Get innerTheta from vectors

	innerTheta = getAngleBetweenVectors(orientationVectorUnit,pointVectorUnit)

	# Return
	return innerTheta, (absoluteThetaAndAbsoluteLocation[1][0], absoluteThetaAndAbsoluteLocation[1][1])


def getInnerTheta(absoluteTheta, targetPoint, currentPoint):
	orientationVector = getOrientationVector(absoluteTheta)
	# print 'orientationVector'
	# print orientationVector
	orientationVectorUnit = getUnitVector(orientationVector)
	# print 'orientationVectorUnit'
	# print orientationVectorUnit

	pointVector = getPointVector(targetPoint=targetPoint, currentPoint=currentPoint)
	# print 'pointVector'
	# print pointVector
	pointVectorUnit = getUnitVector(pointVector)
	# print 'pointVectorUnit'
	# print pointVectorUnit
	innerTheta = getAngleBetweenVectors(orientationVectorUnit,pointVectorUnit)
	return innerTheta

def getUnitVector(vector):
	vectorNorm = math.sqrt(vector[0]**2 + vector[1]**2)
	unitVector = (vector[0]/vectorNorm, vector[1]/vectorNorm)
	return unitVector


def getOrientationVector(theta):
	orientationVector = (math.cos(theta), math.sin(theta))
	return orientationVector

def getPointVector(targetPoint, currentPoint):
	newPointVector = [targetPoint[0] - currentPoint[0], targetPoint[1] - currentPoint[1]]
	return newPointVector

def getAngleBetweenVectors(u, v):

	#artan of u
	uArtan = math.atan2(u[1],u[0])
	# print uArtan

	#artan of v
	vArtan = math.atan2(v[1],v[0])
	# print vArtan

	artanSub = vArtan - uArtan

	if artanSub < 0:
		artanSub = artanSub + 2*math.pi

	theta = artanSub

	return theta

def getCommandFromTheta(theta):
	global error

	# CHECK IF THETA IS MORE THAN 0 AND LESS THAN 2PI
	if theta > 2*math.pi:
		print 'ERROR, newInnerTheta bigger than 2*pi'
	elif theta < 0:
		print 'ERROR, newInnerTheta smaller than 0'

	command = Twist()
	if theta > error and theta < math.pi:
		print 'COMMAND FROM THETA SAYS: TURN LEFT'
		command = leftTurn
	elif theta > math.pi and theta < 2*math.pi - error :
		print 'COMMAND FROM THETA SAYS: TURN RIGHT'
		command = rightTurn
	elif theta < error or theta > 2*math.pi - error:
		print 'COMMAND FROM THETA SAYS: GO FORWARD'
		command = forward
	return command



def follow_point(zumy_name, targetPoint, scanARTAG):

	print 'LETS MOVE ' + str(zumy_name) + ' TO ' + str(targetPoint) + ' with scanARTAG ' + str(scanARTAG)
	time.sleep(0.5)

	# GET ZUMY TAG
	zumy_tag = ''
	if zumy_name == zumy1_name:
		zumy_tag = zumy1Tag
	elif zumy_name == zumy2_name:
		zumy_tag = zumy2Tag

	global prevAbsoluteTheta
	global lastCommand
	global offset
	targetPointIn3D = [unit*targetPoint[0] + offset,unit*targetPoint[1]+ offset]


	while not rospy.is_shutdown():


		# GET NEW INNER THETA AND ABSOLUTE LOCATION

		# InnerTheta
		innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=targetPointIn3D, zumy_tag=zumy_tag)
		innerTheta = innerThetaAndAbsoluteLocation[0]
		prevInnerTheta = innerTheta
		print 'INNERTHETA IS: ' + str(innerTheta)

		# Absolute location
		print 'We Are at'
		print innerThetaAndAbsoluteLocation[1][0], innerThetaAndAbsoluteLocation[1][1] 
		print 'We want to go to'
		print targetPoint[0], targetPoint[1]

		# Break if we arrived
		if innerThetaAndAbsoluteLocation[1][0] == targetPoint[0] and innerThetaAndAbsoluteLocation[1][1] == targetPoint[1]:
			break
		else:
			# Get command
			command = getCommandFromTheta(innerTheta)

			# Execute command
			sendCommandToZumy(zumy_name, command)


			
	print 'ZUMY ARRIVED TO: ' + str(targetPoint)

	# Scan if there is an SCANARTAG
	if scanARTAG != 0:
		scanPoint(zumy_name, innerTheta, targetPoint, scanARTAG)

def sendCommandToZumy(zumy_name, command):
	global lastCommand


	zumy_vel = 0

	if zumy_name == zumy1_name:
		zumy_vel = zumy1_vel
	elif zumy_name == zumy2_name:
		zumy_vel = zumy2_vel

	print zumy_name

	if command == forward:
		print 'GOING FORWARD'
	elif command == leftTurn:
		print 'GOING LEFT'
	elif command == rightTurn:
		print 'GOING RIGHT'
	elif command == idle:
		print 'STOPING'

	zumy_vel.publish(command)
	lastCommand = command
	rate.sleep()

def scanPoint(zumy_name, innerTheta, targetPoint, scanARTAG):
	print 'About to scan...'
	# time.sleep(3)
	global scanning
	scanning = True
	print 'Setting scanning as True'
	global rate

	global targetFound
	global prevAbsoluteTheta

	targetPointIn3D = [unit*targetPoint[0] + offset,unit*targetPoint[1]+ offset]
	scanPointIn3D = [unit*(targetPoint[0] + 2) + offset ,unit*targetPoint[1]+ offset]

	# GET ZUMY TAG
	zumy_tag = ''
	if zumy_name == zumy1_name:
		zumy_tag = zumy1Tag
	elif zumy_name == zumy2_name:
		zumy_tag = zumy2Tag

	# Turn 360 degrees or until target found

	# InnerTheta
	innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=scanPointIn3D, zumy_tag=zumy_tag)
	innerTheta = innerThetaAndAbsoluteLocation[0]
	prevInnerTheta = innerTheta
	print 'INNERTHETA IS: ' + str(innerTheta)
	
	initialCommand = getCommandFromTheta(innerTheta)
	followingCommand = initialCommand
	nextCommand = Twist()

	while (not targetFound):
		try:
			while followingCommand != forward:
				print 'Turnig until facing front'
				time.sleep(0.5)

				nextCommand = followingCommand

				# Execute command
				sendCommandToZumy(zumy_name, followingCommand)

				# InnerTheta
				innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=scanPointIn3D, zumy_tag=zumy_tag)
				innerTheta = innerThetaAndAbsoluteLocation[0]
				prevInnerTheta = innerTheta
				# print 'INNERTHETA IS: ' + str(innerTheta)

				# Get command
				followingCommand = getCommandFromTheta(innerTheta)
				


			print 'Done with first while, facing front!'

			# time.sleep(2)

			if nextCommand == Twist():

				while followingCommand == forward:
					print 'Keep turning Left until not facing front anymore'
					time.sleep(0.5)

					# Execute command
					sendCommandToZumy(zumy_name, leftTurn)

					# InnerTheta
					innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=scanPointIn3D, zumy_tag=zumy_tag)
					innerTheta = innerThetaAndAbsoluteLocation[0]
					prevInnerTheta = innerTheta
					# print 'INNERTHETA IS: ' + str(innerTheta)

					# Get command
					followingCommand = getCommandFromTheta(innerTheta)


				while followingCommand != forward:
					print 'Keep turning right until facing front again'
					time.sleep(0.5)

					# Execute command
					sendCommandToZumy(zumy_name, rightTurn)

					# InnerTheta
					innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=scanPointIn3D, zumy_tag=zumy_tag)
					innerTheta = innerThetaAndAbsoluteLocation[0]
					prevInnerTheta = innerTheta
					# print 'INNERTHETA IS: ' + str(innerTheta)

					# Get command
					followingCommand = getCommandFromTheta(innerTheta)

				nextCommand = rightTurn


			while followingCommand == forward:
				print 'Keep turning until not facing front anymore'
				time.sleep(0.5)

				# Execute command
				sendCommandToZumy(zumy_name, nextCommand)

				# InnerTheta
				innerThetaAndAbsoluteLocation = getInnerThetaAndAbsoluteLocation(targetPoint=scanPointIn3D, zumy_tag=zumy_tag)
				innerTheta = innerThetaAndAbsoluteLocation[0]
				prevInnerTheta = innerTheta
				# print 'INNERTHETA IS: ' + str(innerTheta)

				# Get command
				followingCommand = getCommandFromTheta(innerTheta)
				lastCommand = followingCommand



			print 'Done Turning, not facing front anymore'
			break

		except:
			e = sys.exc_info()[0]
			e2 = sys.exc_info()[1]
			print "<p>Error: %s</p>" % e
			print "<p>Error: %s</p>" % e2
		
	sendCommandToZumy(zumy_name, idle)
	print 'Scan ended, did we find the target!?!?!?!?!?!!??'
	print targetFound
	scanning = False
	targetFound = False


def callback(msg, ar_tags):
	# print 'IN CALLBACK'
	for i in range(0, len(msg.transforms)):
		#print msg.transforms[i].child_frame_id
		if msg.transforms[i].header.frame_id == zumy1_name + "_cam":
			print 'ZUMY CAM!'
			global scanning
			global currentScanTag
			if scanning == True:
				print 'Scanning true'

				if msg.transforms[i].child_frame_id == currentScanTag:
					print "Found target"
					global targetFound
					targetFound = True
					for i in range(0,len(obstaclesARTagsWithScanARTags)):
						if obstaclesARTagsWithScanARTags[i][1] == currentScanTag:
							obstaclesARTagsWithScanARTags[i][2] = 2
							# time.sleep(3)
				else:
					print "Not target"
					print 'target is: ' + currentScanTag

			else:
				print 'Scanning false'
		# elif msg.transforms[i].header.frame_id == 'usb_cam':
		# 	print 'USB CAM!'


def generateZumyPaths(obstaclePointsWithScanTags, zumyLocation, zumy_name):

	obstaclePoints = []

	# Get obstacles
	for i in range(0,len(obstaclePointsWithScanTags)):
		obstaclePoints.append(obstaclePointsWithScanTags[i][0])

	# Get paths
	zumyPaths = []
	start = zumyLocation
	for i in range(0,len(obstaclePointsWithScanTags)):
		end = (obstaclePointsWithScanTags[i][0][0] - 3, obstaclePointsWithScanTags[i][0][1])
		zumyPath = pp.findPath(gridXDim, gridYDim, start, end, obstaclePoints)
		zumyPaths.append(zumyPath)
		start = end
	end = zumyLocation
	zumyPath = pp.findPath(gridXDim, gridYDim, start, end, obstaclePoints)
	zumyPaths.append(zumyPath)

	# Generate paths with ar tags
	zumyPathsWithscanTags = []
	for i in range(0,len(zumyPaths)):
		scanTag = ''
		if i == len(zumyPaths) - 1:
			scanTag = 0
		else:
			scanTag = obstaclePointsWithScanTags[i][1]
		zumyPathsWithscanTags.append((zumyPaths[i],scanTag, zumy_name))

	# Return
	return zumyPathsWithscanTags


def executePaths(zumyPaths):

	global currentScanTag

	for i in range(0, len(zumyPaths)):
		print 'WE ARE ON THE PATH: ' + str(i + 1)
		print 'OUT OF: ' + str(len(zumyPaths))
		# time.sleep(3)
		path = zumyPaths[i][0]
		for j in range(0,len(path)):
			currentPoint = path[j]
			currentScanTag = 0
			if j == len(path) - 1:
				currentScanTag = zumyPaths[i][1] 
			follow_point(zumy_name=zumyPaths[i][2], targetPoint=currentPoint, scanARTAG=currentScanTag)

def exit_handler():
	print 'Ending application!!!'
	sendCommandToZumy(zumy1_name, idle)
	sendCommandToZumy(zumy2_name, idle)


if __name__=='__main__':

	# 0. SetUp
	setUp()
	global zumy1_name

	# 1. Get map
	getMap()

	# 2. Get zumy1 paths
	zumy1ObstaclePointsWithScanTags = []
	for i in range(0,len(obstaclesARTagsWithScanARTags)):
		zumy1ObstaclePointsWithScanTags.append((ar_tags_grid_location[3+i],obstaclesARTagsWithScanARTags[i][1]))
	zumy1Paths = generateZumyPaths(zumy1ObstaclePointsWithScanTags,ar_tags_grid_location[1],zumy1_name)

	# 3. Execute zumy 1 paths
	executePaths(zumy1Paths)

	# 4. Check targets found
	zumy2ObstaclePointsWithScanTags = []
	for i in range(0,len(obstaclesARTagsWithScanARTags)):
		if obstaclesARTagsWithScanARTags[i][2] == 1:
			zumy2ObstaclePointsWithScanTags.append((ar_tags_grid_location[3 + i],0))

	# 5. Get zumy2 paths
	zumy2Paths = generateZumyPaths(zumy2ObstaclePointsWithScanTags,ar_tags_grid_location[2],zumy2_name)


	# 6. Execute zumy 2 paths
	executePaths(zumy2Paths)

	print 'DEMO DONE, THANK YOU'


	# rospy.spin() simply keeps python from exiting until this node is stopped	
	rospy.spin()

	sys.exit()
