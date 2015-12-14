import numpy as np
import sys, logging
from copy import copy, deepcopy
import math as math

penalty = 0.5

def findPath(x,y,start,end, obstaclePoints):

	def prepareObstacleGrid():

		obstacle = 1
		border = 2
		security = 3
		
		# Create obstacles grid
		gridObstacles = np.zeros((x,y))
		for i in range(0, len(obstaclePoints)):
			gridObstacles[obstaclePoints[i][0],obstaclePoints[i][1]] = obstacle
		logging.debug('gridObstacles')
		logging.debug(gridObstacles)

		# Add borders
		gridObstaclesBorders = deepcopy(gridObstacles)
		for i in range(0,x):
			for j in range(0,y):
				if i == 0 or j == 0 or i == x-1 or j == y-1:
					gridObstaclesBorders[i,j] = border
		gridObstacles = gridObstaclesBorders
		logging.debug('gridObstacles with borders')
		logging.debug(gridObstacles)

		# Add security barrier to obstacles
		gridObstaclesSecure = deepcopy(gridObstacles)
		for i in range(0,x):
			for j in range(0,y):
				# print i, j, gridObstacles[i,j]
				if gridObstacles[i,j] == 1:
					if gridObstacles[i+1,j+1] != 1:
						gridObstaclesSecure[i+1,j+1] = security
					if gridObstacles[i,j+1] != 1:
						gridObstaclesSecure[i,j+1] = security
					if gridObstacles[i-1,j+1] != 1:
						gridObstaclesSecure[i-1,j+1] = security
					if gridObstacles[i+1,j] != 1:
						gridObstaclesSecure[i+1,j] = security
					if gridObstacles[i-1,j] != 1:
						gridObstaclesSecure[i-1,j] = security
					if gridObstacles[i+1,j-1] != 1:
						gridObstaclesSecure[i+1,j-1] = security
					if gridObstacles[i,j-1] != 1:
						gridObstaclesSecure[i,j-1] = security
					if gridObstacles[i-1,j-1] != 1:
						gridObstaclesSecure[i-1,j-1] = security			
		gridObstacles = gridObstaclesSecure
		logging.debug('gridObstacles with security and borders')
		logging.debug(gridObstacles)

		# Consolidate obstacles, security zone and borders
		for i in range(0,x):
			for j in range(0,y):
				# print i, j, gridObstacles[i,j]
				if gridObstacles[i,j] != 0:
					gridObstacles[i,j] = obstacle
		logging.debug('gridObstacles with security and borders consolidates')
		logging.debug(gridObstacles)

		# Check if start or end are close to an obstacle
		if gridObstacles[start[0],start[1]] == 1 or gridObstacles[end[0],end[1]] == 1:
			sys.exit("Start or end could be on an obstacle, close to an obstacle or on a border")

		# print gridObstacles

		return gridObstacles


	gridObstacles = prepareObstacleGrid()

	def prepareValueGrid():

		global penalty
		M = math.ceil(math.sqrt(x**2 + y**2)) + 8*penalty + 1
		# M = 1000

		# Great grid with values. Manhattan distance, obstacles and borders
		gridValues = np.zeros((x,y))

		for i in range(0,x):
			for j in range(0,y):
				if gridObstacles[i,j] == 1:
					# print 'Found obstacle'
					gridValues[i,j] = M
				else:
					manhattanX = abs(end[0] - i)
					manhattanY = abs(end[1] - j)
					manhattanTotal = manhattanX + manhattanY
					gridValues[i,j] = manhattanTotal
		logging.debug('gridValues without penalties')
		logging.debug(gridValues)


		# Add obstacle penalty
		for i in range(1,x-1):
			for j in range(1,y-1):
				if gridValues[i,j] != M and not (i==end[0] and j==end[1]):
					if gridValues[i+1,j+1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i,j+1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i-1,j+1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i+1,j] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i-1,j] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i+1,j-1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i,j-1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
					if gridValues[i-1,j-1] == M:
						gridValues[i,j] = gridValues[i,j] + penalty
		logging.debug('gridValues with penalties')
		logging.debug(gridValues)

		# print 'Different gridValues printing format'
		# for i in range(0,x):
		# 	for j in range(0,y):
		# 		print '{:4}'.format(gridValues[i][j]),
		# 	print

		print gridValues
		return gridValues

	gridValues = prepareValueGrid()

	# Find path

	def findPathFromGridValue():

		path = []
		cx = start[0]
		cy = start [1]

		while not (cx == end[0] and cy == end[1]):

			logging.debug('')
			logging.debug('Finding path, we are at')
			logging.debug(cx)
			logging.debug(cy)
			logging.debug('And end is at')
			logging.debug(end[0])
			logging.debug(end[1])

			nextValue = gridValues[cx,cy]
			nextX = cx
			nextY = cy

			if gridValues[cx+1,cy+1] <= nextValue:
				nextValue = gridValues[cx+1,cy+1]
				nextX = cx+1
				nextY = cy+1
				logging.debug('Checking down-right')
			if gridValues[cx,cy+1] <= nextValue:
				nextValue = gridValues[cx,cy+1]
				nextX = cx
				nextY = cy+1
				logging.debug('Checking right')
			if gridValues[cx-1,cy+1] <= nextValue:
				nextValue = gridValues[cx-1,cy+1]
				nextX = cx-1
				nextY = cy+1
				logging.debug('Checking up-right')
			if gridValues[cx+1,cy] <= nextValue:
				nextValue = gridValues[cx+1,cy]
				nextX = cx+1
				nextY = cy
				logging.debug('Checking down')
			if gridValues[cx-1,cy] <= nextValue:
				nextValue = gridValues[cx-1,cy]
				nextX = cx-1
				nextY = cy
				logging.debug('Checking up')
			if gridValues[cx+1,cy-1] <= nextValue:
				nextValue = gridValues[cx+1,cy-1]
				nextX = cx+1
				nextY = cy-1
				logging.debug('Checking down-left')
			if gridValues[cx,cy-1] <= nextValue:
				nextValue = gridValues[cx,cy-1]
				nextX = cx
				nextY = cy-1
				logging.debug('Checking left')
			if gridValues[cx-1,cy-1] <= nextValue:
				nextValue = gridValues[cx-1,cy-1]
				nextX = cx-1
				nextY = cy-1
				logging.debug('Checking up-left')

			if cx == nextX and cy == nextY:
				print 'Did not find path'
				print 'Path was: ' + str(path)
				path = []
				break
			else:
				logging.debug('Next step is')
				logging.debug(nextX)
				logging.debug(nextY)

			cx = nextX
			cy = nextY

			# if cx == end[0]:
			# 	logging.debug('cx = endX')
			# if cy == end[1]:
			# 	logging.debug('cy = endY')

			path.append((cx,cy))

		return path

	path = findPathFromGridValue()

	return path

# MAIN METHOD
if __name__ == '__main__':


	print('MAIN METHOD OF path_planning')

	logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

	print('TESTING')
	x = 15
	y = 15
	start = (12,12)
	end = (1,1)
	obstaclePoints = [(4,3), (5,6)]
	path = findPath(x, y, start, end, obstaclePoints)
	print 'Final path is'
	print path
