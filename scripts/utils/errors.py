class PointcloudException(Exception):
	''' raised when no valid pointcloud is received '''
	pass

class ArucoException(Exception):
	''' raised when markers are required, but it is not possible to spot any of them '''
	pass 

class PlanningException(Exception):
	''' raised if MoveIt cannot find a plan to bring the eef to the desired position '''
	pass
