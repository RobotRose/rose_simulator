#!/usr/bin/env python

from openravepy import *
from numpy import *
import time
from openravepy.misc import InitOpenRAVELogging

def run():

	InitOpenRAVELogging() 
	RaveSetDebugLevel(DebugLevel.Verbose) 
	env = Environment() # create the environment
	env.SetViewer('qtcoin') # start the viewer
	env.Load('robai.xml') # load a scene
	robot = env.GetRobots()[0] # get the first robot
	manip = robot.SetActiveManipulator('arm') # set the manipulator to leftarm
	
	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D, freejoints=['link1'])

	if not ikmodel.load():
		ikmodel.autogenerate()

	with env: # lock environment
		
		Tgoal = numpy.array([[ 0., 0.,  -1., -.31554],[0.,  1., 0., -0.,-.027],   [1. ,  0. ,  0. , 0.318488],[ 0.,0.,0. ,1.]])



		sol = manip.FindIKSolution(Tgoal,IkFilterOptions.IgnoreSelfCollisions,ikreturn=True) 
		with robot: # save robot state
		    robot.SetActiveDOFs(manip.GetArmIndices())
		    robot.SetActiveDOFValues(sol.GetSolution())
		    Tee = manip.GetEndEffectorTransform()
		    env.UpdatePublishedBodies() # allow viewer to update new robot
		    time.sleep(10)
		
		raveLogInfo('Tee is: '+repr(Tee))



	##The following commands will move a robot to a desired position with the joint angles
	robot=env.GetRobots()[0]
	manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
	manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.58]) # call motion planner with goal joint angles
	robot.WaitForController(0) # wait


 
	
if __name__ == '__main__':
        try:
                run()
        finally:
				raw_input('press enter to quit')
				RaveDestroy()
