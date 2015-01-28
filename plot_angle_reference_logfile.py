import sys
import matplotlib.pyplot as plt

logfile_path = sys.argv[1]

with open(logfile_path) as f:
	content = f.readlines()

	time = []

	ref = []

	angle1 = []
	angle2 = []	

	error1 = []
	error2 = []
	
	

	for i in range(0,len(content),5):
		line = content[i]
		line_split = line.split()

		# Timestamp
		time.append(line_split[0])

		# Reference angle
		reference_angle = line_split[1]
		ref.append(reference_angle)

		# Actual angle for first motor and angle error
		motor1_angle = line_split[2]
		angle1.append(motor1_angle)
		error1.append(abs(float(reference_angle) - float(motor1_angle)))
	
		# Actual angle for second motor and angle error
		motor2_angle = line_split[4]
		angle2.append(motor2_angle)
		error2.append(abs(float(reference_angle) - float(motor2_angle)))
	
		if i == 0:
			print "Initial reference = " + reference_angle
			print "Initial motor1 angle = " + motor1_angle
			print "Initial motor2 angle = " + motor2_angle	
	
	
	fig,ax = plt.subplots()
	ax.plot(time,ref, label='Reference angle')	
	ax.plot(time,angle1, label='MOTOR1: actual angle')	
	ax.plot(time,angle2, label='MOTOR2: actual angle')	
	ax.plot(time,error1, label='MOTOR1: angle error')	
	ax.plot(time,error2, label='MOTOR2: angle error')	
	
	ax.legend(loc='upper left')
	
	plt.xlabel('Time in seconds')
	plt.ylabel('Angle')
	plt.show() 	
