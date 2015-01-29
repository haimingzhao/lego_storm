import sys
import matplotlib.pyplot as plt

logfile_path = sys.argv[1]

with open(logfile_path) as f:
	content = f.readlines()

	time = []

	angle1 = []
	ref1 = []

	angle2 = []	
	ref2 = []

	error1 = []
	error2 = []
	
	
	# Get initial reference and motor angles
	ref1_initial = float(content[0].split()[1])
	motor1_initial = float(content[0].split()[2])
	ref2_initial = float(content[0].split()[3])
	motor2_initial = float(content[0].split()[4])
	
	print "Initial reference angle motor1 = " 
	print ref1_initial
	print "Initial motor1 angle = " 
	print motor1_initial
	print "Initial reference angle motor2 = " 
	print ref2_initial
	print "Initial motor2 angle = " 
	print motor2_initial


	for i in range(0,len(content),1):
		line = content[i]
		line_split = line.split()

		# Timestamp
		time.append(line_split[0])

		# Reference angle motor1
		reference_angle1 = float(line_split[1]) - ref1_initial
		ref1.append(reference_angle1)

		# Actual angle for first motor and angle error
		motor1_angle = float(line_split[2]) - motor1_initial
		angle1.append(motor1_angle)
		error1.append(reference_angle1 - motor1_angle)
	
		# Reference angle motor2
		reference_angle2 = float(line_split[3]) - ref2_initial
		ref2.append(reference_angle2)

		# Actual angle for second motor and angle error
		motor2_angle = float(line_split[4]) - motor2_initial
		angle2.append(motor2_angle)
		error2.append(reference_angle2 - motor2_angle)
	
	
	fig,ax = plt.subplots()
	ax.plot(time,angle1, label='MOTOR1: actual angle')	
	ax.plot(time,ref1, label='MOTOR1: reference angle')	
	ax.plot(time,angle2, label='MOTOR2: actual angle')	
	ax.plot(time,ref1, label='MOTOR2: reference angle')	
	ax.plot(time,error1, label='MOTOR1: angle error')	
	ax.plot(time,error2, label='MOTOR2: angle error')	
	
	ax.legend(loc='upper left')
	
	plt.xlabel('Time in seconds')
	plt.ylabel('Angle')
	plt.show() 	
