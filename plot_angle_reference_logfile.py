import sys
import matplotlib

logfile_path = sys.argv[1]

with open(logfile_path) as f:
	print "lala"
	content = f.readlines()
	
	time = []
	ref1 = []
	angle1 = []
	ref2 = []
	angle2 = []
	

	for i in range(0,len(content),5):
		line = content[i]

		line_split = line.split('\t')			

		time += line_split[0]

		# Reference angle and actual angle for first motor
		ref1 += line_split[1]
		angle1 += line_split[2]
		
		# Reference angle and actual angle for second motor
		ref2 += line_split[3]
		angle2 += line_split[4]
		
	
	matplotlib.pyplot.plot(time,ref1,angle1,ref2,angle2)
	matplotlib.pyplot.show() 	
