import math
import sys

# Covariance Matrix Calculator

def sub_then_square(li, mean):
    output = []
    for x in li:
         output.append(math.pow((x - mean), 2))
    return output 

def sub_them_multiply(xli, yli, xm, ym):
    output = []
    for x in range(10):
        output.append((xli[x]-xm)*(yli[x]-ym))
    return output


# Add x and y coordinates of robot final position to arrays in centimeters (cm).
# There should be 10 of each
xcos = [-0.05, 0.12, 0.2, 0.3, -0.45, 0, -0.2, -0.3, 0.05, 0]
ycos = [-0.5, -0.9, -1.05, -1.0, 0.35, -0.3, -0.2, -0.3, -0.3, -0.3]

if not len(xcos) == len(ycos):
    print "Array lengths are different!!!"
    sys.exit()

xmean = sum(xcos)/10
ymean = sum(ycos)/10

matrix = [[0,0],[0,0]]

matrix[0][0] = sum(sub_then_square(xcos, xmean))/10
matrix[0][1] = sum(sub_them_multiply(xcos,ycos,xmean,ymean))/10
matrix[1][0] = sum(sub_them_multiply(xcos,ycos,xmean,ymean))/10
matrix[1][1] = sum(sub_then_square(ycos, ymean))/10

print str(round(matrix[0][0],4)) + "\t" + str(round(matrix[0][1],4))
print str(round(matrix[1][0],4)) + "\t" + str(round(matrix[1][1],4))   


