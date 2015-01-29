import math

# Covariance Matrix Calculator

def sub_then_square(li, mean):
    output = []
    for x in li:
         output.append(math.pow((x - mean), 2))
    return output 

def sub_them_multiply(xli, yli, xmean, ymean):
    output = []
    for x in range(10):
        output.append((xli[x]-xmean)*(yli[x]-ymean))
    return output


# Add x and y coordinates of robot final position to arrays in centimeters (cm).
# There should be 10 of each
#xcos = []
#ycos = []
xcos = [1,2,3,1,2,3,1,2,3,1]
ycos = [3,2,1,3,2,1,4,3,2,1]

xmean = sum(xcos)/10
ymean = sum(ycos)/10

matrix = [[0,0],[0,0]]

matrix[0][0] = sum(sub_then_square(xcos, xmean))/10
matrix[0][1] = sum(sub_them_multiply(xcos,ycos,xmean,ymean))/10
matrix[1][0] = sum(sub_them_multiply(xcos,ycos,xmean,ymean))/10
matrix[1][1] = sum(sub_then_square(ycos, ymean))/10

print str(matrix[0][0]) + "\t" + str(matrix[0][1])
print str(matrix[1][0]) + "\t" + str(matrix[1][1])   




