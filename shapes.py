import math


class Shape:
    def __init__(self, pointCoors, pathTriples):
        self.points = []
        self.pathing = []
        self.radius = 0
        
        for i in range(len(pointCoors)/2):
            self.points.append((pointCoors[2*i], pointCoors[(2*i) + 1]))

        for j in range(len(pathTriples)/3):
            self.pathing.append((pathTriples[3*j], pathTriples[3*j + 1], pathTriples[3*j + 2]))

        if len(self.points) == 1:
            self.radius = 0
            self.center = self.points[0]
        elif len(self.points == 2):
            self.radius = math.sqrt(math.pow(math.abs(self.points[0][0] - self.points[1][0]), 2) + math.pow(math.abs(self.points[0][1] - self.points[1][1]), 2))/2
            self.center = Point((self.points[0][0] + self.points[1][0])/2, (self.points[0][1] + self.points[1][1])/2)

        self.currentPathIndex = 0
        self.currentTime = 0
        


    def getRadius(self, startPoint, currentLargest):
        maxDist = currentLargest
        maxIndex = -1
        for i in range(len(self.points)):
            distance = math.sqrt(math.pow(math.abs(startPoint.x - self.points[i][0]), 2) + math.pow(math.abs(startPoint.y - self.points[i][1]), 2))
            if distance >= maxDist:
                maxDist = distance
                maxIndex = i

        if maxDist == currentLargest:
            self.radius = maxDist/2
            return Point((startPoint.x + self.points[maxIndex].x)/2, (startPoint.y + self.points[maxIndex].y)/2)
        else:
            self.getRadius(self.points[maxIndex], maxDist)
            
            





class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def isBetweenX(self, point1, point2):
        if self.x >= point1.x and self.x <= point2.x:
            return True
        else:
            return False
        
        
    def isBetweenY(self, point1, point2):
        if self.y >= math.min(point1.y, point2.y) and self.y <= math.max(point1.y, point2.y):
            return True
        else:
            return False
        
    
        

class Edge:
    #Always assume point1.x <= point2.x
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2

        run = point2.x - point1.x
        rise = point2.y - point1.y
        
        if(run == 0 and rise != 0):
            self.slope = math.inf
            self.yInt = None
        else:
            self.slope = rise/run
            #y intercept used for finding if a point is on the line
            self.yInt = point1.y - (point1.x * self.slope)
        
        
    #HAS EDGE CASES THAT DON'T WORK, USE y = mx + b
    def isIntersecting(self, candEdge):
        if (self.slope == candEdge.slope and self.yInt != None):
            if self.yInt == candEdge.yInt:
                if (self.point1.isBetweenX(candEdge.point1, candEdge.point2) or self.point2.isBetweenX(candEdge.point1, candEdge.point2) or candEdge.point1.isBetweenX(self.point1, self.point2) or candEdge.point2.isBetweenX(self.point1, self.point2)):
                    return True #Same line, overlaps, intersects
                else: 
                    return False #Same line, no overlap, does not intersect
            else:
                return False #Parallel, not identical, never intersects
        else:
            if(self.yInt != None and candEdge.yInt != None):
                #When m1x + b1 = m2x + b2, x = (b2 - b1)/(m1-m2)
                intersectX = (candEdge.yInt - self.yInt)/(self.slope-candEdge.slope)
                if self.inBounds(intersectX) and candEdge.inBounds(intersectX):
                    return True
                else: 
                    return False
                
            #In these cases, there is only one possible x value a vertical line can intersect.
            else:
                if(self.yInt == None):
                    intersectX = self.point1.x
                else:
                    intersectX = candEdge.point1.x
            
                if self.inBounds(intersectX) and candEdge.inBounds(intersectX):
                    if self.point1.isBetweenY(candEdge.point1, candEdge.point2) or self.point2.isBetweenY(candEdge.point1, candEdge.point2) or candEdge.point1.isBetweenY(self.point1, self.point2) or candEdge.point2.isBetweenY(self.point1, self.point2):
                        return True
                    return False
                else:
                    return False

                
        
    def isOnLine(self, candPoint):
        if((self.slope * candPoint.x) + self.yInt == candPoint.y):
            return True
        else:
            return False
        
    def inBoundsX(self, someX):
        if someX >= self.point1.x and someX <= self.point2.x:
            return True
        else:
            return False



