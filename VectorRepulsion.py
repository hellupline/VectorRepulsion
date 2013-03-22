from collections import namedtuple
from numpy import zeros, int8
from math import *
import sys
try:
    from AriaPy import *
except:
    pass

stderr = sys.stderr
Point = namedtuple('Point', ['x', 'y', 'th'])

MEASURE_REDUCTION = 200
MAP_SIZE = 500
MAP_SAMPLER = 1
HALF_SONAR = 10
SONAR_NOISE_THRHOLD = 4000
EFFECT_RADIUS = 20
EFFECT_NOISE_THRHOLD = 0
K1 = 2
K2 = .25

################################################################
def circle_area_points(x, y, radius):
    #return [ (i+x, j+y) for i in xrange(-radius, radius) for j in xrange(-radius, radius) if ((i)**2 + (j)**2)**0.5 <= radius ]
    return [ (i+x, j+y) for i in xrange(0, radius) for j in xrange(-radius, radius) if ((i)**2 + (j)**2)**0.5 <= radius ]
def arrive(a, b):
	return round(a.x) == round(b.x) and round(a.y) == round(b.y)
def vector_sum(a, b):
    return Point(*[ round(i+j, 5) for i, j in zip(a, b) ])
def vector_minus(a, b):
    return vector_sum(a, Point(-b.x, -b.y, -b.th))
def vector_p_scalar(a, scalar):
    return Point(round(a.x*scalar, 5), round(a.y*scalar, 5), round(a.th*scalar, 5))
def vector_modulus(a):
    return round(sqrt(a.x**2 + a.y**2), 5)
def vector_min_angle(a, b):
    scalar_product = sum([ i*j for i, j in zip(a, b) ])
    modulus_product = vector_modulus(a)*vector_modulus(b)
    try:
        return degrees(acos(scalar_product/modulus_product))
    except:
		return 0
def vector_angle(a, angle):
    cos_angle = cos(radians(angle))
    sin_angle = sin(radians(angle))
    if a.y > 0:
        return vector_min_angle(Point(1, 0, 0), Point(a.x*cos_angle - a.y*sin_angle, a.x*sin_angle + a.y*cos_angle, 0))
    else:
        return -vector_min_angle(Point(1, 0, 0), Point(a.x*cos_angle - a.y*sin_angle, a.x*sin_angle + a.y*cos_angle, 0))
def robot_is_here(robot):
	a = robot.get_pose()
	return Point(round(a.x/MEASURE_REDUCTION, 5), round(a.y/MEASURE_REDUCTION, 5), round(a.th, 5))
def sonar_reduction(sonar):
	return [ round(i/MEASURE_REDUCTION, 5) for i in sonar ]
def robot_vector(angle):
	return Point(round(cos(radians(angle)), 5), round(sin(radians(angle)), 5), angle)
################################################################
class Robot():
    robot = None
    sonar = None
    def __init__(self):
        Aria.init()
        con = ArSimpleConnector(sys.argv)
        self.robot = ArRobot()
        self.sonar = ArSonarDevice()

        if not con.parseArgs():
            con.logOptions()
            Aria.exit(1)
        if not con.connectRobot(self.robot):
            Aria.exit(1)

        self.robot.addRangeDevice(self.sonar)
        self.robot.runAsync(1)

        self.robot.lock()
        self.robot.enableMotors()
        self.robot.unlock()

    def accel(self, speed):
        self.robot.lock()
        self.robot.setVel(speed)
        self.robot.unlock()
    def rotate(self, angle):
        self.robot.lock()
        self.robot.setHeading(angle)
        self.robot.unlock()
    def get_pose(self):
        self.robot.lock()
        a = self.robot.getPose()
        self.robot.unlock()
        return a
    def sonar_read(self):
        return [ self.robot.getSonarRange(i) for i in xrange(8) ]
    def cheat(self):
        return self.sonar.getCurrentBufferAsVector()
################################################################
class MapLearning():
    histogram_map = None
    def __init__(self, MAP_SIZE, MAP_SAMPLER, HALF_SONAR, SONAR_NOISE_THRHOLD, EFFECT_RADIUS, EFFECT_NOISE_THRHOLD, K1, K2):
        self.histogram_map = zeros(shape=(MAP_SIZE,MAP_SIZE), dtype=int8)
        self.MAP_SIZE = MAP_SIZE
        self.MAP_SAMPLER = MAP_SAMPLER
        self.HALF_SONAR = HALF_SONAR
        self.SONAR_NOISE_THRHOLD = SONAR_NOISE_THRHOLD
        self.EFFECT_RADIUS = EFFECT_RADIUS
        self.EFFECT_NOISE_THRHOLD = EFFECT_NOISE_THRHOLD
        self.K1 = K1
        self.K2 = K2
    def render(self, pose, sonar_read):
        #reads = sonar_read[1:7]
        angles = (-50, -30, -10, 10, 30, 50)
        reads = sonar_read
        angles = (-90, -50, -30, -10, 10, 30, 50, 90)
        for sLen, sAngle in zip(reads, angles):
            if sLen < self.SONAR_NOISE_THRHOLD:
                for i in xrange(int(pose.th+sAngle-HALF_SONAR), int(pose.th+sAngle+HALF_SONAR)):
                    x = int(pose.x + sLen*cos(radians(i)))
                    y = int(pose.y + sLen*sin(radians(i)))
                    try:
                        if 0 <= self.histogram_map[x][y] <= 126:
                            self.histogram_map[x][y] += 1
                    except:
                        pass
    def render_cheat(self, cheat_list):
        for point in cheat_list:
            self.histogram_map[point.x][point.y]+=1
    def target_vector(self, origin, destiny):
        try:
            p_repulsive = [ Point(x, y, 0) for x, y in circle_area_points(origin.x, origin.y, self.EFFECT_RADIUS) if (self.histogram_map[x][y] > self.EFFECT_NOISE_THRHOLD) ] #TODO: cache circle_area_points
            v_repulsive = []
            for p in p_repulsive:
                v = vector_minus(origin, p)
                m = vector_modulus(v)
                if m > 0:
                    #levar em consideracao a quantidade de vetores
                    v_repulsive.append(vector_p_scalar(v, self.K1*exp(-m*self.K2)/m))
        except:
            d_vector = vector_minus(destiny, origin)
            return vector_p_scalar(d_vector, 20/vector_modulus(d_vector))

        d_vector = vector_minus(destiny, origin)
        d_vector = vector_p_scalar(d_vector, 20/vector_modulus(d_vector))
        for v in v_repulsive:
            d_vector = vector_sum(v, d_vector)
        return d_vector
    def output_map(self):
        print MAP_SIZE/MAP_SAMPLER,
        for i in self.histogram_map[::MAP_SAMPLER]:
            for j in i[::MAP_SAMPLER]:
                print j,
        print ""
################################################################
if __name__ == "__main__":
    map_learn = MapLearning(MAP_SIZE, MAP_SAMPLER, HALF_SONAR, SONAR_NOISE_THRHOLD/MEASURE_REDUCTION, EFFECT_RADIUS, EFFECT_NOISE_THRHOLD, K1, K2)
    robot = Robot()
    robot.robot.moveTo(ArPose(MAP_SIZE*MEASURE_REDUCTION/2, MAP_SIZE*MEASURE_REDUCTION/2, 0))

    dest_list = [Point(MAP_SIZE, MAP_SIZE/2, 0)]
    for dest in dest_list:
        while not arrive(dest, robot_is_here(robot)):
            sonar_readings = robot.sonar_read()
            buffer_pose = robot_is_here(robot)
            buffer_sonar = sonar_reduction(sonar_readings)

            map_learn.render_cheat([Point(round(i.x/MEASURE_REDUCTION), round(i.y/MEASURE_REDUCTION), 0) for i in robot.cheat()])

            map_learn.render(buffer_pose, buffer_sonar)
            v = map_learn.target_vector(buffer_pose, dest)

            angle = int(vector_angle(v, buffer_pose.th))
            speed = int(vector_modulus(v))

            if speed < 1:
                pass
                #NOVO CODIGO DOS MESTRANDOS
            else:
                pass
                #robot.accel(10*speed)
                robot.robot.move(100)
                robot.rotate(buffer_pose.th+angle)


            print >>stderr, "\r",
            #print >>stderr, "Angle: ", angle, "\t",
            #print >>stderr, "Speed: ", speed, "\t",
            #print >>stderr, "Front: ", robot_vector(buffer_pose.th), "\t",
            #print >>stderr, "Vetor: ", v, "\t",
            #print >>stderr, "Where: ", buffer_pose, "\t",
            #print >>stderr, "Desti: ", dest, "\t",
            #print >>stderr, "Sonar: ", buffer_sonar, "\t",
            print >>stderr, "Sonar: ", sonar_readings, "\t",
            #print >>stderr, ""

            #map_learn.output_map()
