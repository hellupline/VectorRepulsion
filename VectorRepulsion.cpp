#include <iostream>
#include <vector>
#include <math.h>
#include <Aria.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"

#define D_MEASURE_REDUCTION 100
#define D_MAP_SIZE 500
#define D_HALF_SONAR 10

#define D_SONAR_NOISE_THRHOLD 3000
#define D_EFFECT_RADIUS 15
#define D_EFFECT_NOISE_THRHOLD 1

#define D_K1 20.0
#define D_K2 1.0

using namespace std;

/**************************************************************************/
inline bool arrived(dirVector dest, ArRobot& robot) { return (dest.x == robot.getX() && dest.y == robot.getY()); }
inline std::vector<int> sonar_reduction(ArRobot& robot) {
    std::vector<int> reduced;
    unsigned int n_sonar = robot.getNumSonar();
    for(unsigned int i=0; i < n_sonar; i++)
        reduced.push_back(robot.getSonarRange(i)/D_MEASURE_REDUCTION);
    return reduced;
}
inline dirVector robot_is_here(ArRobot& robot) { return dirVector(robot.getX()/D_MEASURE_REDUCTION, robot.getY()/D_MEASURE_REDUCTION, robot.getTh()); }
inline void printVector(dirVector& v) { cout << "(" << round(v.x) << "," << round(v.y) << "," << round(v.z) << ") "; }
/**************************************************************************/

int main(int argc, char **argv) {
/**************************************************************************/
    ArSonarDevice sonar;
    ArRobot robot;

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);
    if(!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed()){
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
    }

    robot.runAsync(true);
    robot.addRangeDevice(&sonar);

    robot.moveTo(ArPose((D_MAP_SIZE*D_MEASURE_REDUCTION)>>1, (D_MAP_SIZE*D_MEASURE_REDUCTION)>>1, 0));

    robot.lock();
    robot.enableMotors();
    robot.unlock();

    robot.lock();
    ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV", robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
    robot.unlock();
/**************************************************************************/
    mapLearning histogram_map(D_MAP_SIZE, D_MEASURE_REDUCTION, D_HALF_SONAR, D_SONAR_NOISE_THRHOLD, D_EFFECT_RADIUS, D_EFFECT_NOISE_THRHOLD, D_K1, D_K2);
    dirVector d_list[] = {dirVector(D_MAP_SIZE, D_MAP_SIZE, 0)};
    //dirVector d_list[] = {dirVector(D_MAP_SIZE, D_MAP_SIZE>>1, 0)};
    for(int d_index=0; d_index < 1; d_index++) {
        while (!arrived(d_list[d_index], robot)) {
            dirVector pose=robot_is_here(robot);
            std::vector<int> sonar=sonar_reduction(robot);

            histogram_map.render(pose, sonar);
            dirVector v=histogram_map.vector(pose, d_list[d_index]);

            float angle=vectorAngle(v, pose);
            float speed=vectorMod(v);

            if (speed < 1) {
            } else {
                robot.move(50);
                robot.setDeltaHeading(angle);
            }

            cout << "angle: " << round(angle) << " " << "speed: " << round(speed) << " ";
            cout << "pose: "; printVector(pose);
            cout << "vector: "; printVector(v);
            cout << "sonar: " << sonar[0] << "," << sonar[1] << "," << sonar[2] << "," << sonar[3] << "," << sonar[4] << "," << sonar[5] << "," << sonar[6] << "," << sonar[7] << " ";
            cout << endl;
            ArUtil::sleep(125);
        }
    }
/**************************************************************************/
    ArUtil::sleep(3000);
    robot.stopRunning();
    robot.waitForRunExit();
    Aria::exit(0);
/**************************************************************************/
}
