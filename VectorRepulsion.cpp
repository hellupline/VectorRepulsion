#include <iostream>
#include <vector>
#include <math.h>
#include <Aria.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"
#include "mapRender.h"

#define D_MEASURE_REDUCTION 25
#define D_MAP_SIZE 10000
#define D_HALF_SONAR 10

#define D_SONAR_NOISE_THRHOLD 3000
#define D_EFFECT_RADIUS 1500/D_MEASURE_REDUCTION
#define D_EFFECT_NOISE_THRHOLD 2

#define D_K1 20.0
#define D_K2 1.0

using namespace std;

/**************************************************************************/
inline dirVector robot_is_here(ArRobot& robot) { return dirVector(robot.getX()/D_MEASURE_REDUCTION, robot.getY()/D_MEASURE_REDUCTION, robot.getTh()); }
inline vector<int> sonar_reduction(ArRobot& robot) {
    vector<int> reduced;
    unsigned int n_sonar = robot.getNumSonar();
    for(unsigned int i=0; i < n_sonar; i++)
        reduced.push_back(robot.getSonarRange(i)/D_MEASURE_REDUCTION);
    return reduced;
}

inline void printVector(dirVector& v) { cout << "(" << round(v.x) << "," << round(v.y) << "," << round(v.z) << ") "; }
inline void printSonar(vector<int>& sonar) {
    cout << "(" << sonar[0];
    for (unsigned int i=1; i < sonar.size(); i++)
        cout << "," << sonar[i];
    cout << ") ";
}

inline bool arrived(dirVector dest, ArRobot& robot) { return (dest.x == robot.getX() && dest.y == robot.getY()); }
/**************************************************************************/
int main(int argc, char **argv) {
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

            //cout << "\r";
            //cout << "angle: " << round(angle) << " ";
            //cout << "speed: " << round(speed) << " ";
            //cout << "pose: "; printVector(pose);
            //cout << "vector: "; printVector(v);
            //cout << "sonar: "; printSonar(sonar);
            //cout << endl;
            render(*(histogram_map.histogramMap), rect(pose.x-D_EFFECT_RADIUS, pose.y-D_EFFECT_RADIUS, D_EFFECT_RADIUS*2, D_EFFECT_RADIUS*2), pose, v, D_EFFECT_RADIUS);

            //ArUtil::sleep(125);
        }
    }
/**************************************************************************/
    ArUtil::sleep(3000);
    robot.stopRunning();
    robot.waitForRunExit();
    Aria::exit(0);
}
