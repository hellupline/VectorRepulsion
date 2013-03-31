#include <iostream>
#include <vector>
#include <math.h>
#include <Aria.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"
#include "mapRender.h"

#define D_MEASURE_REDUCTION 050.0
#define D_MAP_SIZE 1000

#define D_EFFECT_THRHOLD 2
#define D_SONAR_NOISE_THRHOLD 3000
#define D_EFFECT_RADIUS 500/D_MEASURE_REDUCTION

#define D_K1 10.0
#define D_K2 0.4

#define D_SONAR_HALF_POLAR_DISTANCE 10

using namespace std;

/**************************************************************************/
inline dirVector robot_is_here(ArRobot& robot) { return dirVector(robot.getX()/D_MEASURE_REDUCTION, robot.getY()/D_MEASURE_REDUCTION, robot.getTh()); }
inline vector<int> sonar_reduction(ArRobot& robot) {
    vector<int> reduced;
    for (int i=0; i < robot.getNumSonar(); i++)
        reduced.push_back(robot.getSonarRange(i)/D_MEASURE_REDUCTION);
    return reduced;
}

inline void printSonar(vector<int>& sonar) {
    cout << "(" << sonar[0];
    for (unsigned int i=1; i < sonar.size(); i++)
        cout << "," << sonar[i];
    cout << ") ";
}
inline void printVector(dirVector& v) { cout << "(" << round(v.x) << "," << round(v.y) << "," << round(v.z) << ") "; }
/**************************************************************************/
inline bool arrived(dirVector dest, ArRobot& robot) {
    float x=dest.x-robot.getX()/D_MEASURE_REDUCTION;
    float y=dest.y-robot.getY()/D_MEASURE_REDUCTION;
    return (round(sqrt(x*x+y*y)) < 50/D_MEASURE_REDUCTION);
}
/**************************************************************************/
inline vector<xy> getPioneerSonar(ArSonarDevice& sonar) {
    vector<ArPoseWithTime> *pose_list = sonar.getCurrentBufferAsVector();
    vector<xy> xy_list;
    for (unsigned int i=0; i < pose_list->size(); i++)
        xy_list.push_back(xy((*pose_list)[i].getX()/D_MEASURE_REDUCTION, (*pose_list)[i].getY()/D_MEASURE_REDUCTION, 128));
    return xy_list;
}
/**************************************************************************/
int main(int argc, char **argv) {
    ArSonarDevice sonar;
    ArRobot robot;

    Aria::init();

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);
    if (!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "Could not connect to the robot.");
        if (parser.checkHelpAndWarnUnparsed()){
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

    robot.moveTo(ArPose(D_MAP_SIZE*D_MEASURE_REDUCTION/2, D_MAP_SIZE*D_MEASURE_REDUCTION/2, 0));
    ArLog::log(ArLog::Normal, "Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV", robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());

    robot.lock();
    robot.enableMotors();
    robot.unlock();
/**************************************************************************/
    mapLearning histogram_map(D_MEASURE_REDUCTION, D_MAP_SIZE, D_EFFECT_THRHOLD, D_SONAR_NOISE_THRHOLD, D_EFFECT_RADIUS, D_K1, D_K2, D_SONAR_HALF_POLAR_DISTANCE);
    dirVector dest, pose, move, d_list[] = {
        dirVector(5700/D_MEASURE_REDUCTION, 4000/D_MEASURE_REDUCTION, 0),
        //dirVector(0, 4000/D_MEASURE_REDUCTION, 0),
        //dirVector(5700/D_MEASURE_REDUCTION, 0, 0),
    };
    vector<int> readings;
    float angle=0, speed=0;

    for (int d_index=0; d_index < 1; d_index++) { pose=robot_is_here(robot); dest = vectorSum(pose, d_list[d_index]);
        while (!arrived(dest, robot)) {
            pose = robot_is_here(robot);
            readings = sonar_reduction(robot);

            //histogram_map.render(pose, readings);
            histogram_map.renderPrecise(getPioneerSonar(sonar));
            move = histogram_map.vector(pose, dest);

            angle = vectorAngle(move, pose);
            speed = vectorMod(move);

            if (speed < 1) {
            } else {
                robot.setDeltaHeading(angle);
                robot.move(50);
            }
            cout << "\r";
            cout << "vector: "; printVector(move);
            cout << "pose: "; printVector(pose);
            cout << "dest: "; printVector(dest);
            cout << "angle: " << round(angle) << " ";
            cout << "speed: " << round(speed) << " ";
            /*
            cout << "sonar: "; printSonar(readings);
            cout << "arrived: " << arrived(dest, robot) << " ";
            cout << endl;
            */
            render(
                    *(histogram_map.histogramMap),
                    rect(
                        pose.x-D_EFFECT_RADIUS*4,
                        pose.y-D_EFFECT_RADIUS*4,
                        D_EFFECT_RADIUS*8,
                        D_EFFECT_RADIUS*8
                    ),
                    pose, move,
                    D_EFFECT_RADIUS
            );
        }
    }
/**************************************************************************/
    ArUtil::sleep(3000);
    robot.stopRunning();
    robot.waitForRunExit();
    Aria::exit(0);
}
