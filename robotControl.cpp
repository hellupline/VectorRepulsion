#include <vector>
#include <Aria.h>

#include "dirVector.h"
#include "robotControl.h"

robotControl::robotControl(int argc, char **argv) {
    Aria::init();

    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    //ArRobot robot;
    //ArSonarDevice sonar;
    ArRobotConnector robotConnector(&parser, &robot);
    if(!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
    }

    ArLog::log(ArLog::Normal, "simpleConnect: Connected.");

    robot.addRangeDevice(&sonar);
    robot.runAsync(true);
    robot.lock();
    ArLog::log(ArLog::Normal,
        "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
        robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getBatteryVoltage());
    robot.unlock();

}
void robotControl::accel(int a) {
    robot.setVel(a);
}
void robotControl::rotate(int a) {
    robot.setHeading(a);
}
dirVector robotControl::get_pose(){
    return dirVector(robot.getX(), robot.getY(), robot.getTh());
}
std::vector<int> robotControl::get_sonar(){
    std::vector<int> sonar;
    int n_sonar = robot.getNumSonar();
    for(int i=0; i < n_sonar; i++)
        sonar.push_back(robot.getSonarRange(i));
    return sonar;
}
