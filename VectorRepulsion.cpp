#include <iostream>
#include <vector>
#include <Aria.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"
#include "robotControl.h"

#define D_MEASURE_REDUCTION 200
#define D_MAP_SIZE 500
#define D_HALF_SONAR 10

#define D_SONAR_NOISE_THRHOLD 4000
#define D_EFFECT_RADIUS 20
#define D_EFFECT_NOISE_THRHOLD 0

#define D_K1 2
#define D_K2 0.25

inline bool arrived(dirVector dest, dirVector robot) {
    return (dest.x-robot.x < 1 && dest.y-robot.y < 1);
}
inline dirVector robot_is_here(robotControl& robot) {
    dirVector pose = robot.get_pose();
    return dirVector(pose.x/D_MEASURE_REDUCTION, pose.y/D_MEASURE_REDUCTION, 0);
}

inline std::vector<int> sonar_reduction(std::vector<int> sonar) {
    std::vector<int> reduced;
    for(unsigned int i=0; i < sonar.size(); i++)
        reduced.push_back(sonar[i]/D_MEASURE_REDUCTION);
    return reduced;
}

int main(int argc, char **argv) {
    mapLearning histogram_map(D_MAP_SIZE, D_HALF_SONAR, D_SONAR_NOISE_THRHOLD, D_EFFECT_RADIUS, D_EFFECT_NOISE_THRHOLD, D_K1, D_K2);
    robotControl robot(argc, argv);

    robot.robot.moveTo(ArPose((D_MAP_SIZE*D_MEASURE_REDUCTION)>>1, (D_MAP_SIZE*D_MEASURE_REDUCTION)>>1, 0));

    dirVector d_list[] = {dirVector(D_MAP_SIZE, D_MAP_SIZE, 0)};
    for(int d_index=0; d_index < 1; d_index++) {
        while (!arrived(d_list[d_index], robot_is_here(robot))) {
            std::vector<int> sonar_readings=robot.get_sonar();
            dirVector buffer_pose=robot.get_pose();
            std::vector<int> buffer_sonar=sonar_reduction(sonar_readings);

            histogram_map.render(buffer_pose, buffer_sonar);
            dirVector v=histogram_map.vector(buffer_pose, d_list[d_index]);

            int angle=vectorAngle(v, buffer_pose.z);
            int speed=vectorMod(v);

            if (speed < 1) {
            } else {
                robot.robot.move(10);
                robot.rotate(buffer_pose.z+angle);
            }

        }
    }
}
