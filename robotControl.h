class robotControl {
    private:
    public:
        ArRobot robot;
        ArSonarDevice sonar;
    robotControl(int argc, char **argv);
    void accel(int a);
    void rotate(int a);
    dirVector get_pose();
    std::vector<int> get_sonar();
};
