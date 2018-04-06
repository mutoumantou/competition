#ifndef CONTROLLER
#define CONTROLLER

#include "general_header.hpp"
#include "s826.hpp"
#include "vision.hpp"

/* define a class for the controller */
class MMC_Controller {
    public:
      float angleDiff;        // the angle difference between (robot - cargo) and (cargo heading)
    MMC_Controller (void);      // constructor
    int get_latest_pos (int data);  // get latest position info from vision.cpp
    int check_contact (int data);         // check if robot has touched cargo
    float dis, angle;                 // distance from robot to cargo, desired moving angle
    cv :: Point robot, cargo, goal;         // center point position of robot and cargo
    float cargoAngle;                   // orientation of cargo
    int fContact;
    int state;                          // 0: move to cargo; 1: rotate; 2: grab cargo; 3: move to destination
    void update_goal_info_using_cargo_pos (void);
    void set_cargo_as_goal (void);
    cv :: Point robotBeforeContact;     // position of robot before contact
    int robot_away_from_init_pos (void);        // check if robot is away from the pos. where it is before contact. For cases that robot touches cargo then not touches it in following frames
        void calc_angle_difference_to_desired_line (void);  // calc. the angle difference between (robot - cargo) and (cargo heading)

    private:
        cv :: Point preRobot, preCargo;   // previous center point information of robot and cargo
};

#endif
