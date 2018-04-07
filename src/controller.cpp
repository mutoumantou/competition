#include "controller.hpp"

/* constructor */
MMC_Controller :: MMC_Controller () {
    robot.x = 0; robot.y = 0;
    cargo.x = 0; cargo.y = 0;
    goal.x  = 0; goal.y  = 0;
    cargoAngle = 0;
    preRobot.x = 0; preRobot.y = 0;
    preCargo.x = 0; preCargo.y = 0;
    robotBeforeContact.x = 0;
    robotBeforeContact.y = 0;
    dis = 0.0; angle = 0.0;
    fContact = 0;
    state = 0;
    angleDiff = 0.0;            // the angle difference between (robot - cargo) and (cargo heading)
}

/*
output: 1: valid new position; 0: cargo pos not valid
intput: 0: 1st run, use for preCargo and preRobot; 1: following runs
robot pos. is considered to be always valid
*/
int MMC_Controller :: get_latest_pos (int data) {
  if (data) {
    return_center_pt_info ( &robot, &cargo, &cargoAngle );
    //printf("robot (%d, %d) cargo (%d, %d), cargoAngle %.3f\n", robot.x, robot.y, cargo.x, cargo.y, cargoAngle);
    preRobot.x = robot.x;
    preRobot.y = robot.y;
    if ( abs(cargo.x - preCargo.x) < 20 && abs(cargo.y - preCargo.y) < 20 ) {
        preCargo.x = cargo.x;
        preCargo.y = cargo.y;
        robotBeforeContact.x = robot.x;         // update robotBeforeContact pos. only when cargo pos. is valid
        robotBeforeContact.y = robot.y;
        return 1;
    } else {        // if cargo pos. is not valid, do not update dis.
        cargo.x = preCargo.x;
        cargo.y = preCargo.y;
        return 0;
    }
  }
  return_center_pt_info ( &preRobot, &preCargo, &cargoAngle );
  return 1;
}

/* set the goal of controller to be the detected cargo position */
void MMC_Controller :: update_goal_info_using_cargo_pos (void) {
    goal.x = cargo.x + 100 * cosd(cargoAngle);
    goal.y = cargo.y + 100 * sind(cargoAngle);
    //printf("robot (%d, %d) cargo (%d, %d), goal (%d, %d)\n", robot.x, robot.y, cargo.x, cargo.y, goal.x, goal.y);
    dis    = sqrt  ( pow ( goal.x - robot.x, 2 ) + pow ( goal.y - robot.y, 2 ) );
    angle  = atan2 ( goal.y - robot.y, goal.x - robot.x) * 180.0 / M_PI;
}

/* set the detected cargo position as the goal position of the controller */
void MMC_Controller :: set_cargo_as_goal (void) {
    goal.x = cargo.x;
    goal.y = cargo.y;
    dis    = sqrt  ( pow ( goal.x - robot.x, 2 ) + pow ( goal.y - robot.y, 2 ) );
    angle  = atan2 ( goal.y - robot.y, goal.x - robot.x) * 180.0 / M_PI;
    printf("cargo (%d, %d) dis %.3f, angle %.3f\n", cargo.x, cargo.y, dis, angle);
}

/*
output: 1: contact happens; 0: contact not happen
this fun. is only called when cargo is not correctly detected
*/
int MMC_Controller :: check_contact (int data) {
    int dis_threshold = 60;             // threshold for decide contact
    switch (data) {
        case 0: dis_threshold = 60; break;              // circular cargo
        case 1: dis_threshold = 76; break;              // rectangular cargo
        case 2: dis_threshold = 70; break;
    }
  if (dis < dis_threshold)           // if pre. distance is < threshold, then contact happened
    fContact = 1;
                          // else, just a wrong detection, ignore this detection
  return fContact;
}

int MMC_Controller :: robot_away_from_init_pos (void) {
    float dis = sqrt  ( pow ( robotBeforeContact.x - robot.x, 2 ) + pow ( robotBeforeContact.y - robot.y, 2 ) );
    printf("robot dis. to its pos. before contact is %.3f\n", dis);
    if ( dis > 30 )
        return 1;
    else
        return 0;
}

/* calc. the angle difference between (robot - cargo) and (cargo heading) */
void MMC_Controller :: calc_angle_difference_to_desired_line (void) {
    float x = robot.x - cargo.x;
    float y = robot.y - cargo.y;
    float angleRobotCargo = atan2(y, x) * 180 / M_PI;               // angle from cargo pointing to robot
    angleDiff = calc_angle_difference (cargoAngle, angleRobotCargo);     // general fun.
    printf("angle diff. is %.3f\n", angleDiff);
}
