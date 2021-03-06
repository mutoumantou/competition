#include "general_header.hpp"

void my_sleep (unsigned msec) {
    struct timespec req, rem;
    int err;
    req.tv_sec = msec / 1000;
    req.tv_nsec = (msec % 1000) * 1000000;
    while ((req.tv_sec != 0) || (req.tv_nsec != 0)) {
        if (nanosleep(&req, &rem) == 0)
            break;
        err = errno;
        // Interrupted; continue
        if (err == EINTR) {
            req.tv_sec = rem.tv_sec;
            req.tv_nsec = rem.tv_nsec;
        }
        // Unhandleable error (EFAULT (bad pointer), EINVAL (bad timeval in tv_nsec), or ENOSYS (function not supported))
        break;
    }
}

/* get present time of day */
double get_present_time (void) {
    struct timeval presentTime;
    gettimeofday( &presentTime, NULL );
    double returnVal = (double) presentTime.tv_sec + presentTime.tv_usec * 1e-6;        // second
    return returnVal;
}

/* calc. difference between two angles in angles (+: counterclockwise rotationg; -: clockwise rotation)*/
float calc_angle_difference (float angle1, float angle2) {
    float dif = angle2 - angle1;
    while (dif > 180)
        dif -= 360;
    while (dif <= -180)
        dif += 360;
    return dif;
}
