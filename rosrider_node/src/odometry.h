#ifndef __ODOMETRY_h
#define __ODOMETRY_h

float pose_x = 0.0f;
float pose_y = 0.0f;
float pose_theta = 0.0f;
float pose_xVel = 0.0f;
float pose_yVel = 0.0f;
float pose_thetaVel = 0.0f;

float orientation[4];

void quaternion_from_euler(float ai, float aj, float ak) {
    ai /= 2.0f;
    aj /= 2.0f;
    ak /= 2.0f;
    float ci = cos(ai);
    float si = sin(ai);
    float cj = cos(aj);
    float sj = sin(aj);
    float ck = cos(ak);
    float sk = sin(ak);
    float cc = ci * ck;
    float cs = ci * sk;
    float sc = si * ck;
    float ss = si * sk;
    orientation[0] = (cj * sc) - (sj * cs);
    orientation[1] = (cj * ss) + (sj * cc);
    orientation[2] = (cj * cs) - (sj * sc);
    orientation[3] = (cj * cc) + (sj * ss);
}

void update_pose(double delta_seconds) {

    int32_t left_delta = get_left_delta();
    int32_t right_delta = get_right_delta();

    float left_travel = (float) left_delta / TICKS_PER_METER;
    float right_travel = (float) right_delta / TICKS_PER_METER;

    float delta_travel = (right_travel + left_travel) / 2;
    float delta_theta = (right_travel - left_travel) / params_float[PARAM_BASE_WIDTH];

    float delta_x;
    float delta_y;

    if(right_travel == left_travel) {    
        delta_x = left_travel * cos(pose_theta);
        delta_y = left_travel * sin(pose_theta);
    } else {
        float radius = delta_travel / delta_theta;
        // find the instantaneous center of curvature (ICC).
        float icc_x = pose_x - radius * sin(pose_theta);
        float icc_y = pose_y + radius * cos(pose_theta);
        delta_x = cos(delta_theta) * (pose_x - icc_x) - sin(delta_theta) * (pose_y - icc_y) + icc_x - pose_x;
        delta_y = sin(delta_theta) * (pose_x - icc_x) + cos(delta_theta) * (pose_y - icc_y) + icc_y - pose_y;
    }

    pose_x += delta_x;
    pose_y += delta_y;
    pose_theta = (float) fmod((pose_theta + delta_theta), (2.0 * MATHPI));
    
    if(delta_seconds > 0.0) {
        pose_xVel = delta_travel / delta_seconds;
        pose_thetaVel = delta_theta / delta_seconds;
    } else {
        pose_xVel = 0.0f;
        pose_thetaVel = 0.0f;
    }
    pose_yVel = 0.0f; 

}

#endif