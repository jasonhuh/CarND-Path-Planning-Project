#ifndef SETTINGS_H
#define SETTINGS_H

#include <cmath>

const double tint = 0.02;
const double tpint = 0.1;
const double aint = 0.185;

const double MAX_V = 49.84 * 0.44704; // meters per second for 49.95 miles per hour
const double MAX_S = 6945.554; // max s before finishing around the track
const double MAX_A = aint / tint;

const double LANE_WIDTH = 4.0; // meters

const int NUM_LANES = 3; // number of car lanes

// const int COMFORT           = 0xF << 1; //pow(10.0, 1);
// const int CENTER            = 0xF << 1; //pow(10.0,1);
// const int PREPARE_WO_CHANGE = 0xF << 1; // pow(10.0, 1);;
// const int CANCEL_COST       = 0xF << 2; // pow(10.0, 2);
// const int OCCUPIED_LANE     = 0xF << 3; //5 * pow(10.0,2);
// const int DANGER            = 0xF << 4; //pow(10.0, 3);
// const int NEAR              = 0xF << 5; //pow(10.0, 4);
// const int EFFICIENCY        = 0xF << 5; //pow(10.0, 4);
// const double MAXCOST        = 0xF << 6; //pow(10.0, 6);
const int COMFORT           = pow(10.0, 1);
const int CENTER            = pow(10.0,1);
const int PREPARE_WO_CHANGE = pow(10.0, 1);;
const int CANCEL_COST       = pow(10.0, 2);
const int OCCUPIED_LANE     = 5 * pow(10.0,2);
const int DANGER            = pow(10.0, 3);
const int NEAR              = pow(10.0, 4);
const int EFFICIENCY        = pow(10.0, 4);
const double MAX_COST        = pow(10.0, 6);

const double DESIRED_BUFFER = 0.5;

const int TIME_DISTANCE = 200;
const int PLANNING_HORIZON = 5;

const double CAR_LENGTH = 10.0;
const double CAR_WIDTH = 2.0;

const int PLANNING_DISTANCE = 30;
const int PLANNING_STEPS = 20;

#endif // SETTINGS_H