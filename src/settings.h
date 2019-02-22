#ifndef SETTINGS_H
#define SETTINGS_H

#include <cmath>

const double tint = 0.02;
const double tpint = 0.1;
const double aint = 0.185;

const double MAX_V = 49.84 * 0.44704; // meters per second for 49.95 miles per hour
const double MAX_S = 6945.554; // max s before finishing around the track
const double MAX_ACC = aint / tint; // max acceleration

const double LANE_WIDTH = 4.0; // meters

const int NUM_LANES = 3; // number of car lanes

const int COMFORT           = 10;
const int CENTER            = 10;
const int PREPARE_WO_CHANGE = 10;
const int CANCEL_COST       = 100;
const int OCCUPIED_LANE     = 500;
const int DANGER            = 1000;
const int NEAR              = 10000;
const int INEFFICIENCY      = 10000;
const double MAX_COST       = 1000000; // 1M

const double DESIRED_BUFFER = 0.5;

const int TIME_DISTANCE = 200;
const int PLANNING_HORIZON = 7;

const double CAR_LENGTH = 10.0;
const double CAR_WIDTH = 2.0;

const int PLANNING_DISTANCE = 30;
const int PLANNING_STEPS = 20;

#endif // SETTINGS_H