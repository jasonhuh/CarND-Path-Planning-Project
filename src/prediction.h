#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>

using std::vector;

class Vehicle;

class Prediction {
public:
    static vector<vector<double>> generate_prediction(Vehicle* ego, vector<vector<double>> sensor_fusion, 
                            double prev_size, double end_path_s, double end_path_d,
                            double car_s, double car_d);
};

#endif // PREDICTION_H