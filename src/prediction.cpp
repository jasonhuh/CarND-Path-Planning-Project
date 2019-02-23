#include "prediction.h"
#include "vehicle.h"
#include "util.h"
#include "settings.h"

vector<vector<double>> Prediction::generate_prediction(Vehicle* ego, vector<vector<double>> sensor_fusion, 
                            double prev_size, double end_path_s, double end_path_d,
                            double car_s, double car_d) {
    vector<vector<double>> preds;
    if (prev_size > 0) {
        ego->s = end_path_s;
        ego->d = end_path_d;
        if (ego->d < 1.0) {
            ego->d = 1.0;
        } else if (ego->d > (LANE_WIDTH * NUM_LANES) - 1.0) {
            ego->d = (LANE_WIDTH * NUM_LANES) - 1.0;
        }
        // Using sensor fusion, build prediction.
        for (int i = 0; i < sensor_fusion.size(); ++i) {
            vector<double> other = sensor_fusion[i];
            int other_id = (int)other[SF_ID_IDX];
            double other_s = other[SF_S_IDX];
            double other_d = other[SF_D_IDX];
            double other_v = sqrt(pow(other[SF_X_IDX],2) + pow(other[SF_Y_IDX],2));
            double other_sn = other_s + other_v * prev_size * tint;
            int other_l = get_lane(other_d);

            // Other car is within the range of interest.
            if ((other_l >= 0 && other_l < NUM_LANES) &&
                ((other_s - car_s) > -REAR_CAR_REACTION_RANGE)) {
                other[5] = other_sn;
                preds.push_back({(double)other_id,other_sn, other_d, other_v});
            }
        }
    } else {
        ego->s = car_s;
        ego->d = car_d;
        preds = sensor_fusion;
    }
    return preds;
}