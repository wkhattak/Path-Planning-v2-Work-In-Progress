#ifndef PARAMERTS_H
#define PARAMERTS_H

#define EGO_CAR_ID 99999
#define EGO_CAR_START_VELOCITY 35
#define EGO_CAR_START_LANE 1
#define EGO_CAR_MAX_VELCOITY 49.5                       // Max allowed speed of ego car
#define EGO_CAR_TOO_CLOSE_DISTANCE 10                   // If within this unit distance then non-ego car considered too close
#define EGO_CAR_SAFE_DISTANCE 30                        // Min. unit distance for ego car to perform lane change or to travel at max speed

//#define PREDICTION_HORIZON 3                          // Prediction window in seconds (T)
#define PREDICTION_HORIZON_TIMESTEPS 50                 // No. of timesteps in prediction horizon (N)
#define PREDICTION_HORIZON_TIME_DELTA 0.02              // How often to generate the prediction (delta t)

#endif // PARAMERTS_H
