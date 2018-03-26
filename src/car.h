#ifndef CAR_H
#define CAR_H

#include "helpers.h"
#include "parameters.h"
#include "JMT.h"
#include <vector>
#include <map>

using namespace std;

class Car
{
public:
    int id;                             // unique id
    double s;                           // longitudinal position in sd coordinate system
    double s_dot;                       // longitudinal velocity (speed) in sd coordinate system
    double s_dot_dot;                   // longitudinal accleration
    double d;                           // lateral position (position in lane)
    double d_dot;                       // lateral velocity
    double d_dot_dot;                   // lateral acceleration
    double x;                           // horizontal position in cartesian coordinate system
    double y;                           // vertical position in cartesian coordinate system
    Helpers::State state;               // Keep Lane, Lane Change Right, Lane Change Left
    int lane;                           // 0=Left, 1=Centre, 2=Right
    vector<vector<double>> predicted_trajectory; // predicted trajectory
    Helpers::cars_in_close_proximity proximity;  // proximity situation
    vector<Helpers::State> future_states;        // possible future states
    map<Helpers::State,vector<double>> future_states_target_sd;  // futures states' taget s, s_dot, s_dot, d, d_dot, d_dot 
    map<Helpers::State,vector<vector<double>>> future_states_trajectory;  // trajectory for each future state state -> {{s trajectory},{d trajectory}}

    Car();
    //Car(int id, double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot, double x, double y, Helpers::State state, int lane);
    virtual ~Car();
    
    void checkProximity(vector<Car> cars);    
    void computeFutureStates();
    void computeFutureStatesTargetSD(vector<Car> cars, int prev_path_size);
    void generateFutureStatesTrajectory();
    void predictTrajectory(int prev_path_size);
    vector<double> computeStateAtTime(double t);
    Car findLeadCar(vector<Car> cars, int lane);
    void printPredictedTrajectoryLastPoint();
    void printCar();
    void printFutureStates();
    void printFutureStatesTargetSD();
    void printFutureStatesTrajectory();
    void printFutureStatesTrajectoryLastSD();
};

#endif // CAR_H
