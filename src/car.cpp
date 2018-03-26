#include "car.h"
#include <iostream>
#include <math.h>

Car::Car() { this->id = -9999999;}

/*Car::Car(int id, double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot, double x, double y, Helpers::State state, int lane) {
  this->id = id;
  this->s = s;         
  this->s_dot = s_dot;
  this->s_dot_dot = s_dot_dot;  
  this->d = d;        
  this->d_dot = d_dot;      
  this->d_dot_dot = d_dot_dot; 
  this->x = x;
  this->y = y;
  this->state = state;
  this->lane = lane;
}*/

void Car::predictTrajectory(int prev_path_size) {
  this->predicted_trajectory.clear();
  for (int i = 1; i <= PREDICTION_HORIZON_TIMESTEPS; i++) {
    //double predicted_s = this->s + this->s_dot * (double)prev_path_size * PREDICTION_HORIZON_TIME_DELTA * i;
    double predicted_s = this->s + this->s_dot * PREDICTION_HORIZON_TIME_DELTA * i;
    vector<double> predicted_sd = {predicted_s, this->d};
    this->predicted_trajectory.push_back(predicted_sd);
  }
}

void Car::printPredictedTrajectoryLastPoint() {
  if (!this->predicted_trajectory.empty()) {
    cout << "Car " << this->id <<": speed=" << this->s_dot << ", s=" << this->s << ", Predicted trajectory last point-> s=" << this->predicted_trajectory.back()[0] << ", d=" << this->predicted_trajectory.back()[1] << endl;
  }
}

void Car::printCar() {
  cout << "Car: id= " << this->id << " ~~~ speed=" << this->s_dot << " ~~~ lane=" << this->lane << " ~~~ x=" << this->x << ", y=" << this->y << " ~~~ s=" << this->s << ", d=" << this->d << endl;
}

/**
 * @brief Checks if there are any non-ego cars left, right, same lane within the threshold distance.
 * The check is only based on current s.
 * @param cars Non-ego car collection.
 */
void Car::checkProximity(vector<Car> cars) {
  this->proximity.car_infront_same_lane = false;
  this->proximity.car_behind_same_lane = false;
  this->proximity.car_infront_left_lane = false;
  this->proximity.car_behind_left_lane = false;
  this->proximity.car_infront_right_lane = false;
  this->proximity.car_behind_right_lane = false;
  
  for (Car car : cars) {
    double dist_diff = car.s - this->s;
    bool within_critical_distance = ((fabs(dist_diff) < EGO_CAR_TOO_CLOSE_DISTANCE) ? true : false);
    if (within_critical_distance) { 
      if (car.lane == this->lane && dist_diff > 0.0) {
        this->proximity.car_infront_same_lane = true;
        cout << "Car " << car.id <<  ": " << fabs(dist_diff) << " units in front in same lane!" << endl;
      }
      else if (car.lane == this->lane && dist_diff <= 0.0) {
        this->proximity.car_behind_same_lane = true;
        cout << "Car " << car.id <<  ": " << fabs(dist_diff) << " units behind in same lane!" << endl;
      }
      else if (car.lane < this->lane && dist_diff > 0.0) {
        this->proximity.car_infront_left_lane = true;
        cout << "Car " << car.id << ": " << fabs(dist_diff) << " units in front in left lane!" << endl;
      }
      else if (car.lane < this->lane && dist_diff <= 0.0) {
        this->proximity.car_behind_left_lane = true;
        cout << "Car " << car.id << ": " << fabs(dist_diff) << " units behind in left lane!" << endl;
      }
      else if (car.lane > this->lane && dist_diff > 0.0) {
        this->proximity.car_infront_right_lane = true;
        cout << "Car " << car.id << ": " << fabs(dist_diff) << " units in front in right lane!" << endl;
      }
      else if (car.lane > this->lane && dist_diff <= 0.0) {
        this->proximity.car_behind_right_lane = true;
        cout << "Car " << car.id << ": " << fabs(dist_diff) << " units behind in right lane!" << endl;
      }
    }
  }
}

/**
 * @brief Finds lead car in the target lane
 * @param cars Collection of non-ego cars that needs to be searched
 * @param target_lane Lane to search for finding lead car
 * @return The nearest lead car
 */
Car Car::findLeadCar(vector<Car> cars, int target_lane) {
  double nearest_s = 99999999;
  Car lead_car;
  for (Car car : cars) {
    if (car.lane == target_lane) { // same as target lane
      double start_s = car.predicted_trajectory.front()[0];
      double end_s = car.predicted_trajectory.back()[0];
      if (start_s > this->s && end_s < nearest_s) {
        nearest_s = end_s;
        lead_car = car;
      }
    }
  }
  //if (lead_car.id != -9999999) cout << "Lead car found!!!" << endl;
  return lead_car;
}

/**
 * @brief Computes possible states of the ego car based on current lane & if there are any non-ego cars left or right.
 * The keep Lane state is added by default.
 */
void Car::computeFutureStates() {
  this->future_states = {Helpers::KEEP_LANE};
  
  if ((this->lane == 1 || this->lane == 2) && !this->proximity.car_behind_left_lane && !this->proximity.car_infront_left_lane) {
    this->future_states.push_back(Helpers::LANE_CHANGE_LEFT);
    //cout << "LCL Added!" << endl;
  }
  
  else if ((this->lane == 0 || this->lane == 1) && !this->proximity.car_behind_right_lane && !this->proximity.car_infront_right_lane) {
    this->future_states.push_back(Helpers::LANE_CHANGE_RIGHT);
    //cout << "LCR Added!" << endl;
  }
}

/**
 * @brief Computes the target s, s_dot, s_dot_dot & d, d_dot, d_dot_dot for all possible states.
 * @param cars The collections of non-ego cars whose trajectory has already been predicted
 * @param prev_path_size Size of the unutlised waypoints from previous cycle
 */
void Car::computeFutureStatesTargetSD(vector<Car> cars,int prev_path_size) {
  this->future_states_target_sd.clear();
  
  //double target_s_dot = EGO_CAR_MAX_VELCOITY;
  double target_s_dot = this->s_dot;
  //double target_s = this->s + target_s_dot * (double)prev_path_size * PREDICTION_HORIZON_TIMESTEPS * PREDICTION_HORIZON_TIME_DELTA;
  double target_s = this->s + target_s_dot * PREDICTION_HORIZON_TIMESTEPS * PREDICTION_HORIZON_TIME_DELTA;
  double target_s_dot_dot = 0.0;
  double target_d;
  double target_d_dot = 0.0;
  double target_d_dot_dot = 0.0;
  int target_lane;
  
  for (Helpers::State state : this->future_states) {
    switch (state) {
      case (Helpers::KEEP_LANE):
        target_d = (double)this->lane * 4 + 2;
        target_lane = this->lane;
        break;
      case (Helpers::LANE_CHANGE_LEFT):
        target_d = (double)(this->lane - 1) * 4 + 2;
        target_lane = target_d / 4;
        break;
      case (Helpers::LANE_CHANGE_RIGHT):
        target_d = (double)(this->lane + 1) * 4 + 2;
        target_lane = target_d / 4;
        break;
      default:
        target_d = (double)this->lane * 4 + 2;
        target_lane = this->lane;
        break;
    }
    
    Car lead_car = this->findLeadCar(cars, target_lane);   
    if (lead_car.id != -9999999) {
      double end_s = lead_car.predicted_trajectory.back()[0];
      cout << "Lead car --> state=" << Helpers::getStateStringValue(state) << ", distance=" << (lead_car.s - this->s) << ", speed=" << lead_car.s_dot << ", s=" << lead_car.s << ", predicted s=" << end_s << endl;
      //cout << "Lead car --> state=" << Helpers::getStateStringValue(state) << ", (predicted final s - target s)=" << (end_s - target_s) << " units" << endl;
      if (lead_car.s - this->s < EGO_CAR_TOO_CLOSE_DISTANCE) {
        target_s_dot = lead_car.s_dot * 0.25;
        cout << "Lead too close! Reducing target s to: " << target_s_dot << endl;
      }
      else if (end_s - target_s < EGO_CAR_TOO_CLOSE_DISTANCE) { // Check if lead car prdicted s too close to target s
        cout << "Lead car within " << EGO_CAR_TOO_CLOSE_DISTANCE << " unit distance of target s for state: " << Helpers::getStateStringValue(state) << ". Current targets s= " << target_s << ", s_dot=" << target_s_dot << endl;
        target_s = end_s - EGO_CAR_TOO_CLOSE_DISTANCE; // stay behind the lead car at a safe distance
        //target_s_dot = lead_car.s_dot; // match lead car's speed
        target_s_dot -= 0.224; // gradually decrease speed
        cout << "                                                             Adjusted targets s= " << target_s << ", s_dot=" << target_s_dot << endl;
      }
      else {
        if (target_s_dot < EGO_CAR_MAX_VELCOITY) { // Increase speed
          target_s_dot += 0.224; // ~5 m/s^2 below than the 10 m/s^s threshold requirement
        } 
      }
    }
    else { // No lead car found
      if (target_s_dot < EGO_CAR_MAX_VELCOITY) { // Increase speed
        target_s_dot += 0.224; // ~5 m/s^2 below than the 10 m/s^s threshold requirement
      }
    }
    this->future_states_target_sd[state] = {target_s, target_s_dot, target_s_dot_dot, target_d, target_d_dot, target_d_dot_dot};
  }
}

/**
 * @brief Generates trajectory for all possible states based on previously computed target s & d values.
 */
void Car::generateFutureStatesTrajectory() {
  this->future_states_trajectory.clear();
  for (auto future_state_target_sd : this->future_states_target_sd) {
    vector<vector<double>> future_state_trajectory;
    Helpers::State state = future_state_target_sd.first;
    vector<double> target_sd = future_state_target_sd.second;
    vector<double> start_s = {this->s, this->s_dot, this->s_dot_dot};
    vector<double> start_d = {this->d, this->d_dot, this->d_dot_dot};
    vector<double> goal_s = {target_sd[0], target_sd[1], target_sd[2]};
    vector<double> goal_d = {target_sd[3], target_sd[4], target_sd[5]};
    vector<double> s_coeffs = JMT::get_jmt_coefficients(start_s, goal_s, PREDICTION_HORIZON_TIMESTEPS * PREDICTION_HORIZON_TIME_DELTA);
    vector<double> d_coeffs = JMT::get_jmt_coefficients(start_d, goal_d, PREDICTION_HORIZON_TIMESTEPS * PREDICTION_HORIZON_TIME_DELTA);
    
    vector<double> s_trajectory;
    vector<double> d_trajectory;
    for (int i = 0; i < PREDICTION_HORIZON_TIMESTEPS; i++) {
      double timestamp = i * PREDICTION_HORIZON_TIME_DELTA; // the timestamp to generate the prediction for
      double s = 0.0;
      double d = 0.0;
      for (int j = 0; j < s_coeffs.size(); j++) {
        s += s_coeffs[j] * pow(timestamp, j);
        d += d_coeffs[j] * pow(timestamp, j);
      }
      s_trajectory.push_back(s);
      d_trajectory.push_back(d);
    }
    future_state_trajectory.push_back(s_trajectory);
    future_state_trajectory.push_back(d_trajectory);
    this->future_states_trajectory[state] = future_state_trajectory;
  }
}

/**
 * @brief Computes state of the non-ego car at time t
 * @param t The time for which the state should be calculated.
 * @return Length 6 vector giving the car's expected [s, s_dot, s_dot_dot, d, d_dot, d_dot_dot] state at time t
 */
vector<double> Car::computeStateAtTime(double t) {
  vector<double> state;
  
  state.push_back(this->s + (this->s_dot * t) + (this->s_dot_dot * pow(t,2) / 2.0));
  state.push_back(this->s_dot + this->s_dot_dot * t);
  state.push_back(this->s_dot_dot);
  state.push_back(this->d + (this->d_dot * t) + (this->d_dot_dot * pow(t,2) / 2.0));
  state.push_back(this->d_dot + this->d_dot_dot * t);
  state.push_back(this->d_dot_dot);
  
  return state;
}

void Car::printFutureStates() {
  int i = 0;
  for (Helpers::State future_state : this->future_states) {
    cout << "Futre State " << i << ": " << Helpers::getStateStringValue(future_state) << endl;
    i += 1;
  }
}

void Car::printFutureStatesTargetSD() {
  for (auto future_state_target_sd : this->future_states_target_sd) {
    Helpers::State state = future_state_target_sd.first;
    vector<double> target_sd = future_state_target_sd.second;
    cout << "Future State " << Helpers::getStateStringValue(state) << " Targets: s=" << target_sd[0] << ", s_dot=" << target_sd[1] << ", s_dot_dot=" << target_sd[2] << ", d=" << target_sd[3] << ", d_dot=" << target_sd[4] << ", d_dot_dot=" << target_sd[5] << endl;
  }
}

void Car::printFutureStatesTrajectory() {
  for (auto future_state_trajectory : this->future_states_trajectory) {
    Helpers::State state = future_state_trajectory.first;
    vector<vector<double>> trajectory = future_state_trajectory.second;
    vector<double> s_trajectory = trajectory[0];
    vector<double> d_trajectory = trajectory[1];
    cout << "Future State " << Helpers::getStateStringValue(state) << endl;
    cout << "     s trajectory:" << endl;
    for (int i = 0; i < s_trajectory.size(); i++) {
       cout << "                " << s_trajectory[i] << endl;
    }
    cout << "     d trajectory:" << endl;
    for (int i = 0; i < d_trajectory.size(); i++) {
       cout << "                " << d_trajectory[i] << endl;
    }
  }
}

void Car::printFutureStatesTrajectoryLastSD() {
  for (auto future_state_trajectory : this->future_states_trajectory) {
    Helpers::State state = future_state_trajectory.first;
    vector<vector<double>> trajectory = future_state_trajectory.second;
    vector<double> s_trajectory = trajectory[0];
    vector<double> d_trajectory = trajectory[1];
    cout << "Future State " << Helpers::getStateStringValue(state) << " Trajectory: Last s=" << s_trajectory[s_trajectory.size()-1] << ", d=" << d_trajectory[d_trajectory.size()-1] << endl;
  }
}

Car::~Car(){};