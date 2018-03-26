#ifndef HELPERS_H
#define HELPERS_H

#include "parameters.h"
#include <string>

using namespace std;

class Helpers
{
public:
  //Helpers();
  //virtual ~Helpers();
  
  enum State { KEEP_LANE, LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT};
  
  static string getStateStringValue(int int_value) {
    string state = "";
    switch (int_value) {
      case (0):
        state = "KEEP_LANE";
        break;
      case (1):
        state = "LANE_CHANGE_LEFT";
        break;
      case (2):
        state = "LANE_CHANGE_RIGHT";
        break;
      default:
        state = "KEEP_LANE";
        break;
    }
    return state;
  }
  
  struct cars_in_close_proximity { 
    bool car_infront_left_lane; 
    bool car_behind_left_lane; 
    bool car_infront_right_lane; 
    bool car_behind_right_lane; 
    bool car_infront_same_lane; 
    bool car_behind_same_lane; 
  };
};

#endif // HELPERS_H
