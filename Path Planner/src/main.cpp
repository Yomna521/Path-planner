#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
// for convenience
using nlohmann::json;

using namespace std;

//definitions
const int laneWidth = 4;
const double sampling = 0.02;
const int bufferDistance = 20;
const double maxVel  = 49;
const float front_weight = 0.5;
const float back_weight = 0.4;
const double minVel = 20;
const double acceleration = 0.2;
const double deceleration = 0.4;

int lane = 1;
double refVel = 0;


double dist_cost(double distance, float weight){
	return (weight/distance);
}

bool isLane(double car_d){
  return (car_d < (laneWidth*lane+laneWidth) && car_d > (laneWidth * lane));
}

bool isLeftLane(double car_d){
  int leftLane = lane - 1;
  return ((leftLane < 3) && (leftLane > -1)  && car_d < (laneWidth*leftLane+laneWidth) && car_d > (laneWidth * leftLane));
}

bool isRightLane(double car_d){
  int rightLane = lane +1;
  return (rightLane < 3 && car_d < (laneWidth*rightLane+laneWidth) && car_d > (laneWidth*rightLane));
}

bool isFront(double car_s, double ego_s){
  return (ego_s>car_s);
}

bool isBack(double car_s, double ego_s){
  return  (ego_s < car_s);
}


vector<double> make_prediction(vector<double> ego_car,int prevSize){

  double vx_dash = ego_car[3];
  double vy_dash = ego_car[4];
  double car_s_dash = ego_car[5];
  double car_d_dash = ego_car[6];
  double car_speed = sqrt(vx_dash*vx_dash + vy_dash*vy_dash);
  car_s_dash += car_speed * prevSize * sampling;

  return {car_s_dash,car_d_dash};
}


vector<double> compute_cost(vector<double> ego_car_points, double car_s, double distance){

  double currentCost = 0;
  double leftCost = 0;
  double rightCost = 0;

  vector<double> costs;

  double car_s_dash = ego_car_points[0];
  double car_d_dash = ego_car_points[1];

   //if car's d is within the current lane's range
    if (isLane(car_d_dash)) {
      if(isFront(car_s,car_s_dash) && distance <= bufferDistance){
        currentCost += dist_cost(distance, front_weight);
        } // penalize more for cars infront 
      
    }
    //if the left lane is valid, and car's d is within left lane range
    if (isLeftLane(car_d_dash)) {
      //if there's a car infront or behind in the left lane --> left lane cost += distcost
      if(isFront(car_s,car_s_dash)) {
        leftCost += dist_cost(distance,front_weight); 
      }
     if(isBack(car_s_dash,car_s)){
         leftCost += dist_cost(distance,back_weight);
     }
    }
    //same as the left lane for the right lane
    if (isRightLane(car_d_dash)) {
      if(isFront(car_s,car_s_dash)){
        rightCost += dist_cost(distance,front_weight);
       }
      if(isBack(car_s_dash,car_s)){
         rightCost += dist_cost(distance,back_weight);
     }
    }

  if (lane == 0) {
    leftCost = 999;
  }
  //if right lane is invalid --> cost is infinity
  if (lane == 2) {
    rightCost = 999;
  }

  costs.push_back(leftCost);
  costs.push_back(currentCost);
  costs.push_back(rightCost);

  return costs;

}


//calculate the cost of adjecent lanes
void path_planner(vector<vector<double>> lane_cars, double car_s, int prev_size){

  double currentLane = 0;
  double leftLane = 0;
  double rightLane = 0;  

  double newLaneCost = 0;

  
  for_each(lane_cars.begin(), lane_cars.end(), [&](vector<double> car) {
    vector<double> ego_car_points = make_prediction(car,prev_size);

    double distance = abs(ego_car_points[0] - car_s);
    vector<double> costs = compute_cost(ego_car_points,car_s,distance);
    leftLane += costs[0];
    currentLane += costs[1];
    rightLane += costs[2];
  });

  //if the optimal lane is the current --> new lane = current lane
  if (currentLane <= leftLane && currentLane <= rightLane ) {
    newLaneCost = currentLane;
  } 
  //if the right lane is the optimal lane --> new lane = right lane
  else if (rightLane < currentLane && rightLane <= leftLane ) {
    lane +=1;
    newLaneCost = rightLane;
  } 
  //else then left lane is optimal --> new lane = left lane
  else{
    lane -=1;
    newLaneCost = leftLane;
  }
  
  if(newLaneCost == 0 && refVel < maxVel){
    refVel += acceleration; 
  } else if(refVel > minVel){
    refVel -= deceleration;
  }  
}



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          // MSelim: Begin your code starting from this point
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prevSize = previous_path_x.size();
          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x;
          double ref_y;
          double ref_yaw;

          if(prevSize > 0) {
            car_s = end_path_s;
          }

          path_planner(sensor_fusion, car_s, prevSize);

          //toturial's code
          if(prevSize < 2){
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            double prev_car_x = ref_x - cos(ref_yaw);
            double prev_car_y = ref_y - sin(ref_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);
          } 
          else {
            ref_x = previous_path_x[prevSize - 1];
            ref_y = previous_path_y[prevSize - 1];

            double ref_x_prev = previous_path_x[prevSize - 2];
            double ref_y_prev = previous_path_y[prevSize - 2];
            ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> wp0 = getXY(car_s + 30 , (laneWidth/2 + laneWidth*lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> wp1 = getXY(car_s + 60 , (laneWidth/2 + laneWidth*lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 100 , (laneWidth/2 + laneWidth*lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(wp0[0]);
          ptsx.push_back(wp1[0]);
          ptsx.push_back(wp2[0]);
          ptsy.push_back(wp0[1]);
          ptsy.push_back(wp1[1]);
          ptsy.push_back(wp2[1]);

          for(int i = 0; i < ptsx.size(); ++i){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx,ptsy);

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 0; i < prevSize; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double x_horizon = 30.0;
          double y_horizon = s(x_horizon);
          double dist_horizon = sqrt(pow(x_horizon,2) + pow(y_horizon,2));
          double x_add_on = 0;

          for(int i=0; i < 60 - prevSize; ++i){
            int N = dist_horizon / (sampling*refVel/2.24);
            double x = x_add_on + x_horizon / N;
            double y = s(x);
            x_add_on = x;

            double x_temp = x;
            double y_temp = y;
            x = x_temp * cos(ref_yaw) - y_temp * sin(ref_yaw);
            y = x_temp * sin(ref_yaw) + y_temp * cos(ref_yaw);  
            x += ref_x;
            y+= ref_y;

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }
          // MSelim: End your code before this point


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}



