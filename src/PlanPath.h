//
// Created by lab on 12/26/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H



#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;

class PlanPath {

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //init once : load waypoints from ../data/highway_map.csv
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    const double lane_width = 4.0;

    //keep state
    int lane = 2;
    double safety_gap = 40;
    double ref_vel = .224;
//    int prev_size = 0;
    long last_timer = 0;
    long current_timer = 0;

    /////////////
    const double freq_r = 0.02;
    const double mph_to_m = 2.24;  // 2.24 = 1.604 * 1000 / 3600 // 2.24 to get to m/s from mph

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //load waypoints from ../data/highway_map.csv
    void init();

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
    string hasData(string s);

    // For converting back and forth between radians and degrees.
    constexpr double pi() { return M_PI; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }

    // Ecludian distance
    double distance(double x1, double y1, double x2, double y2);

    // Closest and  Next Waypoint
    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double>
    getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


    //////////////////// lane utils /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double lane2d(int lane)  ;
    int d2Lane(double d) ;
    bool isLaneValid(int nlane) ;
    bool isLaneChangeAllowed(int lane) ;
    bool decideLaneAndSpeed(vector<vector<double>>& sensor_fusion, int prev_size, double car_s );
    bool isInSameLane(double d, int lane) ;

//    bool isTooClose(vector<vector<double>>& sensor_fusion, int prev_size, double& check_lane_speed, double& closest_car_distance, double car_s );

    //////////////// lane and speed /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    double getOtherCarS(vector<double>& sensor_fusion_i, int prev_size);
    bool safeLaneChange( vector<vector<double>>& sensor_fusion, double& check_lane_speed,  int valid_new_lane, int prev_size ,int car_s);
    bool isCurrLaneSafe( vector<vector<double>>& sensor_fusion, double& closest_car_distance, double& check_lane_speed,  int valid_new_lane, int prev_size ,int car_s);

    /////////////// WayPoints and trajectory //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void fillPointAtInterval(vector<double>& ptsx , vector<double>& ptsy, double car_s, int inetrval, int count) ;
    void fillLast2Points(vector<double>& ptsx , vector<double>& ptsy,  vector<double>& previous_path_x ,  vector<double>& previous_path_y,  double& ref_x,  double& ref_y,  double& ref_yaw, int prev_size);
    void transformToCarRef(vector<double>& ptsx , vector<double>& ptsy, double& ref_x,  double& ref_y,  double& ref_yaw ) ;
    void createPointsFromSpline(tk::spline& spline, vector<double>& next_x_vals , vector<double>& next_y_vals, vector<double>& previous_path_x, vector<double>& previous_path_y, double& ref_x,  double& ref_y,  double& ref_yaw );


    //    void fillPointAtInterval(vector<double>& ptsx , vector<double>& ptsy, double car_s, int inetrval, int count);
    //    vector<double> last2Points(  vector<double>& ptsx , vector<double>& ptsy, int prev_size , double car_s, double car_x, double car_y, double car_yaw, vector<double>& previous_path_x, vector<double>& previous_path_y) ;
    //    void decideNextWayPoints(json& j,vector<double>& ptsx , vector<double>& ptsy, double car_s, int prev_size );

public:
    PlanPath() {
        init();
    }

    ~PlanPath() {};

//    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
//    uWS::OpCode opCode) {
    string onMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode);

};
#endif //PATH_PLANNING_UTILS_H
