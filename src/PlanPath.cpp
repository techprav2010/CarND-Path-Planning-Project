//
// Created by lab on 12/26/18.
//

#include "PlanPath.h"

using namespace std;

// for convenience
using json = nlohmann::json;
////////////

// vehicle is in lane   ?
bool PlanPath::isInSameLane(double d, int nlane) {
    int lane_new = d2Lane(d);
    return isLaneValid(nlane) && (lane_new == nlane);
}

bool PlanPath::isLaneChangeAllowed(int nlane) {
    return isLaneValid(nlane) && ( lane + 1 == nlane || nlane -1 == lane);
}

bool PlanPath::isLaneValid(int nlane) {
    return (nlane > 0  && nlane < 4) ;
}

double PlanPath::lane2d(int lane) {
    if (lane == 1) return 2.0;
    if (lane == 2) return 6.0;
    if (lane == 3) return 10.0;
    return 0;
}

int PlanPath::d2Lane(double d) {
    if (d < 4) return 1;
    if (d >= 4.0 & d < 8.0) return 2;
    if (d >= 8.0 & d < 12.0) return 3;
    return 3;
}

double PlanPath::getOtherCarS(vector<double>& sensor_fusion_i, int prev_size){
    double vx = sensor_fusion_i[3];
    double vy = sensor_fusion_i[4];
    double check_lane_speed = sqrt(pow(vx, 2) + pow(vy, 2));// m/s
    double check_car_s = sensor_fusion_i[5];
    //where the car will be in future with respect to planned distance travel
    double forward_dist = prev_size * freq_r * check_lane_speed;
    check_car_s += forward_dist;
    return check_car_s;
}

// set closest_car_distance and  check_lane_speed
bool PlanPath::isCurrLaneSafe( vector<vector<double>>& sensor_fusion, double& closest_car_distance, double& check_lane_speed,  int valid_new_lane, int prev_size ,int car_s){
    bool too_close=false;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        double d = sensor_fusion[i][6];
        if (isInSameLane(d, lane)) {
            double check_car_s = getOtherCarS(sensor_fusion[i], prev_size);
            double gap = check_car_s - car_s;
            if (gap > 0 && gap < safety_gap ) {
                too_close = true;
                if (gap < closest_car_distance) {
                    closest_car_distance = gap;
                    check_lane_speed = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));// m/s
                }
            }
        }
    }
    return too_close;
}

//is lane change safe and what is best speed, acceleration.
bool PlanPath::safeLaneChange ( vector<vector<double>>& sensor_fusion,  double& check_lane_speed,  int valid_new_lane, int prev_size ,int car_s){
    bool should_lane_change=true;
    for (int i = 0; i < sensor_fusion.size(); i++) {
        double d = sensor_fusion[i][6];
        int car_lane = d2Lane(d);
        if (car_lane == valid_new_lane) {
            double check_car_s = getOtherCarS(sensor_fusion[i], prev_size);
            double gap = check_car_s - car_s ;
            if ( abs(gap) < safety_gap ) {
                should_lane_change=false;
                break;
            }
        }
    }
    return should_lane_change;
}

// adjust the speed and decide best lane we should be in
bool PlanPath::decideLaneAndSpeed(vector<vector<double>>& sensor_fusion, int prev_size, double car_s ){
    //check --current lane for collision
    //check if lane shange is needed
    //plan and do lan change if safe
    //try maintain spped without causing jerks

    double closest_car_distance = 10000;
    double check_lane_speed=0;
    bool should_lane_change = false;
    int valid_new_lane = lane;
    int old_lane =lane;

    //first check we are too close the vehicle ahead of us in current lane.
    bool too_close = isCurrLaneSafe(sensor_fusion, closest_car_distance, check_lane_speed, lane, prev_size , car_s);

    //too_close == true; reduce the speed?
    if(too_close){
        if (closest_car_distance < 10)
            ref_vel -= .4;
        else if (closest_car_distance < 20)
            ref_vel -= .338;
        else  if( ref_vel >  check_lane_speed * mph_to_m)
            ref_vel -= .224;
    } else {
        //else adjust speed to traffic .....
        if( check_lane_speed !=0 && 35 >  check_lane_speed * mph_to_m)
            too_close=true;
        else if( check_lane_speed !=0 && ref_vel >  check_lane_speed * mph_to_m)
            ref_vel -= .224;
        else if (ref_vel < 47.5)
            ref_vel += .224;
    }

    //if too_close == true; lane change?
    //look for 'chance' to change lane.
    //should try only once in == (current_timer - last_timer) > 100
    // try left and right lanes , change lane if no cars around in the other lane
    // if both lanes has cars, just slow down and stay in this lane ...
    if (too_close &&  (current_timer - last_timer) > 100 ) {
        //try changing lane to left
        int temp_lane = lane - 1;
        if(!should_lane_change && isLaneChangeAllowed(temp_lane)){
            should_lane_change = safeLaneChange(sensor_fusion, check_lane_speed, temp_lane, prev_size, car_s);
            if(should_lane_change){
                valid_new_lane =temp_lane;
            }
        }
        //else try changing lane to right
        temp_lane = lane + 1;
        if(!should_lane_change && isLaneChangeAllowed(temp_lane)){
            should_lane_change = safeLaneChange(sensor_fusion, check_lane_speed, temp_lane, prev_size, car_s);
            if(should_lane_change){
                valid_new_lane =temp_lane;
            }
        }
    }

    //ok , now switch lane if its safe and required
    if(should_lane_change){
        lane = valid_new_lane;
        last_timer = 0;
        current_timer = 0;
        if (ref_vel < 47.5)
            ref_vel += .224;
    }

//    cout << "           " <<   " ref_vel=" <<  ref_vel << ", closest_car= " << closest_car_distance << ", check_lane_speed="<<  check_lane_speed << ", cur_lane="<<  lane<<  ", old_lane="<<  old_lane << ", too_close=" << too_close << endl;
    return should_lane_change;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//create additional points for spline
void PlanPath::fillPointAtInterval(vector<double>& ptsx , vector<double>& ptsy, double car_s, int inetrval, int count) {
    double new_d = lane2d(lane);
    for(int i=1; i<=count; i++){
        vector<double> next_wp0 = getXY(car_s + inetrval * i, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        // push x
        ptsx.push_back(next_wp0[0]);
        // push y
        ptsy.push_back(next_wp0[1]);
    }
}
//get last two points + fillPointAtInterval == for spline creation
void PlanPath::fillLast2Points(vector<double>& ptsx , vector<double>& ptsy,  vector<double>& previous_path_x ,  vector<double>& previous_path_y,  double& ref_x,  double& ref_y,  double& ref_yaw, int prev_size) {
    if (prev_size < 2) {
        //  path tangent to the car
        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);
        //x
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);
        //y
        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
//        cout << "prev_size<2 :" << " x " << ref_x<< " p_x " << prev_car_x << " ref_yaw " << ref_yaw << endl;
    }  else {
        // last two points
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        //yaw from two points
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        //x
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        //y
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);

//        cout << "prev_size<2 :" << " x " << ref_x<< " p_x " << ref_x_prev << " ref_yaw " << ref_yaw << endl;
    }
}

//anchor points so that it is now in the car's reference frame
void PlanPath::transformToCarRef(vector<double>& ptsx , vector<double>& ptsy, double& ref_x,  double& ref_y,  double& ref_yaw ) {
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
}

// create points using spline -- the number of points should be based on what speed we intend to have
void PlanPath::createPointsFromSpline(tk::spline& spline, vector<double>& next_x_vals , vector<double>& next_y_vals, vector<double>& previous_path_x, vector<double>& previous_path_y, double& ref_x,  double& ref_y,  double& ref_yaw ) {

    //get previuosly created and non consumed points
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    // get points from  splineas many as needed.
    // number of points needed =  N * freq_r * ref_vel = reference distance --> 30m
    // (freq_r =0.2, mph_to_m= 2.24)
    // ref_vel is decided in = decideLaneAndSpeed(sensor_fusion, prev_size, car_s);
    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

    double x_add_on = 0;
    double vel_ms= (freq_r * ref_vel / mph_to_m); //freq_r =0.2, mph_to_m= 2.24
    double N = target_dist / vel_ms;
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        // decide next x at interval decided by N=target_dist / vel_ms
        double x_point = x_add_on + target_x / N;
        //get from spline
        double y_point = spline(x_point);
        //use for next point in seq = cars+spline's ref point
        x_add_on = x_point;

        //transform  back to normal and add to final points
        double x_final = ref_x + (x_point * cos(ref_yaw) - y_point * sin(ref_yaw));
        double y_final = ref_y + (x_point * sin(ref_yaw) + y_point * cos(ref_yaw));
        next_x_vals.push_back(x_final);
        next_y_vals.push_back(y_final);
    }
}


string PlanPath::onMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode){
    current_timer++ ;
    auto s = hasData(data);
    if (s != "") {
        auto j = json::parse(s); //json j = json::parse(s);
//        cout << typeid(j).name() << endl;
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
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];
            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

            // Keep track of Previous Size based on project walkthrough
            int prev_size = previous_path_x.size();
            // reference point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //create trajectory
            vector<double> ptsx;
            vector<double> ptsy;
            //get points from spline and convert to general coord
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Using Sensor Fusion to avoid collision
            if (prev_size > 0) {
                car_s = end_path_s;
            }

            //adjust speed and lane, so that we should p[roperly do lane change and jerk avoidance
            decideLaneAndSpeed(sensor_fusion, prev_size, car_s);

            /////////////////// spline with 5 points ///////////////////////
            //1. fillLast2Points: get last two points
            fillLast2Points(ptsx ,  ptsy,  previous_path_x ,   previous_path_y,  ref_x,   ref_y,   ref_yaw,   prev_size);
            //2. fillPointAtInterval - add frenent points 30ms apart , total 3
            fillPointAtInterval( ptsx ,  ptsy,   car_s, 30, 3) ;
            //transform the anchor points to car's ref coord
            transformToCarRef( ptsx ,   ptsy,   ref_x, ref_y,  ref_yaw ) ;
            //3. Create a spline
            tk::spline spline;
            spline.set_points(ptsx, ptsy);
            /////////////////// spline with 5 points :Done ///////////////////////

            //sample points from spline -- this will be decided based on velocity and action interval=0.2
            createPointsFromSpline(spline,  next_x_vals ,  next_y_vals,  previous_path_x, previous_path_y,  ref_x, ref_y, ref_yaw ) ;

            //create json and return string
            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            return msg;
        }
    }
    return "42[\"manual\",{}]";
}






///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Waypoint map to read from ../data/highway_map.csv
void  PlanPath::init() {
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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

    cout << "INITED "  << endl;
}



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string PlanPath::hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double PlanPath::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int PlanPath::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int PlanPath::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> PlanPath::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PlanPath::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// copied from main.ccp ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


