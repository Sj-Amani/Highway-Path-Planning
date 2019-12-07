#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "json.hpp"


using namespace std;

// For convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/* Checks if the SocketIO event has JSON data.
** If there is data the JSON object in string format will be returned,
*/ else the empty string "" will be returned.
string hasData(string s) {
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
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

	// Find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// See if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// Calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// The x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


vector<string> successor_states(string current_state,int lane) {
    /*
    ** Provides the possible next states given the current state for the FSM 
    ** discussed in the course, with the exception that lane changes happen 
    ** instantaneously, so LCL and LCR can only transition back to KL.
    */
    int lanes_available=3;
    vector<string> states;
    states.push_back("KL");
    if(current_state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (current_state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (current_state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
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

  //int lane = 1;
  double ref_vel = 0;
  string current_state="KL";
  int lock=30;
  int prev_target_lane=1;
  int target_lane=1;

  h.onMessage([&target_lane,&prev_target_lane,&lock,&current_state,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    /* "42" at the start of the message means there's a websocket message event.
    ** The 4 signifies a websocket message
    ** The 2 signifies a websocket event
    ** auto sdata = string(data).substr(0, length);
    */cout << sdata << endl;
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
            int lane = (int)round((car_d-2)/4);

            cout<<lane<<" ";
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            if(prev_size>0)
            {
                car_s=end_path_s;
            }

            bool too_close=false;
            bool right_clear=true;
            bool left_clear=true;
            bool right2_clear=true;
            bool left2_clear=true;
            double left_speed=0;
            double right_speed=0;            
            double left2_speed=0;
            double right2_speed=0;
            bool left_block=false;
            bool right_block=false;

            double left_dist=1000;
            double right_dist=1000;            
            double left2_dist=1000;
            double right2_dist=1000;
            double front_dist=1000;

            double front_speed=ref_vel;

            for(int i=0;i<sensor_fusion.size();++i)
            {


                float d=sensor_fusion[i][6];
                double vx=sensor_fusion[i][3];
                double vy=sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s=sensor_fusion[i][5];
                check_car_s+=(double)prev_size*0.02*check_speed;



                if(d<=(4*lane+4) && d>(4*lane))
                {

                    if(check_car_s>car_s && check_car_s-car_s<30)
                    {
                        too_close=true;//ref_vel=29.5;
                    }
                    if(check_car_s-car_s<front_dist && check_car_s-car_s>0)
                    {
                        front_speed=check_speed;
                        front_dist=check_car_s-car_s;
                    }

                }
                double threshold=30;

                if(lane==0)
                {
                    left_clear=false;
                    left2_clear=false;
                    //middle lane
                    if (d<=8 && d>4)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            right_clear=false;
                        if(check_car_s-car_s>-8 && check_car_s-car_s<15)
                            right_block=true;
                        if(check_car_s-car_s>10 && check_car_s-car_s<right_dist)
                        {
                            right_speed=check_speed;
                            right_dist=check_car_s-car_s;
                        }
                    }
                    //right lane
                    if (d<=12 && d>8)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            right2_clear=false;
                        if(check_car_s-car_s>10 && check_car_s-car_s<right2_dist)
                        {
                            right2_speed=check_speed;
                            right2_dist=check_car_s-car_s;
                        }
                    }

                }
                else if(lane==1)
                {
                    left2_clear=false;
                    right2_clear=false;

                    //left lane
                    if (d<=4 && d>0)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            left_clear=false;
                        if(check_car_s-car_s>8 && check_car_s-car_s<left_dist)
                        {
                            left_speed=check_speed;
                            left_dist=check_car_s-car_s;
                        }
                    }
                    //right lane
                    if (d<=12 && d>8)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            right_clear=false;
                        if(check_car_s-car_s>10 && check_car_s-car_s<right_dist)
                        {
                            right_speed=check_speed;
                            right_dist=check_car_s-car_s;
                        }
                    }
                }
                else if(lane==2)
                {
                    right_clear=false;
                    right2_clear=false;
                    //middle lane
                    if (d<=8 && d>4)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            left_clear=false;
                        if(check_car_s-car_s>-8 && check_car_s-car_s<15)
                            left_block=true;
                        if(check_car_s-car_s>10 && check_car_s-car_s<left_dist)
                        {
                            left_speed=check_speed;
                            left_dist=check_car_s-car_s;
                        }
                    }
                    //left lane
                    if (d<=4 && d>0)
                    {
                        if(check_car_s-car_s>-8 && check_car_s-car_s<threshold)
                            left2_clear=false;
                        if(check_car_s-car_s>10 && check_car_s-car_s<left2_dist)
                        {
                            left2_speed=check_speed;
                            left2_dist=check_car_s-car_s;
                        }
                    }
                }



            }

            double front_headway=1000;
            double right_headway=1000;
            double left_headway=1000;
            double left2_headway=1000;
            double right2_headway=1000;
            front_headway=front_dist/(car_speed-front_speed);
            if (front_headway<0) front_headway=1000;
            left_headway=left_dist/(car_speed-left_speed);
            if (left_headway<0) left_headway=1000;
            right_headway=right_dist/(car_speed-right_speed);
            if (right_headway<0) right_headway=1000;
            right2_headway=right2_dist/(car_speed-right2_speed);
            if (right2_headway<0) right2_headway=1000;
            left2_headway=left2_dist/(car_speed-left2_speed);
            if (left2_headway<0) left2_headway=1000;

            if (lane==0) {left_headway=0; left2_headway=0;} else right2_headway=0;
            if (lane==2) {right_headway=0; right2_headway=0;} else left2_headway=0;


            if(lock<30)
                ++lock;



            //disable two lane change
            //left2_headway=0;
            //right2_headway=0;

            if(car_speed>40 && (lock==30 || lock<5)&& front_dist>8 && max(max(left2_headway,right2_headway),max(left_headway,right_headway))>0.3+front_headway)
            {
                if(left_clear && right_clear)
                {
                    target_lane=lane+2*(right_headway>left_headway)-1;
                    lock=0;
                }
                else if(left_clear && max(left_headway,left2_headway)>0.3+front_headway)
                {
                    target_lane=lane-1;
                    lock=0;
                }
                else if(right_clear && max(right_headway,right2_headway)>0.3+front_headway)
                {
                    target_lane=lane+1;
                    lock=0;
                }
            }

            if(too_close && car_speed>front_speed)
            {
                if (target_lane==lane)
                    ref_vel-=5*(40-front_dist)/30*0.224;
            }
            else if(ref_vel<49.0)
            {
                ref_vel+=4*0.224;
            }

            if (target_lane!=prev_target_lane) lock=5;
            prev_target_lane=target_lane;

            cout<<"S "<<car_speed<<" lock "<<lock<<" left: "<<left_headway<<" right: "<<right_headway<<" front "<<front_headway<<" "<<left_clear<<right_clear<<endl;



          	json msgJson;

            //waypoints
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            vector<double> best_x_vals;
            vector<double> best_y_vals;
            double min_cost=1000000;
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            vector<double> ptsx;
            vector<double> ptsy;
            double ref_x=car_x;
            double ref_y=car_y;
            double ref_yaw=deg2rad(car_yaw);

            //Get two points
            if(prev_size<2)
            {
                //
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else
            {
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }
            
            //Get three far away waypoints in Frenet coordinates
            vector<double> wp0 = getXY(car_s+40,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> wp1 = getXY(car_s+70,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> wp2 = getXY(car_s+100,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            
            ptsx.push_back(wp0[0]);
            ptsx.push_back(wp1[0]);
            ptsx.push_back(wp2[0]);

            ptsy.push_back(wp0[1]);
            ptsy.push_back(wp1[1]);
            ptsy.push_back(wp2[1]);


            //Convert to car fixed coordinates
            for (int i=0;i<ptsx.size();++i)
            {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                ptsx[i] = (shift_x*cos(-ref_yaw)-shift_y*sin(-ref_yaw));
                ptsy[i] = (shift_x*sin(-ref_yaw)+shift_y*cos(-ref_yaw));
            }


            tk::spline s;
            s.set_points(ptsx,ptsy);


            for(int i=0;i<previous_path_x.size();++i)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x=30;
            double target_y=s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);

            double x_add_on = 0;



            for (int i=1;i<=50-previous_path_x.size();++i)
            {
                double N = target_dist/(0.02*ref_vel/2.24);
                double x_point = x_add_on+target_x/N;
                double y_point = s(x_point);

                x_add_on = x_point;
                double x_ref = x_point;
                double y_ref = y_point;

                // change back to the world coordinates
                x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
                y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
                x_point+=ref_x;
                y_point+=ref_y;


                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }





          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile 
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
