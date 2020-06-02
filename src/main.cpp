#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steer;
  pid_steer.Init(0, 0.001, 5);
  int counter=-1;
  int flow_cnt=true;


  h.onMessage([&pid_steer,&counter,&flow_cnt](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle=0.5;
          
         if(counter==-1)
         {
           counter++;
           
         }
         else
         {
           if(!pid_steer.P_update)
           {
             std::cout<<"Kp val "<<pid_steer.Kp<<std::endl; 
             pid_steer.param=0;
             if(counter==0 && flow_cnt==true)
             {
               counter=1;
               pid_steer.Twiddle_stage1(cte,0.2);

             }
             else if(counter==1 && flow_cnt==true)
             {
               counter=0;
               flow_cnt=pid_steer.Twiddle_stage2(cte,0.2);
               if(flow_cnt==true)
               {
               	pid_steer.P_update=true;
               	pid_steer.D_update=false;
               }
             }
             else if(flow_cnt==false)
             {
             	pid_steer.Twiddle_stage3(cte);
                flow_cnt=true;
                pid_steer.P_update=true;
                pid_steer.D_update=false;
                
             }

            }
           
           if(!pid_steer.D_update)
           {
             std::cout<<"Kd val "<<pid_steer.Kd<<std::endl; 
             pid_steer.param=2;
             if(counter==0 && flow_cnt==true)
             {
               counter=1;
               pid_steer.Twiddle_stage1(cte,0.2);

             }
             else if(counter==1 && flow_cnt==true)
             {
               counter=0;
               flow_cnt=pid_steer.Twiddle_stage2(cte,0.2);
               if(flow_cnt==true)
               {
               	pid_steer.P_update=false;
               	pid_steer.D_update=true;
               }
             	
             }
             else if(flow_cnt==false)
             {
             	pid_steer.Twiddle_stage3(cte);
                flow_cnt=true;
               	pid_steer.P_update=false;
               	pid_steer.D_update=true;
             }

           }
         }
           
 //          if(!pid_steer.I_updat//         {
//              std::cout<<"Kd val "<<pid_steer.Kd<<stdndl; 
//              pid_steparam=1;
//              if(counter==0 && f_cnt==true)
         
//                coer=(counter+1)%2;
//                pid_steer.Twle_stage1(ct.2);

//              }
//              else if(nter==1 && f_cnt==true)
//              {
//             counter=(counter+1)%2;
//                flow_cnt=pid_steer.Tdle_stage2(c0.2);
             	
//            
//           else if(flow_cnt==false)
//             //              	pid_steer.Tdle_stage3(ct
//                 flow_cnt=true;
//              }

//            }
     
         
          
      
          pid_steer.UpdateError(cte);
         
          steer_value = pid_steer.TotalError();

          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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