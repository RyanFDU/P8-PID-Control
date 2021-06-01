#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using namespace std;

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

    PID pid;
    /**
     * TODO: Initialize the pid variable.
     */

    // initial vector p, dp, p_best
    vector<double> p = {0.10000, 0.00050, 1.50000};
    vector<double> dp = {0.01000, 0.00010, 0.1000};
    vector<double> p_best = {p[0],p[1],p[2]};
    
    bool is_twiddle = false; // whether to use the twiddle algorithm
    bool first_drive = true; // run first loop trying: p[p_index] + dp[p_index]
    bool second_drive = true; // run second loop trying: p[p_index] - dp[p_index]
    bool move = false; // whether to move to next parameter of vector p

    int n = 0; // start point of simulator
    int n_begin = 0; // start to calculate error
    int n_end = 700; // end to calculate error
    int p_id = 0; // index of current parameter in p
    int iteration = 0; // count the loops simulator have runed

    double total_error = 0.0;
    double error = 0.0;
    double best_error = 999999;
    double tolerance = 0.1; // the sum of dp < tolerance

    bool best_parameter = false; // whether best p is found

    if (is_twiddle) {
        pid.Init(p[0], p[1], p[2]);
    } else {
        pid.Init(0.155, 0.0011, 1.691);
    }


    h.onMessage([&pid, &n, &n_begin, &n_end, &total_error, &error, &first_drive, &second_drive, &is_twiddle,
                &p, &p_id, &dp, &best_parameter, &p_best, &best_error, &iteration, &move, &tolerance]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

                    json msgJson;
                    /**
                     * TODO: Calculate steering value here, remember the steering value is
                     *   [-1, 1].
                     * NOTE: Feel free to play around with the throttle and speed.
                     *   Maybe use another PID controller to control the speed!
                     */

                    if (is_twiddle) {
                        int n_interval = n_end - n_begin;
                        // initialize the PID when restart the simulator
                        if (n == 0) {
                            pid.Init(p[0], p[1], p[2]);
                        }
                        
                        // calculate the sum of error
                        if (n > n_begin && n < n_end) {
                            total_error += cte * cte;
                        }

                        // update the PID and calculate the steer value between [-1,1]
                        pid.UpdateError(cte);
                        steer_value = pid.TotalError();

                        if (steer_value > 1) {
                            steer_value = 1;
                        } else if (steer_value < -1) {
                            steer_value = -1;
                        }
                        n += 1; // go next point in each simulator loop

                        // twiddle after each simulator loop
                        if (n > n_end) {
                            if (iteration == 0) {
                                best_error = total_error / n_interval;
                            }

                            // twiddle process
                            if (first_drive) {
                                p[p_id] += dp[p_id];
                                first_drive = false;

                            } else {
                                error = total_error / n_interval;
                                // check if p+dp reduce the error
                                if (error < best_error && second_drive) {
                                    p_best = {p[0], p[1], p[2]};
                                    best_error = error;
                                    dp[p_id] *= 1.1;
                                    move = true;

                                } else {
                                    if (second_drive) { // change to p-dp
                                        p[p_id] -= 2 * dp[p_id];
                                        second_drive = false;

                                    } else { // check if p-dp reduce the error
                                        if (error < best_error) {
                                            p_best = {p[0], p[1], p[2]};
                                            best_error = error;
                                            dp[p_id] *= 1.1;
                                            move = true;

                                        } else {
                                            p[p_id] += dp[p_id];
                                            dp[p_id] *= 0.9;
                                            move = true;
                                        }
                                    }
                                }
                            }

                            // reset the flags after each optimization move
                            if (move) {
                                p_id += 1;
                                cout << "move_index: " << p_id << endl;
                                first_drive = true;
                                second_drive = true;
                                move = false;

                            } else {
                                cout << "stay_index: " << p_id << endl;
                            }

                            // circulate within p[0], p[1], p[2]
                            if (p_id == 3) {
                                p_id = 0;
                            }

                            // check if we can stop optimization
                            double sum_dp = dp[0] + dp[1] + dp[2];
                            if (sum_dp < tolerance) {
                                best_parameter = true;
                            }

                            n = 0;
                            iteration += 1;
                            total_error = 0.0;

                            // print the output
                            if (n == 0 && !best_parameter) {
                                std::string reset_msg = "42[\"reset\",{}]";
                                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

                            } else if (best_parameter) {
                                cout << "Optimization complete!" << endl;
                                cout << "best p" << p_best[0] << ", " << p_best[1] << ", " << p_best[2] << endl;
                            }

                            cout << "iteration: " << iteration << endl;
                            cout << "best_error: " << best_error << endl;
                            cout << "current best p up till now: " << p_best[0] << ", " << p_best[1]<< ", "<< p_best[2] << endl;
                            cout << "sum of dp: " << sum_dp << endl;
                            if (first_drive) {
                                cout << "first drive: " << " next p index: "<< p_id << " current p value: " << p[0] << ", " << p[1]<< ", " << p[2] << endl;

                            } else {
                                cout << "second drive: " << " next p index: "<< p_id << " current p value: " << p[0] << ", " << p[1]<< ", " << p[2] << endl;
                            
                            }
                            cout << endl;

                        } else {
                            msgJson["steering_angle"] = steer_value;
                            msgJson["throttle"] = 0.3;
                            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                            // std::cout << msg << std::endl;
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);                            
                        } 

                    } else { // if not twiddle

                        // update the PID and calculate the steer value between [-1,1]
                        pid.UpdateError(cte);
                        steer_value = pid.TotalError();
                        if (steer_value > 1) {
                            steer_value = 1;
                        } else if (steer_value < -1) {
                            steer_value = -1;
                        }

                        // DEBUG
                        std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                                    << std::endl;
                        
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = 0.3;
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        std::cout << msg << std::endl;
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
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