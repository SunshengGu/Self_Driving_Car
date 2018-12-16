[ 33%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
/home/workspace/CarND-PID-Control-Project/src/main.cpp: In lambda function:
/home/workspace/CarND-PID-Control-Project/src/main.cpp:86:31: warning: comparison between signed and unsigned integer expressions [-Wsign-compare]
             for (int i = 0; i < controls.size(); i++) {
                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:68:18: warning: unused variable 'speed' [-Wunused-variable]
           double speed = std::stod(j[1]["speed"].get<std::string>());
                  ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:69:18: warning: unused variable 'angle' [-Wunused-variable]
           double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                  ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:173:4: error: expected primary-expression before ')' token
   });
    ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:177:3: error: 'h' is not captured
   h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
   ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:190:3: error: 'h' is not captured
   h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
   ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:190:20: error: 'h' is not captured
   h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
                    ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:194:3: error: 'h' is not captured
   h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
   ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:194:23: error: 'h' is not captured
   h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
                       ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:200:7: error: 'h' is not captured
   if (h.listen(port))
       ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:209:3: error: 'h' is not captured
   h.run();
   ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:50:79: note: the lambda has no capture-default
   h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *dat
                                                                               ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:34:12: note: 'uWS::Hub h' declared here
   uWS::Hub h;
            ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp:210:1: error: expected '}' at end of input
 }
 ^
/home/workspace/CarND-PID-Control-Project/src/main.cpp: In function 'int main()':
/home/workspace/CarND-PID-Control-Project/src/main.cpp:210:1: error: expected ')' at end of input
/home/workspace/CarND-PID-Control-Project/src/main.cpp:210:1: error: expected '}' at end of input
CMakeFiles/pid.dir/build.make:86: recipe for target 'CMakeFiles/pid.dir/src/main.cpp.o' failed
make[2]: *** [CMakeFiles/pid.dir/src/main.cpp.o] Error 1
CMakeFiles/Makefile2:67: recipe for target 'CMakeFiles/pid.dir/all' failed
make[1]: *** [CMakeFiles/pid.dir/all] Error 2
Makefile:83: recipe for target 'all' failed
make: *** [all] Error 2
