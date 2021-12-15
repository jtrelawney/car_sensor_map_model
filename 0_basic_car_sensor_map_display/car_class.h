#ifndef CAR_CLASS
#define CAR_CLASS

#include <iostream>
#include "opencv2/opencv.hpp"

#include "sensor_class.h"
#include "support.h"

// defines the data type for the position in the map
// often troubling: calculations between cv:Point and cv::Vec and CV::Mat
// see mosaic for details:
// 1. convert Point to Mat with cv::Matx21f(point) to make arith. with vector and mat work
// 2. convert Vector to Point with cv::Point2f(vec) to make arith. with Point work
// hence chose piont a s position type
using POSE_POSITION_TYPE = cv::Point2f;
//using POSE_POSITION_TYPE = cv::Vec2f;
using PPType = POSE_POSITION_TYPE;

// keeps the pose, ie. the position (as Point) and the direction (alpha)
// also keep redundant x,y position for faster access (no need to extract x,y from point)
class pose_class{
public:
    pose_class(PPType pos, double alpha):_position(pos),_alpha(alpha),_x(pos.x),_y(pos.y){};
    ~pose_class(){};

    // update the pose by a relative motion given by vector and the additive angle
    void update(cv::Vec2f distance, double delta_alpha){
        _position.x+=distance[0];
        _position.y+=distance[1];
        _x = _position.x;
        _y = _position.y;
        _alpha+=delta_alpha;
    }

    PPType position(){ return _position; };
    double alpha(){ return _alpha; };

    void get(PPType &position, double &alpha){ position = _position; alpha = _alpha; };
    void get(double &x, double &y, double &alpha){ x = _x; y = _y; alpha = _alpha; };

private:
    //private default constructor -> if used by other objects then initialize in their default constructor list
    pose_class(){};

    PPType _position;
    double _alpha;
    double _x,_y;
};


class car_class{
public:
    car_class(PPType start_pos, double start_alpha, cv::Size car_size);
    ~car_class();

    // car parameters
    cv::Size _car_size;

    // RC input paremeters (max settings)
    double _max_steering_angle, _max_velocity;
    double _min_steering, _max_steering, _min_thrust, _max_thrust;

    // rc inputs, real from device
    double _thrust, _steering;

    // resulting from rc input: car velocity in direction of car in m/s and its x.y components
    double _car_velocity, _vel_x, _vel_y;

    // resulting from rc input: turning angle in deg/s i.e. the steering angle of the front wheels
    double _steering_angle;

    // updates the state after delta t s
    void update_state(double dt);

    // set thrust and steering according to input from rc signals, motor pwm signals to thrust and steering
    void set_thrust(double thrust);
    void set_steering(double steering);

    // read car kinemeatics
    double read_velocity();
    double read_steering_angle();

    // read current pose
    pose_class pose() const;

    std::vector<object_position_vector> sensor_scan(map_class my_map);

private:
    car_class();

    // linear range of steering input over the max possible range
    void update_steering_angle();

    // linear range of thrust input over the max possible range
    void update_velocity();

    // car pose ,ie. position and alpha, (result of inputs and sequential state)
    pose_class _pose;

    // and the sensor
    sensor_class _sensor;
};

#endif
