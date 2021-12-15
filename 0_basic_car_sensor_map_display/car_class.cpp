#include "car_class.h"

std::vector<object_position_vector> car_class::sensor_scan(map_class my_map){
    return _sensor.scan(my_map, _pose.position(), _pose.alpha());
}

car_class::car_class(PPType start_pos, double start_alpha, cv::Size car_size)

            // car parameters: size and the max steering angle in deg / s and max car speed in m / ss
            :_car_size(car_size), _max_steering_angle(30.0), _max_velocity(10.0),
            // rc input signals, max range
            _min_steering(1200.0), _max_steering(1800.0), _min_thrust(1200.0), _max_thrust(1800.0),
            // the initial thrust and steering settings, neutral settings with stationary car
            _thrust(1500.0), _steering(1500.0),

            // the cars actual velocity in m/s = function of thrust, in direction of car
            _car_velocity(0.0), _vel_x(0.0), _vel_y(0.0),
            // the cars actual steering angle in deg/s, function of steering,
            _steering_angle(0.0),
            // the pose
            _pose( pose_class(start_pos,start_alpha) ),
            // and create the sensor, currently not parameterized
            _sensor(sensor_class(45,25))
{
    // year, code here ...
};

// moves the car according to the current state (position, velocity, direction), for time interval dt
void car_class::update_state(double dt) {

    bool debug = false;

    if (debug) std::cout << "===============================" << std::endl;
    if (debug) std::cout << "kinematic update = " << std::endl;
    if (debug) std::cout << "current p = " << _pose.position() << "  a = " << _pose.alpha() << std::endl;

    // the car has moved for time = dt with the current velocity, in the direction of the alpha in the pose, from the position in the pose
    // { _car_velocity and _steering angle are set independently outside this function by RC calls }

    // fetch the car direction (alpha) and calculate the vx and vy componets
    double alpha1 = to_rad(_pose.alpha());
    _vel_x = _car_velocity * cos(alpha1);
    _vel_y = _car_velocity * sin(alpha1);

    if (debug) std::cout << "current v = " << _car_velocity << "   vx, vy = " << _vel_x << "," << _vel_y << std::endl;

    // now update the pose by traveling into the vx, vy direction for time dt
    // and also update the car direction by delta steering angle
    _pose.update(cv::Vec2f(dt * _vel_x, dt * _vel_y), dt*_steering_angle);

    if (debug) std::cout << "updated p = " << _pose.position() << "  a = " << _pose.alpha() << std::endl;
}

// RC input signal
// linear range of steering input over the max possible range
void car_class::update_steering_angle() {
    double help = _steering - 1500.0;
    _steering_angle =  help / 300.0 * _max_steering_angle;
};

// RC input signal
// linear range of thrust input over the max possible range
void car_class::update_velocity() {
    double help = _thrust - 1500.0;
    _car_velocity = help / 300.0 * _max_velocity;
};

// process input from rc signals, motor pwm signals to thrust and steering
void car_class::set_thrust(double thrust)  {
    _thrust=thrust;
    if (_thrust>_max_thrust) _thrust = _max_thrust;
    if (_thrust<_min_thrust) _thrust = _min_thrust;
    update_velocity();
}

void car_class::set_steering(double steering) {
    _steering=steering;
    if (_steering>_max_steering) _steering = _max_steering;
    if (_steering<_min_steering) _steering = _min_steering;
    update_steering_angle();
}

double car_class::read_velocity() {  return _car_velocity; };
double car_class::read_steering_angle() {  return _steering_angle; };
pose_class car_class::pose() const { return _pose; };

car_class::~car_class(){}
