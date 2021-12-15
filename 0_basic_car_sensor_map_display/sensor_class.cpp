
#include "sensor_class.h"

sensor_class::sensor_class(double vision_angle, double max_distance):_vision_angle(vision_angle), _max_distance(max_distance){}
sensor_class::~sensor_class(){}

std::vector<object_position_vector> sensor_class::scan(const map_class &my_map, const cv::Point2f &position, const double &alpha){

    bool debug = false;

    std::vector<object_position_vector> result_list;

    // create a vector in direction of the car
    double alpha0 = to_rad(alpha);
    cv::Vec2f v_car_direction(  cos(alpha0), sin(alpha0));
    //cv::Vec2f v_sensor(-v_car_direction[1],v_car_direction[0]);
    //v_sensor/=norm(v_sensor,cv::NORM_L2);

    if (debug) std::cout << "car direction = " << v_car_direction << std::endl;

    // for each of the cones
    for (auto p:my_map._cone_positions){

        // determine the relative position to the sensor
        cv::Vec2f car_2_cone(p - position);

        // determine the angle relative to car direction, using the dot product to get the cos in deg
        float phi0 = car_2_cone.dot(v_car_direction) / norm(car_2_cone,cv::NORM_L2) / norm(v_car_direction,cv::NORM_L2);
        auto phi = acos(phi0);
        phi = to_deg(phi);

        // determine the distance
        float r = norm(car_2_cone);

        // check if the angle is within the vision of the sensor
        if (phi < _vision_angle){

                // check if within sensor range
               if (r<_max_distance){

                // the dot angle is always positive, ie. it doesnt' tell whether the object is left or right of the car direction
                // create a mat and its determinant, ie. the cross product tells if the object is left or right of the car center
                cv::Mat to_test = (cv::Mat_<double>(2,2) << car_2_cone[0], car_2_cone[1], v_car_direction[0], v_car_direction[1]);
                double left = cv::determinant(to_test);
                // std::cout << "l /r = " << left << std::endl;
                // if (left>0) std::cout << "to the left" << std::endl; else std::cout << "to the right" << std::endl;
                if (signbit(left)!=0) phi = -phi;

                // now add to results
                object_position_vector obj_pv(p, position, alpha, phi, r);

                // std::cout << "cone = " << p << std::endl;
                // std::cout << "car_2_cone = " << car_2_cone << std::endl;
                // std::cout << "phi (sensor, cone) in deg = " <<  phi << std::endl;
                // std::cout << "phi = " << phi0 << std::endl;
                // std::cout << "r (sensor, cone) = " <<  r << std::endl;
                // std::cout << "cone sensor reading  for cone at = " << p << " = " << obj_pose.get_pos_mat() << std::endl;

                result_list.push_back(obj_pv);

            } else if (debug) std::cout << "outside of range " << r << " > " << _max_distance << "   (phi = " << phi << ")" << std::endl;
        } else if (debug) std::cout << "outside of vision angle " << phi << " > " << _vision_angle << "   (r = " << r << ")" << std::endl;
    } // end of loop

    return result_list;
    //return result;

    cv::Mat result;

    // for each of the cones determine the relative position
    for (auto p:my_map._cone_positions){
        std::cout << "cone = " << p << std::endl;
        cv::Vec2f v_car_to_cone( p.x, p.y);
        std::cout << "cone as vec = " << v_car_to_cone << std::endl;
        v_car_to_cone -= cv::Vec2f(position.x,position.y);
        std::cout << "car to cone = " << v_car_to_cone << std::endl;
        float phi = v_car_to_cone.dot(v_car_direction) / norm(v_car_to_cone,cv::NORM_L2) / norm(v_car_direction,cv::NORM_L2);
        std::cout << "phi (sensor, cone) = " <<  phi << std::endl;
        phi = acos(phi) * 180.0 / M_PI;
        float r = norm(v_car_to_cone);
        std::cout << "phi (sensor, cone) = " <<  phi << "   , r = " << r << std::endl;

        // float data[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
        // cv::Mat your_matrix = cv::Mat(1, 10, CV_32F, data);


        // float data[2] = { phi, r };
        cv::Mat rowsy = ( cv::Mat_<double>(1,2) << (double)phi , (double)r );
        std::cout << "cone sensor reading  for cone at = " << p << " = " << rowsy << std::endl;
        result.push_back(rowsy);
    }

    //std::cout << "cone sensor readings = " << result << std::endl;
    //return result;
}

sensor_class::sensor_class(){}
