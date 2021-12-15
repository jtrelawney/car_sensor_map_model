#include "visualize_map_class.h"

visualize_map_class::visualize_map_class(const map_class &my_map, int resolution):orig_map_size(my_map.get_map_size()),_resolution(resolution){

        std::cout << "creating visualzation object for map size = " << orig_map_size << " and resolution = " << _resolution << " dots per unit."<< std::endl;

        // creates an empty map template with borders, this template will be used to reset the plot with init_map
        _map_image = cv::Mat::zeros(orig_map_size * _resolution, CV_8U);
        cv::line( _map_image, cv::Point2i(0,0), cv::Point2i(orig_map_size.width*_resolution,0),255,10);
        cv::line( _map_image, cv::Point2i(orig_map_size.width*_resolution,0), cv::Point2i(orig_map_size.width*_resolution,orig_map_size.height*_resolution),255,10);
        cv::line( _map_image, cv::Point2i(orig_map_size.width*_resolution,orig_map_size.height*_resolution), cv::Point2i(0,orig_map_size.height*_resolution),255,10);
        cv::line( _map_image, cv::Point2i(0,orig_map_size.height*_resolution), cv::Point2i(0,0),255,10);
};

visualize_map_class::~visualize_map_class(){};

// resets the map, uses the template prepared in the constructor
void visualize_map_class::init_map(cv::Mat &map_image){
    map_image = _map_image.clone();
};

// plots a scan result (from the data stored in object)
void visualize_map_class::plot_scan(cv::Mat &map_image,car_class my_car, object_position_vector object){
    PPType car_pos = object._car_position;
    PPType cone_pos = object._cone_position;
    car_pos = car_coordinates_to_opencv_coordinates(car_pos);
    cone_pos = car_coordinates_to_opencv_coordinates(cone_pos);
    cv::line(map_image, car_pos*_resolution, cone_pos * _resolution,255,1);
}

// this plots a car given the car parameters and its pose
void visualize_map_class::plot_car(cv::Mat &map_image, const car_class &my_car){

    // 1. transfer car position from car / map cordinates to opencv coordinates   (x is ok, but y is inverted
    PPType opencv_car_position = my_car.pose().position();
    opencv_car_position = car_coordinates_to_opencv_coordinates(opencv_car_position);

    // mark the position of the car
    cv::circle(map_image, opencv_car_position * _resolution,  _resolution, 255, 1 );

    // 2. get the direction of the car and create the rotation matrix
    double alpha = my_car.pose().alpha();
    alpha = to_rad(alpha);

    // rot mat from car angle
    cv::Mat rotation_matrix = (
        cv::Mat_<float>(2,2) <<
            cos(alpha), -sin(alpha),
            sin(alpha), cos(alpha)
        );

    // 3. get the 4 cornerpoints of the car as well as the direction triangle in a mat
    cv::Mat points = get_car_mat(my_car);
    points = points.reshape(1); // it somehow creates 2 channels, make it one channel
    //std::cout << points << std::endl;
    //print_image_info(points,"points");

    // 4 rotate the 4 car corner points and plot the frame
    cv::Mat rotated = points * rotation_matrix;
    //std::cout << rotated << std::endl;

    // plot car frame from first 4 points in mat
    cv::Point2f recent, first;
    for (int i=0; i<4; i++){
        auto r = rotated.row(i);
        //std::cout << r << std::endl;
        cv::Point2f p(r);
        p+=opencv_car_position;
        //std::cout << p << std::endl;
        if (i>0) {
            //cv::circle(map_image, p * _resolution,  _resolution, 255, 1 );
            cv::line(map_image, recent*_resolution, p*_resolution, 255,1);
        } else first = p;
        recent = p;
    }
    // close the frame
    cv::line(map_image, recent*_resolution, first*_resolution, 255,1);

    // 5. extract the direction arrow points and plot them
    cv::Point2f apc(rotated.row(4));
    cv::Point2f apl(rotated.row(5));
    cv::Point2f apr(rotated.row(6));
    cv::line(map_image, (apc+opencv_car_position)*_resolution, (apl+opencv_car_position)*_resolution, 255,1);
    cv::line(map_image, (apl+opencv_car_position)*_resolution, (apr+opencv_car_position)*_resolution, 255,1);
    cv::line(map_image, (apr+opencv_car_position)*_resolution, (apc+opencv_car_position)*_resolution, 255,1);

    // 6. print the key variables into string stream (to round numbers) and then transfer into image
    float font_size = 0.7;
    int x_offset = 30;
    int row_distance = 20, y_offset;

    // for rounded strings
    std::stringstream ss;
    std::string rounded_string;

    ss << std::fixed << std::setprecision(2) <<  my_car._steering_angle;
    rounded_string = ss.str();
    std::string this_text = "steer   = " + rounded_string;//std::to_string( round( my_car._steering_angle * 100.0) / 100.0 );

    //std::string this_text = "steering_angle = " + std::to_string( std::round(my_car._steering_angle) ); //round( my_car._steering_angle * 100.0) / 100.0 );
    y_offset = row_distance;
    put_text(map_image, this_text, x_offset,y_offset, cv::Scalar(255,255,255), font_size);

    ss.clear();
    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) << to_deg(alpha);
    rounded_string = ss.str();
    this_text = "alpha  = " + rounded_string;//std::to_string(alpha*180.0/3.141);
    //this_text = "alpha = " + std::to_string( std::round(to_deg(alpha)) );
    y_offset += row_distance;
    put_text(map_image, this_text, x_offset,y_offset, cv::Scalar(255,255,255), font_size);

    ss.clear();
    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) <<  my_car._car_velocity;
    rounded_string = ss.str();
    this_text = "vel     = " + rounded_string;//std::to_string(vel);
    y_offset += row_distance;
    put_text(map_image, this_text, x_offset,y_offset, cv::Scalar(255,255,255), font_size);

    ss.clear();
    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) <<  my_car._vel_x << "  ,  " << my_car._vel_y;
    rounded_string = ss.str();
    this_text = "vel x,y = " + rounded_string; //std::to_string() + "," + std::to_string();
    y_offset += row_distance;
    put_text(map_image, this_text, x_offset,y_offset, cv::Scalar(255,255,255), font_size);

    ss.clear();
    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) <<  opencv_car_position.x << "  ,  " << opencv_car_position.y;
    rounded_string = ss.str();
    y_offset += row_distance;
    put_text(map_image, this_text, x_offset,y_offset, cv::Scalar(255,255,255), font_size);
}

visualize_map_class::visualize_map_class(){};

// this plot the obstacles as defined in the map
void visualize_map_class::plot_obstacles(cv::Mat &map_image, const std::vector<cv::Point2f> &cone_positions){
    for (auto p:cone_positions){
        PPType opencv_position = car_coordinates_to_opencv_coordinates(p);
        cv::circle(map_image, opencv_position * _resolution,  _resolution, 255, 3 );
    }
};

// from the car parameters finds the 4 corners of the car, as well as the direction arrow and returns all of them in a mat for easy rotation
cv::Mat visualize_map_class::get_car_mat(const car_class &my_car){

    // fetch the car parameters related to size and create list of points that describe car in neutral position (ie. alpha = 0 -> to rhte right)
    float width = my_car._car_size.width;
    float length =  my_car._car_size.height;

    // calculate 4 car corners, in direction of principle coordinate system, ie. angle = 0 -> right
    cv::Point2f front_l = cv::Point2f(length / 2, -width/2);
    cv::Point2f front_r = cv::Point2f(length / 2, +width/2);
    cv::Point2f back_r = cv::Point2f(-length / 2, +width/2);
    cv::Point2f back_l = cv::Point2f(-length / 2, -width/2);

    // calculate the arrow to show direction of car
    int arrow_y_offset = length / 3.0;
    cv::Point2f arrow_pos_center = cv::Point2f(length/2.0, 0.0);
    cv::Point2f arrow_pos_left  = cv::Point2f(length/2.0 - arrow_y_offset , -width/2.0);
    cv::Point2f arrow_pos_right = cv::Point2f(length/2.0 - arrow_y_offset , +width/2.0);

    cv::Mat points;
    points.push_back(front_l);
    points.push_back(front_r);
    points.push_back(back_r);
    points.push_back(back_l);
    points.push_back(arrow_pos_center);
    points.push_back(arrow_pos_left);
    points.push_back(arrow_pos_right);

    return points;
}

// coordinate transform from car to opencv (opencv origin in top left -> x stays the same, but the y coordinate is "inverted"), angle considerations are the same
PPType visualize_map_class::car_coordinates_to_opencv_coordinates(const PPType &from_position){
    PPType to_position(from_position);
    to_position.y = orig_map_size.height - to_position.y;
    return to_position;
};
