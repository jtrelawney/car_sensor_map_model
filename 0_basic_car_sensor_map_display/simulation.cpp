
#include <iostream>
#include <math.h>

// classes for simulation
#include "map_class.h"
#include "visualize_map_class.h"
#include "car_class.h"

// helper code (display, files, images etc)
#include "support.h"
#include "file_operations.h"
#include "image_operations.h"
#include "display_manager_class.h"
#include "opencv2/opencv.hpp"


void display_purpose(){
    std::cout <<"simulates the car" << std::endl;
    std::cout <<"esck                     : stop program" << std::endl;
    //
    std::cout <<"blank                    : sets alpha to neutral" << std::endl;
    // std::cout <<"t                        : iterates through the cone templates" << std::endl;
    // std::cout <<"cone low and cone high   : the color thresholds for the cones" << std::endl;
    // std::cout <<"equalize                 : apply image equalizion - on / off" << std::endl;
    // std::cout <<"template                 : use a manual template or the cropped cone image as template - on / off" << std::endl;
    // std::cout <<"binarize                 : transform grayimage to binary image using the thresholds - on / off" << std::endl;
    // std::cout <<"\npress key to continue" << std::endl;

    std::getchar();
}

int main(int, char**){

    display_purpose();

    // those are RC inputs
    int speed=0;
    int alpha=300;

    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar( "speed", "map", &speed, 300, NULL);
    cv::createTrackbar( "alpha", "map", &alpha, 600, NULL);

    map_class my_map(100,100);
    my_map.add_cone(cv::Point2i(50,70));
    my_map.add_cone(cv::Point2i(80,70));
    my_map.add_cone(cv::Point2i(80,50));

    // start car at origin, with orientation to right
    car_class my_car(cv::Point2i(50,50), 0.0, cv::Size(5,10));
    visualize_map_class vm(my_map,10);

    cv::Mat map_image;

    bool accelerate_once = true;

    while(1){

        double delta_t = 0.1;

        vm.init_map(map_image); // reset the map
        vm.plot_car(map_image, my_car); // draws the car in position with proper direction
        vm.plot_obstacles(map_image, my_map._cone_positions);

        // read the scanner to find objects and plot the signals
        std::vector<object_position_vector> objects = my_car.sensor_scan(my_map);
        for (auto opv:objects){
            vm.plot_scan(map_image, my_car, opv);
        }

        cv::imshow("map", map_image);

        if (accelerate_once) {
            my_car.set_thrust(1500+speed);    // 1800 = 100/100 of max range
            my_car.set_steering(1500+alpha - 300 ); // 1530 = 1/10 of max range
            my_car.update_state(delta_t);
            // with trackbars keep updating
            //accelerate_once=false;
        } else my_car.update_state(delta_t);

        char c = (char) cv::waitKey(25);
        // esc ends the show
        if(c==27) break;
        // next frame
        else if (c==32) {
            //std::cout << "press t" << std::endl;
            alpha = 300;
            //if ( frame_counter < ir_images.get_image_count()-1 ) frame_counter++; else frame_counter = 0;
        }
        else if (c==116) {
            //std::cout << "press t" << std::endl;
            //if ( cone_counter < ir_cones.get_image_count()-1 ) cone_counter++; else cone_counter = 0;
        }
    }

    return 0;
}
