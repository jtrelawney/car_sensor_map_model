#ifndef SUPPORT
#define SUPPORT

#include "opencv2/opencv.hpp"

double to_rad(double angle);
double to_deg(double angle);

// global variables, necessary to make the image sliders work
//hsv parameters preset
// int filter_hue_low_tracker = 25, filter_hue_low;
// int filter_hue_high_tracker = 150, filter_hue_high;
//
// int filter_sat_low_tracker = 90, filter_sat_low;
// int filter_sat_high_tracker = 225, filter_sat_high;
//
// int filter_val_low_tracker = 110, filter_val_low;
// int filter_val_high_tracker = 140, filter_val_high;
//
// int detection_threshold_tracker = 50;
// double detection_threshold;
//
// // call back functions for the sliders, only necessary when they should ajdust / display something
// static void callback_hsv_hue_low( int, void* ){ filter_hue_low = filter_hue_low_tracker; }
// static void callback_hsv_hue_high( int, void* ){ filter_hue_high = filter_hue_high_tracker; }
//
// static void callback_hsv_sat_low( int, void* ){ filter_sat_low = filter_sat_low_tracker; }
// static void callback_hsv_sat_high( int, void* ){ filter_sat_high = filter_sat_high_tracker; }
//
// static void callback_hsv_val_low( int, void* ){ filter_val_low = filter_val_low_tracker; }
// static void callback_hsv_val_high( int, void* ){ filter_val_high = filter_val_high_tracker; }
//
// static void callback_detection_threshold( int, void* ){
//     detection_threshold = double(detection_threshold_tracker) / 100.0;
//     // avoids to low threshold
//     if (detection_threshold <0.1) detection_threshold = 0.1;
// }
//
#endif
