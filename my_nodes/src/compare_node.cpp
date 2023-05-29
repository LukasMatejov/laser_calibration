
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <unistd.h>

ros::Subscriber hokuyo_subscriber;
ros::Subscriber lidarrp_subscriber;

double hokuyo_measurement_1;
double hokuyo_measurement_2;
double lidarrp_measurement_1;
double lidarrp_measurement_2;

float rotation_angle_hokuyo;
float rotation_angle_lidarrp;

float angle_increment_hokuyo;
float angle_increment_lidarrp;
double real_angle_lidarrp;
double real_angle_hokuyo;
double real_angle_lidarrp2;
double real_angle_hokuyo2;

double correct_alfal;
double real_rad_hokuyo;
double real_rad_hokuyo2;

double real_rad_lidarrp;
double real_rad_lidarrp2;


float real_angle_lidarrp_div;
float real_angle_hokuyo_div;

float kh1;
float alfah;
float alfah_angle;

float kl1;
float alfal;
float alfal_angle;
float correct_alfal_angle;

float kh1t;
float kh2t;
float kl1t;
float kl2t;
float kl_corretion;


float yh1;
float xh1;
float yh2;
float xh2;

float yl1;
float xl1;
float yl2;
float xl2;

float qh;
float qh1;
float qh2;

float ql;
float ql1;
float ql2;

float correction_angle;

float laser1 = 276;
float laser2 = 296;
float laser3 = 181;
float laser4 = 172;


float pi = 22 / 7;

float calibration_factor_hokuyo;
float calibration_offset_hokuyo;
double senzor_off_set = 0.03;
double reference_distance_1 = 0.35;
double reference_distance_2 = 0.4;
double calibration_offset_correction_lidarrp;
double calibration_offset_lidarrp;
double calibration_factor_correction_lidarrp;
double calibration_factor_lidarrp;

double reference_angle = -90;
double reference_angle2 = -94;
float kr1;
float yr1;
float xr1;
float yr2;
float xr2;
float qr;

float alfa_reference;
float alfa_reference_angle;


// Function to calibrate the sensors
void calibrate_sensors() {
    // usleep(500000);

    //  reference for calibration
    yr1 = reference_distance_1 * sin(reference_angle);
    xr1 = reference_distance_1 * cos(reference_angle);
    yr2 = reference_distance_2 * sin(reference_angle2);
    xr2 = reference_distance_2 * cos(reference_angle2);

    kr1 = (yr2 - yr1) / (xr2 - xr1);

    alfa_reference = atan(kr1);
    alfa_reference_angle = alfa_reference * (180 / pi);



    // Calculate calibration offsets
    calibration_offset_lidarrp = (alfa_reference_angle - correct_alfal_angle);
    calibration_offset_correction_lidarrp = (alfa_reference_angle - alfal_angle);
    calibration_offset_hokuyo = alfa_reference_angle - alfah_angle;

    // Calculate calibration factors

    calibration_factor_lidarrp = alfa_reference_angle / alfal_angle;
    calibration_factor_correction_lidarrp = alfa_reference_angle / correct_alfal_angle;
    calibration_factor_hokuyo = alfa_reference_angle / alfah_angle;


    // Equation of line for Hokuyo
    real_rad_hokuyo = angle_increment_hokuyo * laser1 * (-1);
    real_rad_hokuyo2 = angle_increment_hokuyo * laser2 * (-1);
    real_angle_hokuyo = real_rad_hokuyo * (180 / pi);
    real_angle_hokuyo2 = real_rad_hokuyo2 * (180 / pi);


    kh1t = tan(real_rad_hokuyo);
    kh2t = tan(real_rad_hokuyo2);

    yh1 = hokuyo_measurement_1 * sin(real_rad_hokuyo);
    xh1 = hokuyo_measurement_1 * cos(real_rad_hokuyo);


    yh2 = hokuyo_measurement_2 * sin(real_rad_hokuyo2);
    xh2 = hokuyo_measurement_2 * cos(real_rad_hokuyo2);

    kh1 = (yh2 - yh1) / (xh2 - xh1);
    alfah = atan(kl1);


    qh1 = yh1 - kh1t * xh1;
    qh2 = yh2 - kh2t * xh2;


    alfah = atan(kh1);


    qh = yl1 - kh1 * xh1;

    alfah_angle = alfah * (180 / pi);


    real_angle_hokuyo_div = real_angle_hokuyo - real_angle_hokuyo2;

    // Equation of line for Lidar
    real_rad_lidarrp = angle_increment_hokuyo * laser3;
    real_rad_lidarrp2 = angle_increment_hokuyo * laser4;

    real_angle_lidarrp = real_rad_lidarrp * (180 / pi);
    real_angle_lidarrp2 = real_rad_lidarrp2 * (180 / pi);

    yl2 = lidarrp_measurement_2 * sin(real_rad_lidarrp2);
    xl2 = lidarrp_measurement_2 * cos(real_rad_lidarrp2);

    yl1 = lidarrp_measurement_1 * sin(real_rad_lidarrp);
    xl1 = lidarrp_measurement_1 * cos(real_rad_lidarrp);


    kl1t = tan(real_rad_lidarrp);
    kl2t = tan(real_rad_lidarrp2);

    ql1 = yl1 - kl1t * xl1;
    ql2 = yl2 - kl2t * xl2;

    kl1 = (yl2 - yl1) / (xl2 - xl1);
    alfal = atan(kl1);

    ql = yl1 - kl1 * xl1;

    alfal_angle = alfal * (180 / pi);


    real_angle_lidarrp_div = real_angle_lidarrp2 - real_angle_lidarrp;

    // calibration of lidar
    correction_angle = alfal_angle - alfa_reference_angle;

    correct_alfal_angle = alfal_angle - 1.1;






    // Display calibration results
    ROS_INFO("------------------------------------------------");
    ROS_INFO("------------------------------------------------");
    // ROS_INFO("Calibration parameters for Hokuyo URG sensor:");
    // ROS_INFO("LidarRP rotation angle %.5f", rotation_angle_hokuyo);
    ROS_INFO("Hokuyo measurement %.5f", hokuyo_measurement_1);
    ROS_INFO("Hokuyo measurement %.5f", hokuyo_measurement_2);

    //   ROS_INFO("equation of line measurement 1    %.4f = %.4f * %.4f + %.9f", yh1 ,kh1t, xh1, qh1);
    //  ROS_INFO("equation of line measurement 2    %.4f = %.4f * %.4f + %.9f", yh2 , kh2t , xh2, qh2);
    // ROS_INFO("equation of line for alfa         %.4f = %.4f * %.4f + %.9f", yh1 , kh1 , xh1, qh);

    ROS_INFO("------------------------------------------------");
    ROS_INFO("Calibration parameters for LidarRP sensor:");
    // ROS_INFO("LidarRP rotation angle %.5f", rotation_angle_lidarrp);
    ROS_INFO("LidarRP measurement %.5f", lidarrp_measurement_1);
    ROS_INFO("LidarRP measurement %.5f", lidarrp_measurement_2);

    // ROS_INFO("equation of line measurement 1   %.4f = %.4f * %.4f +%.9f", yl1 , kl1t , xl1,ql1);
    // ROS_INFO("equation of line measurement 2   %.4f = %.4f * %.4f +%.9f", yl2 , kl2t , xl2,ql2);
    // ROS_INFO("equation of line for alfa        %.4f = %.4f * %.4f + %.9f", yl1 , kl1 , xl1, ql);

    ROS_INFO("------------------------------------------------");

    ROS_INFO("%.8f real_angle_lidarrp", real_angle_lidarrp);
    ROS_INFO("%.8f real_angle_lidarrp2", real_angle_lidarrp2);
    //ROS_INFO( "%.8f real_angle_lidarrp_div",real_angle_lidarrp_div);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("%.8f real_angle_hokuyo", real_angle_hokuyo);
    ROS_INFO("%.8f real_angle_hokuyo2", real_angle_hokuyo2);
    //  ROS_INFO( "%.8f real_angle_hokuyo_div",real_angle_hokuyo_div);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("%.8f   Alfa angle for Hokuyo", alfah_angle);
    ROS_INFO("%.8f   Alfa angle for lidar", alfal_angle);
    ROS_INFO("%.8f   Alfa angle  reference", alfa_reference_angle);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("%.8f   Alfa angle_correct for lidar", correct_alfal_angle);
    ROS_INFO("%.8f   Alfa correction angle for lidar", correction_angle);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("Calibration Offset_corretion: %.4f", calibration_offset_correction_lidarrp);
    ROS_INFO("Calibration Factor_correction: %.4f", calibration_factor_correction_lidarrp);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("Calibration Offset_lidarrp: %.4f", calibration_offset_lidarrp);
    ROS_INFO("Calibration Factor_lidarrp: %.4f", calibration_factor_lidarrp);
    ROS_INFO("------------------------------------------------");
    ROS_INFO("Calibration Factor_hokuyo: %.4f", calibration_offset_hokuyo);
    ROS_INFO("Calibration Factor_hokuyo: %.4f", calibration_factor_hokuyo);


}


void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Retrieve measurement from Hokuyo URG sensor
    hokuyo_measurement_1 = msg->ranges[laser1] - senzor_off_set;
    angle_increment_hokuyo = msg->angle_increment;
    rotation_angle_hokuyo = msg->angle_min;


    hokuyo_measurement_2 = msg->ranges[laser2] - senzor_off_set;
    // Perform calibration when both sensor measurements are available
    if (hokuyo_measurement_1 != 0 && lidarrp_measurement_1 != 0)
        calibrate_sensors();
}

void lidarrp_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // Retrieve measurement from LidarRP sensor
    lidarrp_measurement_1 = msg->ranges[laser3];
    angle_increment_lidarrp = msg->angle_increment;
    rotation_angle_lidarrp = msg->angle_min;

    lidarrp_measurement_2 = msg->ranges[laser4];
    // Perform calibration when both sensor measurements are available
    if (hokuyo_measurement_1 != 0 && lidarrp_measurement_1 != 0)
        calibrate_sensors();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "compare_node");
    ros::NodeHandle nh;

    // Subscribe to the Hokuyo URG sensor topic
    hokuyo_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/hokuyo_scan", 1, hokuyo_callback);

    // Subscribe to the LidarRP sensor topic
    lidarrp_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidarrp_callback);

    ros::spin();

    return 0;
}