#include <Lidar_ERP_2024/Lidar_declare.h>


void Parsing_GE(const sensor_msgs::PointCloud2ConstPtr& scan){

    RT::start();
    if(Sensor_Hesai == true){
        Ground_Extraction_H(scan);
    }
    else{
        Ground_Extraction(scan);
    }
    RT::end_cal("GroundExtraction");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "GE_node");
    ros::NodeHandle nh;
    nh.getParam("/GE_node/Sensor_Hesai", Sensor_Hesai);
    nh.getParam("/GE_node/GE_Z", GE_Z);
    nh.getParam("/GE_node/GE_Z_re", GE_Z_re);
    nh.getParam("/GE_node/GE_slope", GE_slope);
    nh.getParam("/GE_node/GE_delta_slope", GE_delta_slope);

    ros::Subscriber sub_rawdata = nh.subscribe<sensor_msgs::PointCloud2>("/0_0_UDP_rawdata", 1, Parsing_GE);
    // ros::Subscriber sub_rawdata = nh.subscribe<sensor_msgs::PointCloud2>("/0_UDP_rawdata",1,Parsing_GE);

    pub_others = nh.advertise<sensor_msgs::PointCloud2>("/0_1_GE_others", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/0_2_GE_ground", 1);
    ros::spin();

    return 0;
    
}