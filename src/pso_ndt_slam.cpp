#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<time.h>


const int pi = 3.14159;
const float resolution = 0.5;
int beam_quantity;
bool flag1 = false;
bool flag2 = false;

struct float_vec2{
    float x;
    float y;
};

struct cell{
    float_vec2 mean;
    float_vec2 cov;
};

struct position{
    float x;
    float y;
};

struct cell_coor{
    int x;
    int y;
};

struct robot_position{
    position pos;
    float theta;
};

struct ndt_map{
    int height;
    int width;
    cell data[1600];
};

robot_position pose;
position lidar_data[720];
cell_coor lidar_temp[720];

void get_cell_ndt(ndt_map &map);
void mean_cal(std::vector<position> line, ndt_map &map);



void get_lidar_position(const sensor_msgs::LaserScan::ConstPtr& scan){
    if(flag1 == true){
        int num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment)); 
        beam_quantity = num + 1;
        for(int i=0; i < beam_quantity; i++){
            float angle = (pose.theta)*pi/180 + scan->angle_min + i * scan->angle_increment;
            float x = float((scan->ranges[i]*cos(angle)*1 + pose.pos.x + 0.287*cos((pose.theta)/180*pi)));
            float y = float((scan->ranges[i]*sin(angle)*1 + pose.pos.y + 0.287*sin((pose.theta)/180*pi)));
            lidar_data[i].x = x;
            lidar_data[i].y = y;
            lidar_temp[i].x = int(x/resolution);
            lidar_temp[i].y = int(y/resolution);
            // ROS_INFO("%d , %d",int(lidar_data[i].x/resolution),int(lidar_data[i].y/resolution));
        }   
        flag2 = true;
    }
    flag1 = false;
}



void get_robot_position(const nav_msgs::Odometry::ConstPtr& msg){
    pose.pos.x = (20/2) + msg->pose.pose.position.x;
    pose.pos.y = (20/2) + msg->pose.pose.position.y;
    pose.theta = 0; 
    flag1 = true;
}

int main(int argc, char** argv){
    clock_t start, end;   // Khai báo biến thời gian
    double time_use;      // Thời gian sử dụng
    flag1 = false;
    flag2 = false;
    ros::init(argc, argv, "pso_ndt_slam");
    ros::NodeHandle n;
    ros::Publisher map_data = n.advertise<nav_msgs::OccupancyGrid>("map", 50);
    ros::Subscriber lidar_data = n.subscribe("/scan1", 1000, get_lidar_position);
    ros::Subscriber odom_data = n.subscribe("/odom", 1000, get_robot_position);
    nav_msgs::OccupancyGrid map;
    ndt_map mymap;    

    u_int16_t map_size_x = 20;
    u_int16_t map_size_y = 20;
    mymap.width = int(map_size_x/resolution);
    mymap.height = int(map_size_y/resolution);
    map.info.height = 20;
    map.info.width = 20;
    map.info.resolution = 1;
    map.data.resize(1600);
    int total_size = mymap.width * mymap.height;
    for(int i=0; i < 1600; i++){
        mymap.data[i].mean.x = 0;
        mymap.data[i].mean.y = 0;
        mymap.data[i].cov.x = 0;
        mymap.data[i].cov.y = 0;
        map.data[i] = 0;
    }
    // map.info.width = int(map_size_x/resolution);
    // map.info.height = int(map_size_y/resolution);
    // map.info.resolution = resolution;
    int a = 1;
    map.data.resize(total_size);

    for(int i = 0; i < 400; i++){
        mymap.data[i].mean.x = 0;
        mymap.data[i].mean.y = 0;
        mymap.data[i].cov.x = 0;
        mymap.data[i].cov.y = 0;
    }

    while(n.ok()){
        // start = clock(); 
        ros::spinOnce(); // check for incoming messages

        if((flag2 == true) && (a < 2)){
            get_cell_ndt(mymap);
            a++;
            for(int i = 0; i < total_size; i++){
                if(map.data[i] > mymap.data[i].mean.x){
                    map.data[i] = 100;
                }
            }
        }
        map_data.publish(map);
        // end = clock();
        // time_use = (double)(end - start) / CLOCKS_PER_SEC;
        // ROS_INFO("%f", time_use);
    }
}

void get_cell_ndt(ndt_map &map){
    std::vector<position> line;
    std::vector<position> data;
    bool start = true;
    bool start1 = false;
    for(int i = 0; i < beam_quantity; i++){
        // ROS_INFO("%d",i);
        // ROS_INFO("%d , %d",int(lidar_data[i].x/resolution),int(lidar_data[i].y/resolution));
        start1 = false;
        if(start == true){
            for(int j = 0; j < beam_quantity; j++){
                if((lidar_temp[i].x == lidar_temp[j].x) && (lidar_temp[i].y == lidar_temp[j].y)){
                    line.push_back(lidar_data[j]);
                }
            }
            data.push_back(lidar_data[i]);
            i+=line.size();
            start = false;
        }
        else{
            for(int z = 0; z < data.size(); z++){
                if((lidar_temp[i].x != int(data[z].x/resolution)) || (lidar_temp[i].y != int(data[z].y/resolution))){
                    start1 = true;
                }
                else{
                    start1 = false;
                    break;
                }
            }
            if(start1 == true){
                for(int j = 0; j < beam_quantity; j++){ 
                    if((lidar_temp[i].x == lidar_temp[j].x) && (lidar_temp[i].y == lidar_temp[j].y)){
                        line.push_back(lidar_data[j]);
                    }
                }
                data.push_back(lidar_data[i]);

            }
        }
     if(line.size() > 3){
        // ROS_INFO("%ld", line.size());
        mean_cal(line, map);
    }

    line.clear();    
    }
    // ROS_INFO("%ld", data.size());
    data.clear();
    }

void mean_cal(std::vector<position> line, ndt_map &map){
    float mean_x = 0;
    float mean_y = 0;
    int index = int(line[0].x/resolution) + int(line[0].y/resolution) * map.width;
    for(int i = 0; i < line.size(); i++){
        mean_x += line[i].x;
        mean_y += line[i].y;
    }
    map.data[index].mean.x = mean_x/(line.size());
    map.data[index].mean.y = mean_y/(line.size());
}


