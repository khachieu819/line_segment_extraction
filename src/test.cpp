#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<vector>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include <unistd.h>

using namespace std;
int beam_quantity;

float delta = 100;
float epsilon = 30;
float resolution;
bool start = false;
int Np = 50;
int P_min = 10;
int S_num = 6;
float L_min = 60;
double pi = 3.14;
double angle_increment;
double angle_min;

struct endpoint{
    float start_point;
    float end_point;
};

struct least{
    float a;
    float b;
    float c;
};

struct cell{
    int x;
    int y;
};

struct point{
    double x;
    double y;
    int index;
};

struct robot_position{
    cell pos;
    float theta;
};
struct line_segment{
    point start_point;
    point end_point;
    least param;
};

struct make_line_data{
    int start_index;
    int end_index;
};


point lidar_data[50];
std::vector<cell> point_set;
std::vector<endpoint> sets_point;
robot_position robot_pos;
point break_point;


int detech_full_line(std::vector<point> seed_segment, point &break_point, line_segment &segment, nav_msgs::OccupancyGrid &map);
float dist_to_line(least temp, cell point);
least fitting_tagent_line(std::vector<point> point_set);
void seed_segment_detection(point break_point,std::vector<point> &seed_segment);
void Overlap_region_process(std::vector<line_segment> line,std::vector<cell> &line1, nav_msgs::OccupancyGrid &map);
void makeline(cell a, cell b, std::vector<cell> &line, nav_msgs::OccupancyGrid &map);

void get_position_xy(const sensor_msgs::LaserScan::ConstPtr& scan ){
    int num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment)); 
    beam_quantity = num + 1;
    // ROS_INFO("%d",beam_quantity);
    for(int i=0; i < beam_quantity; i++){
        float angle = scan->angle_min + i * scan->angle_increment;
        // ROS_INFO("%f",angle);
        double x = double(scan->ranges[i]*cos(angle)*100 + 500);
        double y = double(scan->ranges[i]*sin(angle)*100 + 500);
        // ROS_INFO("%f",x);
        lidar_data[i].x = x;
        lidar_data[i].y = y;
        lidar_data[i].index = i + 1;
    }
    angle_min= scan->angle_min;
    angle_increment = scan->angle_increment;
    start = true;
}

int main(int argc, char** argv){
    // clock_t start, end;   // Khai báo biến thời gian
    // double time_use;      // Thời gian sử dụng
    ros::init(argc, argv, "line_extraction1");
    ros::NodeHandle n;
    ros::Publisher map_data = n.advertise<nav_msgs::OccupancyGrid>("map", 50);
    ros::Subscriber lidar_data = n.subscribe("/scan1", 1000, get_position_xy);
    nav_msgs::OccupancyGrid map;
    resolution = 4;
    map.info.width = int(1000/resolution);
    map.info.height = int(1000/resolution);
    map.info.resolution = resolution;
    map.data.resize(1000*1000);

    robot_pos.pos.x = 500;
    robot_pos.pos.y = 500;

    ros::Rate r(2);

    point break_point;
    break_point.x = 0;
    break_point.y = 0;
    break_point.index = 0;
    int a = 0;  
    std::vector<line_segment> list_of_line_segment;
    std::vector<cell> line1;

    while(n.ok()){
        start = false;
        int n = 0;
        ros::spinOnce(); // check for incoming messages
        while((start == true) && (break_point.index < (Np - P_min))&&(a<3)){
            // start = clock(); 
            ROS_INFO("--------");
            std::vector<point> seed_segment;
            line_segment segment;
            seed_segment_detection(break_point, seed_segment);
            int v = detech_full_line(seed_segment, break_point, segment, map);
            
            if(v == 1){
            list_of_line_segment.push_back(segment);
            }
            a++;
        }
        // ROS_INFO("%ld",list_of_line_segment.size());
        //  for(int i=0; i < list_of_line_segment.size();i++){
        //     ROS_INFO("%d, %d",list_of_line_segment[i].start_point.index, list_of_line_segment[i].end_point.index);
        // }
        if(break_point.index > (Np - P_min)){
            // Overlap_region_process(list_of_line_segment, line1, map);
            break;
            list_of_line_segment.clear();
            // break_point.index = 0;
        }
        map_data.publish(map);
        // end = clock();
        // time_use = (double)(end - start) / CLOCKS_PER_SEC;
        // ROS_INFO("%f", time_use);
        r.sleep();
        }
}

least fitting_tagent_line(std::vector<point> point_set){
    least temp;
    float sum_x = 0;
    float sum_y = 0;
    float sum_xx = 0;
    float sum_xy = 0;

    int n = point_set.size();
    
    for(int i=0; i < point_set.size();i++){
        ROS_INFO("%f", point_set[i].x);
    }

    for(int i=0 ; i < n; i++){
        sum_x += point_set[i].x;
        sum_y += point_set[i].y;
        sum_xy += (point_set[i].x * point_set[i].y);
        sum_xx += pow(point_set[i].x,2);
    }
    temp.a = float(((n*sum_xy)-(sum_x*sum_y))/((n*sum_xx) - (pow(sum_x,2))));
    temp.c = point_set[2].y - temp.a*point_set[2].x;
    temp.b = -1;
    // ROS_INFO("%f,%f,%f",temp.a, temp.b, temp.c);
    return temp;
}

float dist_point_to_line(least temp, point point){
    ROS_INFO("%f", point.x);
    float ts = abs(temp.a * point.x + temp.b* point.y + temp.c);
    float ms = sqrt(pow(temp.a,2)+pow(temp.b,2));
    return ts/ms;
}

float dist_point_to_point(point point_start, point point_end){
    return float(sqrt(pow(point_start.x - point_end.x,2) + pow(point_start.y - point_end.y,2)));
}


point predicted_point_cal(least temp, double theta){
    point predicted_point;
    predicted_point.x = (-temp.c*cos(theta) - temp.b*(-sin(theta)*500 + cos(theta)*500))/(temp.a*cos(theta) + temp.b*sin(theta));
    predicted_point.y = (-temp.c*sin(theta) + temp.a*(-sin(theta)*500 + cos(theta)*500))/(temp.a*cos(theta) + temp.b*sin(theta));
    return predicted_point;
}

void seed_segment_detection(point break_point,std::vector<point> &seed_segment){
    seed_segment.clear();
    least temp;
    point predicted_point;
    int j;
    int i;
    std::vector<point> seed_segment_test;
    double theta = 0;

    for(i = break_point.index; i < (Np - P_min); i++){
        bool flag = true;
        j = S_num + i;
        for(int c = i ; c < j; c++){
            seed_segment_test.push_back(lidar_data[c]);
        }
        temp = fitting_tagent_line(seed_segment_test);

        for(int k = i; k < j; k++){
            theta = angle_increment*k + angle_min;
            predicted_point = predicted_point_cal(temp, theta);
            ROS_INFO("%f", predicted_point.x);
            double d1 = dist_point_to_point(seed_segment_test[k-i], predicted_point);
            double d2 = dist_point_to_line(temp, seed_segment_test[k-i]);
            ROS_INFO("%f, %f", d1, d2);
            if((d1 > delta) || (d2 > epsilon)){
                flag = false;
                ROS_INFO("out of all condition");
                break;
            }
        }

        if(flag == true)
        { 
            seed_segment = seed_segment_test;  
            for(int i = 0; i< seed_segment.size(); i++){
                ROS_INFO("%d", seed_segment[i].index);
            }
            break;    
        }
    seed_segment_test.clear();
    }
}

int detech_full_line(std::vector<point> seed_segment, point &break_point, line_segment &segment, nav_msgs::OccupancyGrid &map){
    std::vector<point> line;
    least temp;
    line = seed_segment;
    temp = fitting_tagent_line(seed_segment);

    int Pl = 0;
    double Ll = 0;
    int Pf = seed_segment[seed_segment.size()-1].index;
    int Pb = line[0].index - 1;
    while(dist_point_to_line(temp,lidar_data[Pf-1]) < epsilon){
        if(Pf >= Np){
            break;
        }
        else{
                line.push_back(lidar_data[Pf]);
            }
            temp = fitting_tagent_line(line);
            Pf = Pf + 1;
        }
    
    while(dist_point_to_line(temp,lidar_data[Pb]) < epsilon){
        if(Pb < 1){      
            break;
        }
        else{
            line.insert(line.begin(),lidar_data[Pb-1]);
            }
            Pb = Pb - 1;
    }

    for(int i =0; i <line.size();i++){ 
        // int index = int(line[i].x/resolution) + int(line[i].y/resolution) * map.info.width;
        Pl += 1;
    }

    Ll = sqrt(pow(line[line.size()].x - line[0].x,2) + pow(line[line.size()].y - line[0].y,2));
    break_point.index = line[line.size()-1].index;
    if((Pl > P_min) && (Ll > L_min)){
        segment.start_point.x = line[0].x;
        segment.start_point.y = line[0].y;
        segment.start_point.index = line[0].index;

        segment.end_point.x = line[line.size()-1].index;
        segment.end_point.y = line[line.size()-1].index;
        segment.end_point.index = line[line.size()-1].index;

        segment.param.a = temp.a;
        segment.param.b = temp.b;
        segment.param.c = temp.c;
        return 1;
    }
    seed_segment.clear();
    return 0;
}

void Overlap_region_process(std::vector<line_segment> line,std::vector<cell> &line1, nav_msgs::OccupancyGrid &map){
    std::vector<make_line_data> make_line_segment;
    std::vector<cell> make_line_cell;
    sets_point.clear();
    endpoint set_point;
    make_line_data point_segment;
    least temp_1, temp_2;
    std::vector<point> seed1;
    for(int i=0; i < line.size()-1 ; i++){
        bool flag = false;
        int j = i + 1;
        int m1 = line[i].start_point.index; 
        int n1 = line[i].end_point.index;
        int m2 = line[j].start_point.index;
        int n2 = line[j].end_point.index;
        // ROS_INFO("%d,%d,%d,%d",m1, n1, m2, n2);
        temp_1= line[i].param;
        temp_2= line[j].param;
        if(m2 <= n1){
            for(int k = m2; k < n1; k++){
                float d1 = dist_point_to_line(temp_1, lidar_data[k]);
                float d2 = dist_point_to_line(temp_2, lidar_data[k]);
                // ROS_INFO("%f, %f", d1, d2);
                if(d1 > d2){
                    flag = true;
                    break;
                }
                else{
                    n1 = k-1;
                    m2 = k;
                    point_segment.start_index = m1;
                    point_segment.end_index = n1;
                    make_line_segment.push_back(point_segment);
                    // ROS_INFO("%d, %d",m1 , n1);
                    // if( i = line.size()-2){
                    //     point_segment.start.x = lidar_data[n1+1].x;
                    //     point_segment.start.x = lidar_data[n1+1].y;
                    //     point_segment.end.x = lidar_data[n1].x;
                    //     point_segment.end.x = lidar_data[n1].y;
                        
                    // }
                }    
            }
            // if(flag == false){
            //     ROS_INFO("%d",n1);
            //     ROS_INFO("%d",m2);
            //     for(int c = m1; c <= n1; c++){
            //         seed1.push_back(lidar_data[c]);
            //     }
            //     for(int c = m2; c <= n2; c++){
            //         seed2.push_back(lidar_data[c]);
            //     }
            //     ROS_INFO("%d",m1);
            //     ROS_INFO("%d",n1);
     
            //     temp_1 = fitting_tagent_line(seed1);
            //     temp_2 = fitting_tagent_line(seed2);
            //     ROS_INFO("%f, %f, %f",temp_1.a, temp_1.b, temp_1.c);
            //     float x_s_n1 = (pow(temp_1.b,2)*seed1[line.size()-1].x - temp_1.a*temp_1.b*seed1[line.size()-1].y - temp_1.a*temp_1.c)/(pow(temp_1.a,2)+pow(temp_1.b,2));
            //     float y_s_n1 = (pow(temp_1.a,2)*seed1[line.size()-1].y - temp_1.a*temp_1.b*seed1[line.size()-1].x - temp_1.b*temp_1.c)/(pow(temp_1.a,2)+pow(temp_1.b,2));
            //     float x_s_n2 = (pow(temp_2.b,2)*seed2[line.size()-1].x - temp_2.a*temp_2.b*seed2[line.size()-1].y - temp_2.a*temp_2.c)/(pow(temp_2.a,2)+pow(temp_2.b,2));
            //     float y_s_n2 = (pow(temp_2.a,2)*seed2[line.size()-1].y - temp_2.a*temp_2.b*seed2[line.size()-1].x - temp_2.b*temp_2.c)/(pow(temp_2.a,2)+pow(temp_2.b,2));
            //     ROS_INFO("%f %f",x_s_n1,y_s_n1);
            //     ROS_INFO("%f %f",x_s_n2,y_s_n2);
            // }

            }
        }
        // ROS_INFO("%ld", make_line_segment.size());
        for(int i=0; i < make_line_segment.size(); i++){ 
            cell data;
            for(int z = make_line_segment[i].start_index; z <= make_line_segment[i].end_index; z++){
                // ROS_INFO("%d, %d",make_line_segment[i].start_index,make_line_segment[i].end_index );
                seed1.push_back(lidar_data[z]);
                // ROS_INFO("%d",lidar_data[z].index);
            }
            temp_1 = fitting_tagent_line(seed1);
            int x_s = int((pow(temp_1.b,2)*seed1[line.size()-1].x - temp_1.a*temp_1.b*seed1[line.size()-1].y - temp_1.a*temp_1.c)/(pow(temp_1.a,2)+pow(temp_1.b,2))/resolution);
            int y_s = int((pow(temp_1.a,2)*seed1[line.size()-1].y - temp_1.a*temp_1.b*seed1[line.size()-1].x - temp_1.b*temp_1.c)/(pow(temp_1.a,2)+pow(temp_1.b,2))/resolution);
            data.x = x_s;
            data.y = y_s;
            // ROS_INFO("%d",data.x);
            // ROS_INFO("%d",data.y);
            make_line_cell.push_back(data);
            seed1.clear();
            // ROS_INFO("%ld", make_line_cell.size());
        }
        //   ROS_INFO("%ld", make_line_cell.size());
        if(make_line_cell.size() > 2){
            for(int i=0; i < make_line_cell.size()-2; i++){
                int j = i  + 1;
                makeline(make_line_cell[i], make_line_cell[j], line1, map);
            }
        }

        make_line_cell.clear();
     }
        
        

void makeline(cell a, cell b, std::vector<cell> &line, nav_msgs::OccupancyGrid &map){
    line.clear();
	cell cell;
	if (b.y>a.y){
		cell = a;
		a = b;
		b = cell;
	}
	if ((b.y != a.y)&&(a.x>b.x)) for(int i = b.y;i < a.y;i++){

 		for (int j = (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-(i+1))/(a.y-b.y)));j++){
 			cell.y = i;
 			cell.x = j;
 			line.push_back(cell);
 		}
 	}
 	else if ((b.y != a.y)&&(a.x<=b.x)) for(int i=b.y;i<a.y;i++){
 		for (int j = (a.x + ((b.x-a.x)*(a.y-i-1)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j++){
 			cell.y = i;
 			cell.x = j;
 			line.push_back(cell);
 		}

 	}
 	else {
 		if(a.x<b.x){
 			for(int i =a.x;i<=b.x;i++){
 				cell.y = a.y;
 				cell.x = i;
 				line.push_back(cell);
 			}
 		}
 			else if(a.x>=b.x){
 				for(int i = b.x;i<=a.x;i++){
 					cell.y = a.y;
 					cell.x = i;
 					line.push_back(cell);
 				}

 			}

 	}
    // ROS_INFO("%ld",line.size());
    for(int i = 0; i < line.size(); i++){
        map.data[line[i].x + line[i].y * map.info.width] = 100;
        // ROS_INFO("ff");
    }
 }