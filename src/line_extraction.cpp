#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<vector>
#include<sensor_msgs/LaserScan.h>
#include<nav_msgs/OccupancyGrid.h>
#include <unistd.h>

u_int16_t Np = 100;
u_int8_t P_min = 10;
u_int8_t S_num = 6;
float L_min = 60;
float delta = 10;
float epsilon = 3;
double pi = 3.14;
int break_point = 0;

struct point{
    float x;
    float y;
};

struct cell{
    int x;
    int y;
};

struct pos
{
    point linear;
    float angular;
};

struct least{
    float a;
    float b;
    float c;
};

struct seed_seg{
    u_int16_t start;
    u_int16_t end;
    least temp;
};

u_int16_t beam_quantity;
point lidar_data[100];
sensor_msgs::LaserScan sensor_data;
pos pose;
bool flag1 = false;
void makeline(std::vector<seed_seg> &list_point_seed, nav_msgs::OccupancyGrid &map);
void seed_segment_detection(std::vector<seed_seg> &list_point_seed);
least fitting_tagent_line(int start, int end);
float dist_point_to_line(least temp, point point);
float dist_point_to_point(point point_start, point point_end);
point predicted_point_cal(least temp, double theta);
int detech_full_line(int m1, int n1, least temp, std::vector<seed_seg> &list_point_seed);
void overlap_region(std::vector<seed_seg> &list_point_seed, nav_msgs::OccupancyGrid &map);
void breseham(cell a, cell b, std::vector<cell> &line, nav_msgs::OccupancyGrid &map);
void get_lidar_data(const sensor_msgs::LaserScan::ConstPtr& scan){


    int num = int((scan->angle_max - scan->angle_min)/(scan->angle_increment)); 
    beam_quantity = num + 1;
    // ROS_INFO("%d",beam_quantity);
    for(int i=0; i < beam_quantity; i++){
        float angle = scan->angle_min + i * scan->angle_increment;
        // ROS_INFO("%f",angle);
        double x = double(scan->ranges[i]*cos(angle)*100 + pose.linear.x);
        double y = double(scan->ranges[i]*sin(angle)*100 + pose.linear.y);
        // ROS_INFO("%f",x);
        lidar_data[i].x = x;
        lidar_data[i].y = y;
    }
    sensor_data.angle_min = scan->angle_min;
    sensor_data.angle_increment = scan->angle_increment;
    flag1 = true;
}

int main(int argc, char** argv){
    clock_t start, end;   // Khai báo biến thời gian
    double time_use;      // Thời gian sử dụng
    ros::init(argc, argv, "line_extraction");
    ros::NodeHandle n;
    ros::Publisher map_data = n.advertise<nav_msgs::OccupancyGrid>("map", 50);
    ros::Subscriber lidar_data = n.subscribe("/scan1", 1000, get_lidar_data);
    nav_msgs::OccupancyGrid map;
    map.info.resolution = 4;
    map.info.width = int(1000/map.info.resolution);
    map.info.height = int(1000/map.info.resolution);
    map.data.resize(map.info.width*map.info.height);

    pose.linear.x = 500;
    pose.linear.y = 500;
    pose.angular = 0;   

    std::vector<seed_seg> list_of_seed;

    ros::Rate r(2);
    while(n.ok()){
        ros::spinOnce();
        while((flag1 == true) && (break_point < (Np - P_min))){
            // ROS_INFO("--------");
            start = clock(); 
            seed_segment_detection(list_of_seed);

            if(break_point > (Np - P_min)){
                overlap_region(list_of_seed, map);
                makeline(list_of_seed, map);
                break_point = 0;
                end = clock();
                time_use = (double)(end - start) / CLOCKS_PER_SEC;
                ROS_INFO("%f", time_use);
                break;
            }
        }
        map_data.publish(map);
        r.sleep();
    }
}

least fitting_tagent_line(int start, int end){
    least temp;
    double w1 = 0, w2 = 0, w3 = 0;
    int n = end - start + 1;
    double mid1 = 0;
    double mid2 = 0;
    double mid3 = 0;
    double mid4 = 0;
    double mid5 = 0;


    for(int i= start ; i <= end; i++){
        mid1 += lidar_data[i].x;
        mid2 += lidar_data[i].y;
        mid3 += pow(lidar_data[i].x,2);
        mid4 += pow(lidar_data[i].y,2);
        mid5 += (lidar_data[i].x * lidar_data[i].y);

    }

    w1 = n*mid5-mid1*mid2;
	w2 = mid2*mid2-n*mid4-mid1*mid1+n*mid3;
	w3 = mid1*mid2-n*mid5;
	//ax+by+c = 0 等价于 y = kx + b;kx-y + b = 0 //a = k,c = b,b=-1
	if(w1==0)
	{
		temp.a = -1;
		temp.b = 0;
		temp.c = mid1/n;
	}
	else
	{
		temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
		temp.b = -1;
		temp.c = (mid2-temp.a*mid1)/n;
	}
    // ROS_INFO("%f,%f,%f",temp.a, temp.b, temp.c);
    return temp;
}

float dist_point_to_line(least temp, point point){
    float ts = abs(temp.a * point.x + temp.b* point.y + temp.c);
    float ms = sqrt(pow(temp.a,2)+pow(temp.b,2));
    return ts/ms;
}

float dist_point_to_point(point point_start, point point_end){
    return float(sqrt(pow(point_start.x - point_end.x,2) + pow(point_start.y - point_end.y,2)));
}

point predicted_point_cal(least temp, double theta){
    point predicted_point;
    predicted_point.x = (-temp.c*cos(theta) - temp.b*(-sin(theta)*pose.linear.x + cos(theta)*pose.linear.y))/(temp.a*cos(theta) + temp.b*sin(theta));
    predicted_point.y = (-temp.c*sin(theta) + temp.a*(-sin(theta)*pose.linear.x + cos(theta)*pose.linear.y))/(temp.a*cos(theta) + temp.b*sin(theta));
    return predicted_point;
}


void seed_segment_detection(std::vector<seed_seg> &list_point_seed){
    least temp;
    point predicted_point;
    int j,k;
    double theta = 0;

    for(int i = break_point; i < (Np - P_min); i++){
        bool flag = true;
        j = S_num + i;
        temp = fitting_tagent_line(i, j-1);

        for( k = i; k < j; k++){
            theta = sensor_data.angle_increment * k + sensor_data.angle_min;
            predicted_point = predicted_point_cal(temp, theta);
            // ROS_INFO("%f", predicted_point.x);
            double d1 = dist_point_to_point(lidar_data[k], predicted_point);
            double d2 = dist_point_to_line(temp, lidar_data[k]);
            // ROS_INFO("%f, %f", d1, d2);
            if((d1 > delta) || (d2 > epsilon)){
                flag = false;
                ROS_INFO("out of all condition");
                break;
            }
        }
        if(flag == true)
        {   detech_full_line(i,j-1,temp,list_point_seed);
            break;    
        }
    }
}

int detech_full_line(int m1, int n1, least temp, std::vector<seed_seg> &list_point_seed){
    // std::vector<seed_seg> list_point_seed;
    seed_seg data;
    int Pl = 0;
    double Ll = 0;
    int Pf = n1 + 1;    // end point
    int Pb = m1 - 1;    // start point
    while(dist_point_to_line(temp,lidar_data[Pf]) < epsilon){
        if(Pf > Np){break;}
        else{ temp = fitting_tagent_line(Pb+1, Pf);}
        Pf +=1;
    }
    while(dist_point_to_line(temp, lidar_data[Pb]) < epsilon){
        // ROS_INFO("%d",Pb);
        if(Pb < 0){ break;}
        else{ temp = fitting_tagent_line(Pb, Pf-1);}
        Pb-=1;
    }
    Pf -=1;
    Pb += 1;
    Pl = Pf - Pb + 1;
    Ll = sqrt(pow(lidar_data[Pf].x - lidar_data[Pb].x,2) + pow(lidar_data[Pf].y - lidar_data[Pb].y,2));
    // ROS_INFO("%d",Pb);
    // ROS_INFO("%d",Pf);
    break_point = Pf;
    if((Pl >= P_min) && (Ll >= L_min)){
        // ROS_INFO("hello");
        data.start = Pb;
        data.end = Pf;
        data.temp = temp;
        list_point_seed.push_back(data);
        return 1;
    }else{
        ROS_INFO("Khong thoa man dieu kien Pmin or Lmin");
        return 0;
    }
}

void overlap_region(std::vector<seed_seg> &list_point_seed, nav_msgs::OccupancyGrid &map){
    // for(int i=0; i < list_point_seed.size(); i++){
    //     ROS_INFO("%d, %d", list_point_seed[i].start, list_point_seed[i].end);
    // }
    int m1=0, n1=0, m2=0, n2=0, k=0;
    for(int i=0; i < list_point_seed.size()-1; i++){
        int j=i+1;
        m1 = list_point_seed[i].start;
        n1 = list_point_seed[i].end;
        m2 = list_point_seed[j].start;
        n2 = list_point_seed[j].end;
        if(m2 <= n1){
            for(k = m2; k <= n1; k++){
                float d1 = dist_point_to_line(list_point_seed[i].temp,lidar_data[k]);
                float d2 = dist_point_to_line(list_point_seed[j].temp,lidar_data[k]);
                if(d1 < d2){
                    continue;
                }
                else{
                    break;
                }
            }
        list_point_seed[i].end = k - 1;
        list_point_seed[j].start = k ;
        }
        else{
            break;
        }
        list_point_seed[i].temp = fitting_tagent_line(m1,n1);
        list_point_seed[j].temp = fitting_tagent_line(m2,n2);
    }
}

void makeline(std::vector<seed_seg> &list_point_seed, nav_msgs::OccupancyGrid &map){
    cell a;
    cell b;
    std::vector<cell> line;         // testing
    for(int i=0; i<list_point_seed.size(); i++){
        a.x = lidar_data[list_point_seed[i].start].x/map.info.resolution;
        a.y = lidar_data[list_point_seed[i].start].y/map.info.resolution;
        // b.x = lidar_data[list_point_seed[i].end].x/map.info.resolution;
        // b.y = lidar_data[list_point_seed[i].end].y/map.info.resolution;
        // dang do tinh x_s va y_s doan nay
        b.x = int((pow(list_point_seed[i].temp.b,2)*lidar_data[list_point_seed[i].end].x - list_point_seed[i].temp.a*list_point_seed[i].temp.b*lidar_data[list_point_seed[i].end].y - list_point_seed[i].temp.a*list_point_seed[i].temp.c)/(pow(list_point_seed[i].temp.a,2)+pow(list_point_seed[i].temp.b,2))/map.info.resolution);
        b.y = int((pow(list_point_seed[i].temp.a,2)*lidar_data[list_point_seed[i].end].y - list_point_seed[i].temp.a*list_point_seed[i].temp.b*lidar_data[list_point_seed[i].end].x - list_point_seed[i].temp.b*list_point_seed[i].temp.c)/(pow(list_point_seed[i].temp.a,2)+pow(list_point_seed[i].temp.b,2))/map.info.resolution);
        // ROS_INFO("%d, %d",b.x,b.y);
        // ROS_INFO("%d, %d",a.x,a.y);
        breseham(a,b,line,map);
    }
}

void breseham(cell a, cell b, std::vector<cell> &line, nav_msgs::OccupancyGrid &map){
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