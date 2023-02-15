#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_box.h>

#include <chrono>
#include <iomanip>
#include<iostream>
#include <algorithm>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cmath>
#include <vector>
#include <numeric>
#include <iostream>
#include <fstream>

#include<conversions.h>
#include<my_common.h>
#include"gps_ins_read/gps_data.h"

using namespace std;
pcl::PointCloud<PointType>::Ptr MappedPoint(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr slopedata(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obstcle(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_rawcloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);


float L=0.897;
float T=0.515;
float TurnThr=1e-3;
double line_len = 3;
int point_num=5;
int z=0;
int q = 0;
int s=0;
int sl = 0;
vector<double >k_;
double res,init_z,height;
double k_mean=0.0;

struct point
{
    double x;
    double y;
    double z;

};

struct GridIndex 
{
    int  x;
    int y;
};

class Prediction{
    private:
        ros::NodeHandle nh;
        ros::Subscriber Pointcloud_Sub;
        ros::Subscriber Ins_sub;
        ros::Publisher pub_car_path,
                                      pub_left_front_path,
                                      pub_right_front_path,
                                      pub_pred_right_front_path,
                                      pub_pred_left_front_path,
                                      pub_lf_marker,
                                      pub_rf_marker,
                                      pub_lf_markers,
                                      pub_rf_markers,
                                      pub_PointCloud,
                                      pub_ground,
                                      pub_OccMap;

    public:

        nav_msgs::Path left_front_path,right_front_path,pred_left_front_path,pred_right_front_path;
        sensor_msgs::PointCloud2 pointcloud2,groundcloud;

        vector<gps_ins_read::gps_data>pose_vector;
        vector<gps_ins_read::gps_data>pose_vector_last;
        vector<double>pred_left_path;
        vector<double>pred_right_path;
        vector<vector<double> >ret_l,ret_r;
        pair<vector<vector<double> >,vector<vector<double> >>  ret;
        double init_time;
        double delta[2]={0,0};

        point car,left_front,left_rear,right_rear,right_front,init_pose;
        gps_ins_read::gps_data pose;

        void Prediction_subscribe()
        {
            pub_left_front_path=nh.advertise<nav_msgs::Path>("/left_front_path",100);
            pub_right_front_path=nh.advertise<nav_msgs::Path>("/right_front_path",100);
            pub_pred_right_front_path=nh.advertise<nav_msgs::Path>("/pred_right_front_path",100);
            pub_pred_left_front_path=nh.advertise<nav_msgs::Path>("/pred_left_front_path",100);
            pub_PointCloud=nh.advertise<sensor_msgs::PointCloud2>("/Map",100);
            pub_ground=nh.advertise<sensor_msgs::PointCloud2>("/ground",100);
            pub_OccMap=nh.advertise<nav_msgs::OccupancyGrid>("/GridMap",100);
           
            pub_lf_marker=nh.advertise<visualization_msgs::Marker>("/lf_Marker",100);
            pub_rf_marker=nh.advertise<visualization_msgs::Marker>("/rf_Marker",100);
            pub_lf_markers=nh.advertise<visualization_msgs::MarkerArray>("/lf_Markers",100);
            pub_rf_markers=nh.advertise<visualization_msgs::MarkerArray>("/rf_Markers",100);

            Pointcloud_Sub =nh.subscribe("time_point",1,&Prediction::PointCloudCallback,this);
            Ins_sub=nh.subscribe("BD992_INS",5,&Prediction::InsCallback,this);

         }

        void InsCallback(const gps_ins_read::gps_dataConstPtr& imu_msg)
         {
            pose.Heading=M_PI/2-deg2rad(imu_msg->Heading);
            pose.Easting=imu_msg->Easting;
            pose.Northing=imu_msg->Northing;
            pose.altitude=imu_msg->altitude;
            pose.Total_VEL=imu_msg->Total_VEL;
            pose_vector.emplace_back(pose);

            if(pose_vector.size()>1)
            {
                pose_vector_last.emplace_back(pose);
            }
            else
            {
                init_pose.x=pose.Easting;
                init_pose.y=pose.Northing;
                init_pose.z=pose.altitude;
                init_time = ros::Time::now().toSec();
            }
            wheelpose();

            dispwheel();

            ret=pred_model();

            pubpredpath(ret.first,ret.second);
 
            pubwheel_path();

            double time = ros::Time::now().toSec();
            double timed =time-init_time;
            if (timed>2)
            {
                if(q ==0)
                {
                    if(!isnan(searchPoint()))
                    {
                        init_z =searchPoint();
                        res=0.0;
                        q++;
                    }
                    else
                    {
                        init_z=0.0;
                        z=0.0;
                    }
                }
                else
                {
                    res=searchPoint4k()-init_z;
                    height = searchPoint()-init_z;
                }
                double dis= get_distance();

                pcl::PointXYZ tmp;

                tmp.x=dis;
                tmp.y=res; 
                tmp.z=0;     
                slopedata->points.push_back(tmp);

                if(slopedata->points.size()>5)
                {
                    double k= slope();
                    k_.push_back(k);
                }
                if(k_.size()>1)
                {
                    double sum = accumulate(k_.begin(),k_.end(),0.0);
                    k_mean= sum/k_.size();
                    cout <<"k="<<k_mean<<endl;
                    cout <<"res= "<<height-k_mean*dis<<endl;
                    ofstream outFile;
                    outFile.open("/home/z/data.txt",ofstream::app);
                    outFile<<dis<<"   "<<height-k_mean*dis<<endl;
                }
                else
                {
                    cout <<res<<endl;
                    ofstream outFile;
                    outFile.open("/home/z/data.txt",ofstream::app);
                    outFile<<dis<<"   "<<height<<endl;
                }
            }
        }
       
        double get_distance()
        {
            double h = sqrt(pow(ret.first[2][0],2)+pow(ret.first[2][1],2));

            return h;
        }

        double slope()
        {
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(slopedata));	//指定拟合点云与几何模型
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);	//创建随机采样一致性对象

            ransac.setDistanceThreshold(0.0008);	//内点到模型的最大距离
            ransac.setMaxIterations(1200);		//最大迭代次数
            ransac.computeModel();	

            vector<int> inliers;				//存储内点索引的向量
            ransac.getInliers(inliers);			//提取内点对应的索引

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*slopedata, inliers, *cloud_line);
            Eigen::VectorXf coefficient;
            ransac.getModelCoefficients(coefficient);

            /*cout << "直线点向式方程为：\n"
                << "   (x - " << coefficient[0] << ") / " << coefficient[3]
                << " = (y - " << coefficient[1] << ") / " << coefficient[4]
                <<"  = (z- " <<coefficient[2]<<" )/ "<<coefficient[5]<<endl;
            */
            return coefficient[4]/coefficient[3];

        }

        void PointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& laserCloudMsg)
        {            
            
            pcl::fromROSMsg(*laserCloudMsg,*pcl_rawcloud);
        
            pcl::PointXYZ tmp;
            
            double time = ros::Time::now().toSec();
            double timed =time-init_time;
            if (timed >10 &&s<1)
            {
               
               // recov();
                s++;
            }
            for(int i=0;i<pcl_rawcloud->points.size();i++)
            {
                if (sqrt(pow((pcl_rawcloud->points[i].x-car.x),2)+pow((pcl_rawcloud->points[i].y-car.y),2))<4.5&&pcl_rawcloud->points[i].z<0)
                {   
                    z++;
                    if(z<200000)
                    {
                        tmp.x=pcl_rawcloud->points[i].x;
                        tmp.y=pcl_rawcloud->points[i].y;
                        tmp.z=pcl_rawcloud->points[i].z;

                        pcl_cloud->push_back(tmp);
                    }
                    else
                    {
                        if(pose.Total_VEL>0.03)
                          {
                            int zz=z%200000;
                            pcl_cloud->points[zz].x=pcl_rawcloud->points[i].x;
                            pcl_cloud->points[zz].y=pcl_rawcloud->points[i].y;
                            pcl_cloud->points[zz].z=pcl_rawcloud->points[i].z;
                          }
                        else
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    continue;
                }
            }

            pcl::toROSMsg(*pcl_cloud, pointcloud2);
            pointcloud2.header.stamp=ros::Time::now();
            pointcloud2.header.frame_id="velodyne";
            pub_PointCloud.publish(pointcloud2);


                   
        }
        double searchPoint()
        {
            pcl::PointXY Point;
            Point.x = ret.first[2][0];
            Point.y = ret.first[2][1];

            const float R_squared = 0.1 * 0.1;
            vector<double> res;
            res.reserve(pcl_cloud->points.size());

            for (auto it = pcl_cloud->begin(); it != pcl_cloud->end(); ++it)
            {
                const float dx = it->x - Point.x;
                const float dy = it->y - Point.y;

                if (dx * dx + dy * dy <= R_squared)
                {
                    res.push_back(it->z);
                }
            }

            double qz = 0.0;
            if (!res.empty())
            {
                qz = accumulate(res.begin(), res.end(), 0.0) / res.size();
            }
            return qz;
        }

        double  searchPoint4k()//该函数求得结果为求地面斜率//
        {
            pcl::PointXY Point;

            Point.x=ret.first[2][0];
            Point.y=ret.first[2][1];
            vector<double> res ;

            float R = 0.1;

            for(int i =0;i<ground->points.size();i++)
            {
                if(sqrt(pow((ground->points[i].x-Point.x),2)+pow((ground->points[i].y-Point.y),2))<R)
                   {
                        res.push_back(ground->points[i].z);
                   }
                   else
                   {
                        continue;
                   }
            }
            double sum = accumulate(res.begin(),res.end(),0.0);
            double qz= sum/res.size();

            return  qz ;
        }
        
        vector<double> linspace(double  start_in, double  end_in, int num_in)
        {
            vector<double> linspaced;

            double start = static_cast<double>(start_in);
            double end = static_cast<double>(end_in);
            double num = static_cast<double>(num_in);

            if (num == 0) { return linspaced; }
            if (num == 1) 
                {
                linspaced.emplace_back(start);
                return linspaced;
                }

            double delta = (end - start) / (num - 1);

            for(int i=0; i < num-1; ++i)
                {
                linspaced.emplace_back(start + delta * i);
                }
            linspaced.emplace_back(end); 
                                    
            return linspaced;
        }

        pair<vector<vector<double> >,vector<vector<double> >> pred_model ()
        {
            pair<vector<vector<double> >,vector<vector<double> >> ret;
            if(abs(delta[0])<TurnThr and abs(delta[1])<TurnThr)
            {
                ret=fun_straight();
            }
            else
            {
                ret=fun_turn();
            }
            return ret;
        }

        vector <vector<double> > sample_in_line(double thet,double x0,double y0,double  line_len,int point_num)
        {
            thet = fmod(thet+2*M_PI,2*M_PI);
            vector<double> rhos =linspace(0,y0+line_len,point_num);
           vector <vector<double> >xsys;

            for(int i=0;i<point_num;i++)
            {
                vector<double> array;
                double a,b;

                a=rhos[i]*cos(thet)+x0;
                array.emplace_back(a);
                b=rhos[i]*sin(thet)+y0;
                array.emplace_back(b);
                xsys.emplace_back(array);
                array.clear();
            }   
            return xsys;
        }
    
        pair<vector<vector<double> >,vector<vector<double> >> fun_straight()
        {
            pair<vector<vector<double> >,vector<vector<double> >> ret;
            vector<vector<double> > ret_l,ret_tmp,ret_r;
            vector<double> tmp_l,tmp_r;
            ret_tmp = sample_in_line(pose.Heading,0,0,line_len,point_num);
            
            
            for(int i=0;i<ret_tmp.size();i++)
            { 
                double a=ret_tmp[i][0]+right_front.x;
                double b=ret_tmp[i][1]+right_front.y;
                tmp_r.emplace_back(a);
                tmp_r.emplace_back(b);
                ret_r.emplace_back(tmp_r);
                tmp_r.clear();

                double c=ret_tmp[i][0]+left_front.x;
                double d=ret_tmp[i][1]+left_front.y;
                tmp_l.emplace_back(c);
                tmp_l.emplace_back(d);
                ret_l.emplace_back(tmp_l);
                tmp_l.clear();
            }
            ret=make_pair(ret_l,ret_r);
            return ret;

        }
  
        vector<vector<double> >sample_in_circle(double r,double x0,double y0,double thet0,double line_len,int point_num,int polarity)
        {
            double dthet=line_len/r;
            vector<double>thets;
            thets=linspace(thet0,thet0+dthet*polarity,point_num);
            vector<vector<double> >xsys;
            for(int i=0;i<point_num;i++)
            {
                double a,b;
                vector<double> array;
                a=r*cos(thets[i])+x0;
                array.emplace_back(a);
                b=r*sin(thets[i])+y0;
                array.emplace_back(b);
                xsys.emplace_back(array);
                array.clear();
            }   
            return xsys;
        }

        pair<vector<vector<double> >,vector<vector<double> >> fun_turn()
        {
            double inside_delta,outside_delta,center_x,center_y,inside_r,outside_r,inside_thet0,outside_thet0;
            int polarity;
            point inside_front,inside_rear,outside_front,outside_rear;
            vector<vector<double> >ret_inside,ret_outside;
            pair<vector<vector<double> >,vector<vector<double> >> ret;
            
            if(delta[0]>0)//左转大于0
            {
                inside_front=left_front;
                inside_rear=left_rear;
                outside_front=right_front;
                outside_rear=right_rear;
                inside_delta=delta[0];
                outside_delta=delta[1];
                polarity=1;
            }
            else//右转
            {
                inside_front=right_front;
                inside_rear=right_rear;
                outside_front=left_front;
                outside_rear=left_rear;
                inside_delta=delta[0];
                outside_delta=delta[1];
                polarity=-1;
            }
            center_x=tan(inside_delta + pose.Heading) * tan(pose.Heading) / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * (inside_rear.y - inside_front.y) \
                                + tan(inside_delta + pose.Heading) / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * inside_rear.x \
                                - tan(pose.Heading) / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * inside_front.x;
                                
            center_y = (-tan(pose.Heading)) / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * inside_rear.y \
                                + tan(inside_delta + pose.Heading) / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * inside_front.y \
                                - 1 / (tan(inside_delta + pose.Heading) - tan(pose.Heading)) * (inside_rear.x - inside_front.x);

            inside_r = abs(L /sin(inside_delta));
            outside_r = abs(L /sin(outside_delta));

            inside_thet0=get_thet(center_x,center_y,inside_r,inside_front.x,inside_front.y);
            outside_thet0=get_thet(center_x,center_y,outside_r,outside_front.x,outside_front.y);     
            
            ret_inside=sample_in_circle(inside_r,center_x,center_y,inside_thet0,line_len,point_num,polarity);
            ret_outside=sample_in_circle(outside_r,center_x,center_y,outside_thet0,line_len,point_num,polarity);
            if (delta[0]<0)
            {
              ret=make_pair(ret_outside,ret_inside);
            }
            else
            {
                ret=make_pair(ret_inside,ret_outside);
            }
            return ret;
        }
        
        double get_thet(double center_x,double center_y,double r,double point_x,double point_y)
        {
            double thet;
            thet =acos((point_x-center_x)/r);
            if(point_y>center_y)
            {
                thet = abs(thet);
            }
            else
            {
                thet =-abs(thet);
            }
            return thet;
        }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pred_model");

  Prediction p ;
  p.Prediction_subscribe();
  ros::MultiThreadedSpinner spinner(4); 
  spinner.spin(); 

  return 0;

}
