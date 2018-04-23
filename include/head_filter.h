#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cmath>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/builtin_float.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include "tfpose_ros/Persons.h"
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>

#include <math.h>

// #include <dynamic_reconfigure/server.h>

#include <boost/bind.hpp>

#include <boost/shared_ptr.hpp>





// #include <point_cloud_filtering_and_dimensioning/pcl_dynamic_reconfigConfig.h>

/**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **				       Définition de la fonction de filtrage	          	 	 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/
namespace point_cloud_filtering_and_dimensioning {


//Définition de la classe
    class PointCLoud_Modif_Class {

        geometry_msgs::Point p;

        tfpose_ros::Persons person;


        int dimension_flag;


        int u;
        int v;

        //Passetout
        float xmin;
        float xmax;
        float ymin;
        float ymax;
        float zmin;
        float zmax;

        int body_part;


        float x;
        float y;
        float z;

        float height_offset;

        float roll;

        //Statistical
        int nb_nei_stat_filt;
        float StddevMulThresh;

        //Voxelgrid
        float grid;





//    tfpose_ros::Persons pers;

        visualization_msgs::Marker Part_id_1;   //Points à visualiser avec Rviz

        geometry_msgs::Point p_;


        ros::Publisher tf_pub;


//	sensor_msgs::PointCloud2::Ptr filtered_pcloud;
        sensor_msgs::PointCloud2ConstPtr pcloud_handle;



        pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ;
        pcl::PCLPointCloud2::Ptr pcl2;

        // Create a ROS publisher for the output point cloud
        ros::Publisher pub_cloud;
//        ros::Publisher pub_pclXYZ;
        ros::Publisher pub_max_height;

        ros::Publisher pub_Part_id_1;
        ros::Subscriber sub_kinect_cloud;
        ros::Subscriber sub_heads;


        /********************************************************************************

         Définition de la classe de traitement du point cloud



         Fonctions:


         -fonction1
         Explications :

         -fonction2
         Explications :

         *********************************************************************************/

    public:

        void set_box(const geometry_msgs::Point &p);

        void init(ros::NodeHandle& nh);

        void heads_cb(const tfpose_ros::PersonsConstPtr &persons);

        void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p);

        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

        void filtering(const sensor_msgs::PointCloud2ConstPtr& input);

        void filter_center_point(const sensor_msgs::PointCloud2ConstPtr &pCloud);

        // void callback(pcl_dynamic_reconfigConfig &config, uint32_t level);



    };

}
