#include <head_filter.h>

//***********************************************************************

//		   Définition de la classe de traitement du point cloud

//***********************************************************************
namespace point_cloud_filtering_and_dimensioning {



//Callback pour initialisation des parametre de la classe traitement
/* void PointCLoud_Modif_Class::callback(pcl_dynamic_reconfigConfig &config,
		uint32_t level) {
	ROS_INFO("xmin: %f ", config.xmin);





	xmin = config.xmin;
	xmax = config.xmax; 
	ymin = config.ymin;
	ymax = config.ymax;
	zmin = config.zmin;
	zmax = config.zmax;

	roll = config.roll;

	height_offset=-config.kinect_height;   


	nb_nei_stat_filt = config.nb_nei_stat_filt;
	StddevMulThresh = config.StddevMulThresh;
	grid = config.grid;
	//disp_curv_window.init

}






    /**********************************************************************************
 **********************************************************************************
 **********************************************************************************
 **																  		    	 **
 **				      Définition de la fonction de filtrage	               		 **
 **																                 **
 **********************************************************************************
 **********************************************************************************
 **********************************************************************************/

    void PointCLoud_Modif_Class::filtering(
            const sensor_msgs::PointCloud2ConstPtr &input) {


        xmin = x - 0.17; //0.17
        xmax = x + 0.17; //0.17
        ymin = y - 0.17; //0.2
        ymax = y + 0.10; //0.1
        zmin = z - 0.17; //0.17
        zmax = z + 0.17; //0.17


//        roll = 0;

        height_offset = 0;



/**********************************************************************************
 * **				                Definition d'une autre tf			         **
 **********************************************************************************/

        //publier une tf
        static tf::TransformBroadcaster br_rot;
        tf::Transform transform_rota;

        //Manipulation par raport a la tf de la camera
        tf::Quaternion q;
        q.setRPY((-roll) * M_PI / 180, 0, 0);
        transform_rota.setRotation(q);
        transform_rota.setOrigin(tf::Vector3(0, 0, 0));

        br_rot.sendTransform(
                tf::StampedTransform(transform_rota, ros::Time::now(), "kinect2_ir_optical_frame", "angle_correction"));


        q.setRPY((roll) * M_PI / 180, 0, 0);
        transform_rota.setRotation(q);


        static tf::TransformBroadcaster br;


        tf::Transform transform_trans;


        transform_trans.setOrigin(tf::Vector3(0, -height_offset, 0));

        tf::Quaternion q2;
        q2.setRPY(0, 0, 0);
        transform_trans.setRotation(q2);

//        std::string tf_name = "WORLD";

        std::string tf_name = "kinect2_ir_optical_frame";


        br.sendTransform(tf::StampedTransform(transform_trans, ros::Time::now(), "angle_correction", tf_name));

        transform_trans.setOrigin(tf::Vector3(0, height_offset, 0));

        sensor_msgs::PointCloud2 p;

        const sensor_msgs::PointCloud2Ptr &out = boost::make_shared<sensor_msgs::PointCloud2>(p);

        const sensor_msgs::PointCloud2Ptr &out2 = boost::make_shared<sensor_msgs::PointCloud2>(p);


        pcl_ros::transformPointCloud(tf_name, transform_rota, *input, *out);


//        out->header.frame_id = "angle_correction";

        out->header.frame_id = "kinect2_ir_optical_frame";


        pcl_ros::transformPointCloud(tf_name, transform_trans, *out, *out2);


        pcl::PCLPointCloud2::Ptr pcl2(new pcl::PCLPointCloud2());


        pcl_conversions::toPCL(*out2, *pcl2);


        //out2->header.frame_id=tf_name;



/**********************************************************************************
 **				                 Filtrage densite		    	                 **
 **********************************************************************************/
//        grid = 0.01;             //Distance min entre les points
//        //Voxelgrid Filter
//        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//        sor.setInputCloud(pcl2);
//        sor.setLeafSize(grid, grid, grid);
//        sor.filter(*pcl2);



/**********************************************************************************
 **				                 Filtrage BOX		    	                 **
 **********************************************************************************/

        pcl::PointCloud<pcl::PointXYZ>::Ptr pclXYZ(
                new pcl::PointCloud<pcl::PointXYZ>);

        // Fill in the cloud data
        pcl::fromPCLPointCloud2(*pcl2,
                                *pclXYZ); //(convertion entre sensor_msgs::PointCloud2 et  pcl::PointCloud<pcl::PointXYZ>.

        // Create the filtered object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
                new pcl::PointCloud<pcl::PointXYZ>);

        //PassTrought filter
        pcl::PassThrough<pcl::PointXYZ> pass;

        cloud_filtered->header.frame_id = tf_name;



        pass.setInputCloud(pclXYZ);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(zmin, zmax);

        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(xmin, xmax);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(ymin, ymax);
        //pass.setFilterLimitsNegative (true);
        pass.filter(*cloud_filtered);


//
//        sensor_msgs::PointCloud::Ptr pcl2_filtered_pub(
//                new sensor_msgs::PointCloud2);
//
//        pcl::PointCloud<pcl::PointXYZ>
//
//
//
//
//
//
//        //On retourne le nuage filtrer dans un message ROS afin de pouvoir le visualiser
//        pcl::toROSMsg(*cloud_filtered, *pcl2_filtered_pub);
//
//        pcl2_filtered_pub->header.frame_id = tf_name;
//
//
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//
//        keypoints_ptr=*cloud_filtered;
//
//
//        pub_cloud.publish(keypoints_ptr);
//


//        int it=0;
//
//        int siz=cloud_filtered->size();
//
//
//        float cloud_coord[100][3];
//
//
////        for(int i=0;i<cloud_filtered->size()/3; i++){
//
//        for(int i=0;i<cloud_filtered->size(); i++){
//
//
//            if(x-1<cloud_filtered->points[i].x<x+1 && y-1<cloud_filtered->points[i].y<y+1 && z-1<cloud_filtered->points[i].z<z+1){
//
//                cloud_coord[it][1]=cloud_filtered->points[i].x;
//
//                cloud_coord[it][2]=cloud_filtered->points[i].y;
//
//                cloud_coord[it][3]=cloud_filtered->points[i].z;
//
//                it++;
//            }
//
//        }
//


        //StatisticalOutlierRemoval filter
        // Create the filtering object

/**********************************************************************************
**				                Filtrage Statistique			                 **
**********************************************************************************/


        nb_nei_stat_filt = 2;   //Nb de pts voisin
        StddevMulThresh = 0.1;   //ecart type maximum



        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud_filtered);
        sor2.setMeanK(nb_nei_stat_filt);
        sor2.setStddevMulThresh(StddevMulThresh);
        sor2.filter(*cloud_filtered);


        sensor_msgs::PointCloud2::Ptr pcl2_filtered_pub(
                new sensor_msgs::PointCloud2);




        //On retourne le nuage filtrer dans un message ROS afin de pouvoir le visualiser
        pcl::toROSMsg(*cloud_filtered, *pcl2_filtered_pub);

        pcl2_filtered_pub->header.frame_id = tf_name;

        pub_cloud.publish(pcl2_filtered_pub);

    }





//***********************************************************************

//Penser à une façon de sauvegarder le pointcloud complet en cas de faute

//***********************************************************************

//Class constructor
    void PointCLoud_Modif_Class::init(ros::NodeHandle &nh) {

        body_part=0;



        Part_id_1.header.frame_id = "/kinect2_ir_optical_frame"; //à mettre en init

        Part_id_1.header.stamp = ros::Time::now();
        Part_id_1.ns = "Part_id_1";
        Part_id_1.action = visualization_msgs::Marker::ADD;
        Part_id_1.pose.orientation.w = 1.0;
        Part_id_1.id = 1;

        // POINTS markers use x and y scale for width/height respectively
        Part_id_1.scale.x = 0.1;
        Part_id_1.scale.y = 0.1;

        // Points are red
        Part_id_1.color.r = 1.0f;

        // Transparency
        Part_id_1.color.a = 0.5;

        Part_id_1.type = visualization_msgs::Marker::POINTS;

        // setup dynamic reconfigure

        dimension_flag = 0;

        // Subscriber au sensor_msgs de la kinect
        sub_kinect_cloud = nh.subscribe("/kinect2/sd/points", 1,
                                        &PointCLoud_Modif_Class::cloud_cb, this);

        sub_heads = nh.subscribe("/head_getter/persons_denorm", 1,
                                 &PointCLoud_Modif_Class::heads_cb, this);

        //Publisher du nuage filtré
        pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("filtered_pcloud", 1);

//
//        typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
//
//        //Publisher du nuage filtré
//        pub_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("filtered_pcloud", 1);

        pub_max_height = nh.advertise<std_msgs::Float32>("max_height", 1);


        pub_Part_id_1 = nh.advertise<visualization_msgs::Marker>("Part_id_1",
                                                                 1);

    }







//***********************************************************************

//Penser à une façon de sauvegarder le pointcloud complet en cas de faute

//***********************************************************************

    void PointCLoud_Modif_Class::cloud_cb(
            const sensor_msgs::PointCloud2ConstPtr &kinect_sensor_pcl) {

//        sensor_msgs::PointCloud2ConstPtr pcl2(new pcl::PCLPointCloud2());
//        pcloud_handle=kinect_sensor_pcl;
        if (dimension_flag == 1) {

            filter_center_point(kinect_sensor_pcl);

            dimension_flag = 0;

        }




        PointCLoud_Modif_Class::filtering(kinect_sensor_pcl);
        pub_max_height.publish(-ymin);

    }


    void PointCLoud_Modif_Class::heads_cb(const tfpose_ros::PersonsConstPtr &persons) {

        if (persons->persons.data()->body_part.data()->part_id == body_part) {

            u = (int) persons->persons.data()->body_part.data()->x;
            v = (int) persons->persons.data()->body_part.data()->y;

        }

        dimension_flag = 1;

    }


    void PointCLoud_Modif_Class::filter_center_point(const sensor_msgs::PointCloud2ConstPtr &pCloud) {
/**
    Function to convert 2D pixel point to 3D point by extracting point
    from PointCloud2 corresponding to input pixel coordinate. This function
    can be used to get the X,Y,Z coordinates of a feature using an
    RGBD camera, e.g., Kinect.
    */

        // get width and height of 2D point cloud data
        int width = pCloud->width;
        int height = pCloud->height;


        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = v * pCloud->row_step + u * pCloud->point_step;

        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + pCloud->fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
        memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
        memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));



//
//        for(int i=15; i>0;i--){
//
//            body_part=i;

  if(X>-20 && X<20|Y>-20 && Y<20|Z>-20 && Z<20){


        x = X;
        y = Y;
        z = Z;

        p_.x = X;

        p_.y = Y;

        p_.z = Z;


        Part_id_1.points.push_back(p_);


        pub_Part_id_1.publish(Part_id_1);

        Part_id_1.points.clear();
  }


//
//            printf("%d",i);
//
//            std::cout << i;
//        }




//        body_part=body_part+1;
//
//        if(body_part>20)body_part=1;
    }

}


//***********************************************************************

//Penser à une façon de sauvegarder le pointcloud complet en cas de faute

//***********************************************************************

int main(int argc, char **argv) {



    ros::init(argc, argv, "cloud_filter");

    ros::NodeHandle nh;

    point_cloud_filtering_and_dimensioning::PointCLoud_Modif_Class nom;

    nom.init(nh);

    // Spin
    ros::spin();
}

