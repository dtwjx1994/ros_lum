#include "ros/ros.h"
#include "std_msgs/String.h"
#include<Eigen/Eigen>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include<pcl/registration/lum.h>
#include<pcl/registration/correspondence_estimation.h>
#include<pcl/common/transforms.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include<pcl/console/parse.h>
#include<pcl/console/print.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<pcl/filters/voxel_grid.h>
using namespace std;
 Eigen::Vector4f centroid;
typedef pcl::PointXYZI pointT;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRegistered2(new pcl::PointCloud<pcl::PointXYZI>());
typedef pcl::PointCloud<pointT> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;


typedef std::pair<double, CloudPtr> CloudPair;
pcl::registration::LUM<pointT> lum;
ros::Publisher *publum = NULL;
ros::Publisher *pubsurrd=NULL;
vector< CloudPair > clouds;
CloudPtr surround(new Cloud);
ros::Publisher *pubpath=NULL;
 pcl::VoxelGrid<pointT> downsample;
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > v_registered;
vector<Eigen::Vector4f> center;
CloudPtr lasersurr(new Cloud);
int  C_POINT=0;
unsigned int loopCount = 20;
int lumIter = 1;
int iter = 10;
double dist = 2.5;
Eigen::Vector6f currpose;
nav_msgs::Path trajectory;

double loopDist=10.0;
double lasertime;
bool findloopclosure()
{
     int z=clouds.size()-1;
    // cout<<"i"<<z<<endl;
        for (size_t j = 0; j < z; j++)
        {
         // cout<<"555555"<<endl;
          Eigen::Vector4f diff = center[z] - center[j] ;
          if(diff.norm () < loopDist && (/*z - j == 1 ||*/ z - j > loopCount))
          {
            //if(z - j > loopCount)
            std::cout << "add connection between " << z << " (" << clouds[z].first << ") and " << j << " (" << clouds[j].first << ")" << std::endl;
            pcl::registration::CorrespondenceEstimation<pointT, pointT> ce;
            ce.setInputTarget (clouds[z].second);
            ce.setInputSource (clouds[j].second);
            pcl::CorrespondencesPtr corr (new pcl::Correspondences);
            ce.determineCorrespondences (*corr, dist);
            //if (corr->size () > 2)
              lum.setCorrespondences (j, z, corr);
            return true;
          }
        }
        return false;
///test//////

//
//   for(size_t i = 0; i < lum.getNumVertices (); i++)
//   {
//     Eigen::Matrix4f m;
//     m=lum.getTransformation(i).matrix();

//     Eigen::Matrix3f f=m.block(0,0,3,3);
//     Eigen::Quaternionf q=Eigen::Quaternionf(f);
//     //Eigen::Quaterniond q=Eigen::Quaterniond(rotationMatrix)
//    // Eigen::Quaternionf q=Eigen::Quaterniond(lum.getTransformation(i));
//     geometry_msgs::PoseStamped this_pose_stamped;
//     this_pose_stamped.pose.orientation.x=q.x();
//     this_pose_stamped.pose.orientation.y=q.y();
//     this_pose_stamped.pose.orientation.z=q.z();
//     this_pose_stamped.pose.orientation.w=q.w();
//     this_pose_stamped.header.stamp=ros::Time::now();
//     cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;

//     trajectory.poses.push_back(this_pose_stamped);

//    //  cout<<surround->size()<<"    ";
//     //std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;

//     clouds[i].second = lum.getTransformedCloud (i);

//     //surround=lum.getConcatenatedCloud();
//   }

//    pubpath->publish(trajectory);


   //lum.getPose()
  //pcl::io::savePCDFile("/home/wu/Desktop/pcd/"+to_string(countsurr++)+".pcd",*surround);
//   pcl::toROSMsg(*surround,laserCloudSurround4);
//   laserCloudSurround4.header.stamp = ros::Time().fromSec(timeCloudRegistered);
//   laserCloudSurround4.header.frame_id = "/camera_init";
//   pubsurrd->publish(laserCloudSurround4);
 // }

}
//void odohandler(const nav_msgs::Odometry::ConstPtr &odom)
//{
//  double roll, pitch, yaw;
//  geometry_msgs::Quaternion geoQuat=odom->pose.pose.orientation;
//  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
//   currpose[0]=-pitch;
//   currpose[1]=-yaw;
//   currpose[2]=roll;
//   currpose[3]=odom->pose.pose.position.x;
//   currpose[4]=odom->pose.pose.position.y;
//   currpose[5]=odom->pose.pose.position.z;
//   cout<<"odom"<<endl;


//}

//void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& CloudRegistered)
//{
//  trajectory.header.frame_id="/camera_init";
//    C_POINT++;
//    double timeCloudRegistered = CloudRegistered->header.stamp.toSec();

//    laserCloudRegistered2->clear();
//    pcl::fromROSMsg(*CloudRegistered, *laserCloudRegistered2);
//    CloudPtr pc(new Cloud);
//    pc->clear();
//    for(int i=0;i<laserCloudRegistered2->points.size();i++)
//    {
//        float x,y,z;
//        pointT pt;
//        x=laserCloudRegistered2->points[i].x;
//        y=laserCloudRegistered2->points[i].y;
//        z=laserCloudRegistered2->points[i].z;
//        pt.x=x;
//        pt.y=y;
//        pt.z=z;
//        pt.intensity=laserCloudRegistered2->points[i].intensity;
////        laserCloudRegistered2->points[i].x=z;
////        laserCloudRegistered2->points[i].y=x;
////        laserCloudRegistered2->points[i].z=y;
//        pc->push_back(pt);
//    }
//    //Eigen::Vector4f center;
//    Eigen::Vector4f ci;
//    pcl::compute3DCentroid(*pc,ci);
//    cout<<"ci"<<ci<<endl;
//    cout<<"pc "<<pc->size()<<endl;
//    center.push_back(ci);//push the centroid into the vector
//    clouds.push_back( CloudPair(timeCloudRegistered,pc));
//    lum.addPointCloud(pc);


//    sensor_msgs::PointCloud2 laserCloudSurround;
//    sensor_msgs::PointCloud2 laserCloudregister;



//    bool _findloop=false;
//    surround->clear();
//    _findloop=findloopclosure();
//       if(_findloop)
//       {
//         lum.compute();
//        ROS_INFO_STREAM("find loop");
//         for(size_t i = 0; i < lum.getNumVertices (); i++)
//           {

//             //std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
//             clouds[i].second = lum.getTransformedCloud (i);
//            // cout<<"points number "<<clouds[i].second->size()<<endl;
//             //*surround+=*(clouds[i].second);
//           }
//       }
//       for(size_t k=0;k<clouds.size();k++)
//       {
//         *surround+=*(clouds[k].second);
//       }
//  lasersurr->clear();
//  downsample.setInputCloud(surround);
//  downsample.filter(*lasersurr);

//   pcl::toROSMsg(*pc, laserCloudregister);
//   pcl::toROSMsg(*lasersurr,laserCloudSurround);
//   laserCloudSurround.header.stamp = ros::Time().fromSec(timeCloudRegistered);
//   laserCloudSurround.header.frame_id = "/camera_init";
//   laserCloudregister.header.stamp = ros::Time().fromSec(timeCloudRegistered);
//   laserCloudregister.header.frame_id = "/camera_init";

//    ROS_INFO_STREAM("RECECEIVE NEW TOPIC");

//    publum->publish(laserCloudregister);
//    pubsurrd->publish(laserCloudSurround);
//}

void callback(const sensor_msgs::PointCloud2ConstPtr& laser, const nav_msgs::Odometry::ConstPtr &odometry)
 {

  cout<<"receive"<<endl;
  double roll, pitch, yaw;
  double timeCloudRegistered = laser->header.stamp.toSec();
  geometry_msgs::Quaternion geoQuat=odometry->pose.pose.orientation;
 // Eigen::Quaterniond qu(geoQuat.z, geoQuat.x, -geoQuat.y, geoQuat.w);
   Eigen::Quaterniond qu(geoQuat.w,geoQuat.x,geoQuat.y, geoQuat.z);

   //     Eigen::Matrix4f m;
   //     m=lum.getTransformation(i).matrix();

   //     Eigen::Matrix3f f=m.block(0,0,3,3);
   //     Eigen::Quaternionf q=Eigen::Quaternionf(f);
   //     //Eigen::Quaterniond q=Eigen::Quaterniond(rotationMatrix)
   //    // Eigen::Quaternionf q=Eigen::Quaterniond(lum.getTransformation(i));
        geometry_msgs::PoseStamped this_pose;
        this_pose.pose.orientation=geoQuat;
        this_pose.pose.position=odometry->pose.pose.position;
        this_pose.header.stamp=ros::Time().fromSec(timeCloudRegistered);
        trajectory.poses.push_back(this_pose);




   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
   currpose[0]=-pitch;
   currpose[1]=-yaw;
   currpose[2]=roll;
   currpose[3]=odometry->pose.pose.position.x;
   currpose[4]=odometry->pose.pose.position.y;
   currpose[5]=odometry->pose.pose.position.z;
   Eigen::Isometry3d T(qu);
   T.pretranslate( Eigen::Vector3d(currpose[3],currpose[4],currpose[5] ));
   //T.translation()=Eigen::Vector3d(currpose[3],currpose[4],currpose[5]);
   //T.rotate(Eigen::Vector3d(currpose[0],currpose[1],currpose[2]));
   pcl::fromROSMsg(*laser, *laserCloudRegistered2);
   CloudPtr pc(new Cloud);
   pc->clear();
   for(int i=0;i<laserCloudRegistered2->points.size();i++)
   {
       float x,y,z;
      Eigen::Vector3d pointworld;
      // Eigen::Vector3d afterpoint;

      pointworld[0]=laserCloudRegistered2->points[i].x;
      pointworld[1]=laserCloudRegistered2->points[i].y;
      pointworld[2]=laserCloudRegistered2->points[i].z;
      Eigen::Vector3d afterpoint = T*pointworld;



       pointT pt;
       x=laserCloudRegistered2->points[i].x;
       y=laserCloudRegistered2->points[i].y;
       z=laserCloudRegistered2->points[i].z;
       //cout<<"after"<<afterpoint[0]<<endl;
       pt.x=afterpoint[0];
       pt.y=afterpoint[1];
       pt.z=afterpoint[2];
       pt.intensity=laserCloudRegistered2->points[i].intensity;
//        laserCloudRegistered2->points[i].x=z;
//        laserCloudRegistered2->points[i].y=x;
//        laserCloudRegistered2->points[i].z=y;

       pc->push_back(pt);
   }
   //Eigen::Vector4f center;
   Eigen::Vector4f ci;
   pcl::compute3DCentroid(*pc,ci);
//   cout<<"ci"<<ci<<endl;
//   cout<<"pc "<<pc->size()<<endl;
   center.push_back(ci);//push the centroid into the vector
   clouds.push_back( CloudPair(timeCloudRegistered,pc));
   lum.addPointCloud(pc/*,currpose*/);


   sensor_msgs::PointCloud2 syc_laserCloudSurround;
   sensor_msgs::PointCloud2 syc_laserCloudregister;



   bool _findloop=false;
   surround->clear();
   _findloop=findloopclosure();

      if(_findloop)
      {
          ROS_INFO_STREAM("FINDLOOP");
          lum.compute();
          trajectory.poses.clear();
        for(size_t i = 0; i < lum.getNumVertices (); i++)
          {
            //std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
            clouds[i].second = lum.getTransformedCloud (i);

            Eigen::Matrix4f m;
            m=lum.getTransformation(i).matrix();

            Eigen::Matrix3f f=m.block(0,0,3,3);
            Eigen::Quaternionf q=Eigen::Quaternionf(f);
            geometry_msgs::PoseStamped this_pose_stamped;
            //this_pose_stamped.pose=lum.getTransformation(i).matrix();
            this_pose_stamped.pose.orientation.x=q.x();
            this_pose_stamped.pose.orientation.y=q.y();
            this_pose_stamped.pose.orientation.z=q.z();
            this_pose_stamped.pose.orientation.w=q.w();
            this_pose_stamped.pose.position.x=m(3,0);
            this_pose_stamped.pose.position.y=m(3,1);
            this_pose_stamped.pose.position.z=m(3,2);


            this_pose_stamped.header.stamp=ros::Time().fromSec(clouds[i].first);
//            //cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;

            trajectory.poses.push_back(this_pose_stamped);
           // cout<<"points number "<<clouds[i].second->size()<<endl;
            //*surround+=*(clouds[i].second);
          }



      }
      for(size_t k=0;k<clouds.size();k++)
      {
        *surround+=*(clouds[k].second);
      }
 lasersurr->clear();
 downsample.setInputCloud(surround);
 downsample.filter(*lasersurr);

  pcl::toROSMsg(*pc, syc_laserCloudregister);
  pcl::toROSMsg(*lasersurr,syc_laserCloudSurround);
  syc_laserCloudSurround.header.stamp = ros::Time().fromSec(timeCloudRegistered);
  syc_laserCloudSurround.header.frame_id = "/camera_init";
  syc_laserCloudregister.header.stamp = ros::Time().fromSec(timeCloudRegistered);
  syc_laserCloudregister.header.frame_id = "/camera_init";

   ROS_INFO_STREAM("RECECEIVE NEW TOPIC");

   publum->publish(syc_laserCloudregister);
   pubsurrd->publish(syc_laserCloudSurround);
     // Solve all of perception here...
 }
 using namespace message_filters;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "sub_registered");
  ros::NodeHandle nh;
  trajectory.header.frame_id="/camera_init";
  message_filters::Subscriber<sensor_msgs::PointCloud2>syc_sub_points(nh,"/velodyne_points",1);
  message_filters::Subscriber<nav_msgs::Odometry>syc_sub_pose(nh,"/integrated_to_init2",1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), syc_sub_points,syc_sub_pose);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  downsample.setLeafSize(0.25, 0.25, 0.25);
  lum.setMaxIterations (lumIter);
  lum.setConvergenceThreshold (0.001f);
  //publish the path
 ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",2);
//ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init2",2,odohandler);
//ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
//  ros::Subscriber sub_points_world= nh.subscribe<sensor_msgs::PointCloud2>
//                                  ("/laser_cloud_registered2", 2, laserCloudHandler);
//ros::Subscriber sub_points_world= nh.subscribe<sensor_msgs::PointCloud2>
//                               ("/velodyne_points", 2, laserCloudHandler);
  ros::Publisher publishlumpoints=nh.advertise<sensor_msgs::PointCloud2>("/lumpoints", 1);
ros::Publisher publishlumsurr=nh.advertise<sensor_msgs::PointCloud2>("/lumsurround", 1);
pubpath=&path_pub;
publum=&publishlumpoints;
pubsurrd=&publishlumsurr;
ros::spin();
  return 0;
}
