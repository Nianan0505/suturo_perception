/**
 * This class implements the client interface to query
 * the PerceivedObjects from the PerceptionServer.
 *
 * It will execute a service call to "GetClusters" and print
 * the returned list of PerceivedObjects in a textual form.
 */
#include "ros/ros.h"
#include "suturo_perception_msgs/GetClusters.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/program_options.hpp>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>


#define PI 3.14159265

using namespace boost;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  string dest_dir = "/tmp";
  string cls = "baguette";

  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("dest,t", po::value<std::vector<std::string> >()->required(), "Destination directory for generated data")
      ("class,t", po::value<std::vector<std::string> >()->required(), "Class of input data")
    ;
  
    po::positional_options_description p;
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
          options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: knowledge_gen [OPTIONS]" << endl << endl;
      cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
    if(vm.count("dest"))
    {
      dest_dir = vm["dest"].as<std::vector<std::string> >().at(0);
    }
    if(vm.count("class"))
    {
      cls = vm["class"].as<std::vector<std::string> >().at(0);
    }
  }
  catch(std::exception& e)
  {
    cout << "Usage: knowledge_gen [OPTIONS]" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  std::string arff_header = "@relation knowledge\n" \
                            "@attribute hue_sin numeric\n" \
                            "@attribute hue_cos numeric\n" \
                            "@attribute vol numeric\n" \
                            "@attribute cuboid_length_relation_1 numeric\n" \
                            "@attribute cuboid_length_relation_2 numeric\n" \
                            "@attribute label_2d string\n" \
                            "@attribute shape numeric\n" \
                            "@attribute class {baguette,corny,dlink}\n" \
                            "@data\n";
  std::string arff_file = dest_dir + "/knowledge-gen.arff";
  
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetClusters>("/suturo/GetClusters");
  suturo_perception_msgs::GetClusters clusterSrv;
  clusterSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  
  ofstream arff_sink;
  
  arff_sink.open (arff_file.c_str(), ios::out | ios::app | ios::binary); 
  arff_sink << arff_header.c_str();
  arff_sink.close();

  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(clusterSrv))
    {
      ROS_INFO("Cluster Service call successful");
      ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );

      // wait a sec if list is empty. service may not be ready yet
      if((long int)clusterSrv.response.perceivedObjs.size() == 0)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_INFO_STREAM("No objects perceived yet.");
        continue;
      }
      
      if (clusterSrv.response.perceivedObjs.size() != 1) {
        ROS_WARN("Found more than one object, skipping!");
        continue;
      }

      arff_sink.open (arff_file.c_str(), ios::out | ios::app | ios::binary); 

      for(int i=0; i < clusterSrv.response.perceivedObjs.size(); i++ ) {
        int hue = clusterSrv.response.perceivedObjs[i].c_color_average_h;
        double l1 = clusterSrv.response.perceivedObjs[i].matched_cuboid.length1;
        double l2 = clusterSrv.response.perceivedObjs[i].matched_cuboid.length2;
        double l3 = clusterSrv.response.perceivedObjs[i].matched_cuboid.length3;
        double maxl = std::max(l1, std::max(l2, l3));
        double midl = std::max(l1, std::min(l2, l3));
        double minl = std::min(l1, std::min(l2, l3));
        std::string label_2d = clusterSrv.response.perceivedObjs[i].recognition_label_2d;
        if (label_2d.empty()) {
          label_2d = "?";
        }
        arff_sink << sin(hue * PI / 180) << ",";
        arff_sink << cos(hue * PI / 180) << ",";
        arff_sink << clusterSrv.response.perceivedObjs[i].matched_cuboid.volume  << ",";
        arff_sink << (maxl / midl) << ",";
        arff_sink << (maxl / minl) << ",";
        arff_sink << label_2d.c_str() << ",";
        arff_sink << clusterSrv.response.perceivedObjs[i].c_shape << ",";
        arff_sink << cls << "\n";

        arff_sink.close();
        
        ROS_INFO("ID of perceived object is: %d", clusterSrv.response.perceivedObjs[i].c_id);
        ROS_INFO("Frame of perceived object is: %s", clusterSrv.response.perceivedObjs[i].frame_id.c_str());
        ROS_INFO("Volume of perceived object is: %f", clusterSrv.response.perceivedObjs[i].c_volume);
        ROS_INFO("Shape of perceived object is: %i", clusterSrv.response.perceivedObjs[i].c_shape);
        ROS_INFO("Average RGB color of perceived object is: r: %i  g: %i  b: %i", 
                  clusterSrv.response.perceivedObjs[i].c_color_average_r,
                  clusterSrv.response.perceivedObjs[i].c_color_average_g,
                  clusterSrv.response.perceivedObjs[i].c_color_average_b);
        ROS_INFO("Average HSV color of perceived object is: h: %i  s: %f  v: %f", 
                  clusterSrv.response.perceivedObjs[i].c_color_average_h,
                  clusterSrv.response.perceivedObjs[i].c_color_average_s,
                  clusterSrv.response.perceivedObjs[i].c_color_average_v);
        ROS_INFO("Average HSV color with quality filter of perceived object is: h: %i  s: %f  v: %f", 
                  clusterSrv.response.perceivedObjs[i].c_color_average_qh,
                  clusterSrv.response.perceivedObjs[i].c_color_average_qs,
                  clusterSrv.response.perceivedObjs[i].c_color_average_qv);
        ROS_INFO("2D recognition label of perceived object is: %s", clusterSrv.response.perceivedObjs[i].recognition_label_2d.c_str());
        ROS_INFO("ROI of object: (origin, width, height): ( %f x %f ) , %i , %i ",
          clusterSrv.response.perceivedObjs[i].c_roi_origin.x,
          clusterSrv.response.perceivedObjs[i].c_roi_origin.y,
          clusterSrv.response.perceivedObjs[i].c_roi_width, 
          clusterSrv.response.perceivedObjs[i].c_roi_height);
        ROS_INFO("Quality of hue histogram: %d ", clusterSrv.response.perceivedObjs[i].c_hue_histogram_quality);
        ROS_INFO("Centroid(x) of perceived object is: %f , %f , %f ",
          clusterSrv.response.perceivedObjs[i].c_centroid.x,
          clusterSrv.response.perceivedObjs[i].c_centroid.y,
          clusterSrv.response.perceivedObjs[i].c_centroid.z);
        std::stringstream vfh_ss;
        for (int j = 0; j < clusterSrv.response.perceivedObjs[i].c_vfh_estimation.size(); j++)
        {
          vfh_ss << clusterSrv.response.perceivedObjs[i].c_vfh_estimation.at(j);
          vfh_ss << " ";
        }
        ROS_INFO("Matched cuboid: %f x %f x %f , Vol: %f",
          clusterSrv.response.perceivedObjs[i].matched_cuboid.length1,
          clusterSrv.response.perceivedObjs[i].matched_cuboid.length2,
          clusterSrv.response.perceivedObjs[i].matched_cuboid.length3,
          clusterSrv.response.perceivedObjs[i].matched_cuboid.volume);
      }
      ROS_INFO_STREAM("------------------------------------------------------------");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetClusters");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
