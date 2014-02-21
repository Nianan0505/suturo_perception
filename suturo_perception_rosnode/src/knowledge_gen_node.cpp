#include "ros/ros.h"
#include "suturo_perception_knowledge_rosnode.h"
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perceived_object.h"
#include <math.h>

#define PI 3.14159265

using namespace boost;
using namespace std;
using namespace suturo_perception_msgs;
namespace po = boost::program_options;

void create_arff(std::string filename);
void add_to_arff(std::string filename, suturo_perception_msgs::PerceivedObject obj, std::string cls);

int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception_knowledge");
  ros::NodeHandle nh;

  string dest_arff = "/tmp/knowledge.arff";
  string cls = "baguette";
  vector<string> bag_files;
  string topic = "/kinect_head/depth_registered/points";

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("dest,d", po::value<string>()->required(), "Destination directory for generated data")
    ("class,c", po::value<string>()->required(), "Class of input data")
    ("bag,b", po::value<vector<string> >()->required(), "Bag file(s) to process")
    ("topic,t", po::value<std::string>()->required(), "PointCloud topic to process")
  ;
  
  try
  {
    po::positional_options_description p;
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
          options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: knowledge_gen_node [OPTIONS]" << endl << endl;
      cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
    if(vm.count("dest"))
    {
      dest_arff = vm["dest"].as<string>();
    }
    if(vm.count("class"))
    {
      cls = vm["class"].as<string>();
    }
    if(vm.count("bag"))
    {
      bag_files = vm["bag"].as<vector<string> >();
    }
    if(vm.count("topic"))
    {
      topic = vm["topic"].as<string>();
    }
  }
  catch(std::exception& e)
  {
    cout << "Usage: knowledge_gen_node [OPTIONS]" << endl << endl;
    cout << desc << "\n";
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  SuturoPerceptionKnowledgeROSNode spr(nh, "");

  create_arff(dest_arff);
  
  BOOST_FOREACH(string bag_file, bag_files)
  {
    cout << "processing bag file " << bag_file << endl;
    rosbag::Bag bag(bag_file.c_str());
    rosbag::View view(bag, rosbag::TopicQuery(topic.c_str()));
    int i = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::PointCloud2::ConstPtr inputCloud = m.instantiate<sensor_msgs::PointCloud2>();
      if (inputCloud == NULL)
      {
        cout << "x";
        continue;
      }

      std::vector<suturo_perception_msgs::PerceivedObject> percObjs = spr.receive_cloud(inputCloud);

      BOOST_FOREACH(suturo_perception_msgs::PerceivedObject obj, percObjs)
      {
        add_to_arff(dest_arff, obj, cls);
      }
      i++;
      cout << ".";
    }
  }

  return (0);
}



void create_arff(std::string filename)
{
  ofstream arff_sink;
  std::string arff_header = "@relation knowledge\n" \
                            "@attribute red numeric\n" \
                            "@attribute green numeric\n" \
                            "@attribute blue numeric\n" \
                            "@attribute hue_sin numeric\n" \
                            "@attribute hue_cos numeric\n" \
                            "@attribute saturation numeric\n" \
                            "@attribute value numeric\n" \
                            "@attribute vol numeric\n" \
                            "@attribute length_1 numeric\n" \
                            "@attribute length_2 numeric\n" \
                            "@attribute length_3 numeric\n" \
                            "@attribute cuboid_length_relation_1 numeric\n" \
                            "@attribute cuboid_length_relation_2 numeric\n" \
                            "@attribute label_2d string\n" \
                            "@attribute shape numeric\n" \
                            "@attribute class {baguette,corny,dlink}\n" \
                            "@data\n";
  arff_sink.open (filename.c_str(), ios::out | ios::binary); 
  arff_sink << arff_header.c_str();
  arff_sink.close();
}

void add_to_arff(std::string filename, suturo_perception_msgs::PerceivedObject obj, std::string cls) {
  ofstream arff_sink;
  arff_sink.open (filename.c_str(), ios::out | ios::app | ios::binary); 

  int hue = obj.c_color_average_h;
  double l1 = obj.matched_cuboid.length1;
  double l2 = obj.matched_cuboid.length2;
  double l3 = obj.matched_cuboid.length3;
  double maxl = std::max(l1, std::max(l2, l3));
  double midl = std::max(l1, std::min(l2, l3));
  double minl = std::min(l1, std::min(l2, l3));
  std::string label_2d = obj.recognition_label_2d;
  if (label_2d.empty()) {
    label_2d = "?";
  }
  arff_sink << (int) obj.c_color_average_r  << ",";
  arff_sink << (int) obj.c_color_average_g  << ",";
  arff_sink << (int) obj.c_color_average_b  << ",";
  arff_sink << sin(hue * PI / 180) << ",";
  arff_sink << cos(hue * PI / 180) << ",";
  arff_sink << obj.c_color_average_s  << ",";
  arff_sink << obj.c_color_average_v  << ",";
  arff_sink << obj.matched_cuboid.volume  << ",";
  arff_sink << maxl << ",";
  arff_sink << midl << ",";
  arff_sink << minl << ",";
  arff_sink << (maxl / midl) << ",";
  arff_sink << (maxl / minl) << ",";
  arff_sink << label_2d.c_str() << ",";
  arff_sink << obj.c_shape << ",";
  arff_sink << cls << "\n";

  arff_sink.close();
}


// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
