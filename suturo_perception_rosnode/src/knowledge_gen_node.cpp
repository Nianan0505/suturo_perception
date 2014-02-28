#include "ros/ros.h"
#include <ros/package.h>
#include "suturo_perception_knowledge_rosnode.h"
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perceived_object.h"
#include <math.h>
#include <string>
#include <boost/algorithm/string.hpp>

#define PI 3.14159265

using namespace boost;
using namespace std;
using namespace suturo_perception_msgs;
namespace po = boost::program_options;

void create_arff(std::string filename, std::string classes);
void add_to_arff(std::string filename, suturo_perception_msgs::PerceivedObject obj, std::string cls);

int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception_knowledge");
  ros::NodeHandle nh;

  string dest_arff = "/tmp/knowledge.arff";
  vector<string> bag_files;
  string pctopic = "/kinect_head/depth_registered/points";
  string imgtopic = "/kinect_head/rgb/image_color";
  vector<string> classes;
  string classes_str;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("dest,d", po::value<string>()->required(), "Destination directory for generated data")
    ("bag,b", po::value<vector<string> >()->required(), "Bag file(s) to process with classname: CLASS:PATH_TO_BAG")
    ("pctopic,p", po::value<string>()->required(), "PointCloud topic to process")
    ("imgtopic,i", po::value<string>()->required(), "Image topic to process")
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
    if(vm.count("bag"))
    {
      vector<string> bags_parm = vm["bag"].as<vector<string> >();
      bool nfirst = true;
      BOOST_FOREACH(string bag_parm, bags_parm)
      {
        vector<string> parts;
        split(parts, bag_parm, is_any_of(":"));
        if (parts.size() != 2)
        {
          cerr << "bad bag parameter! more or less than one occurence of \":\"\n";
          return -1;
        }
        if (!(std::find(classes.begin(), classes.end(), parts.at(0)) != classes.end()))
        {
          classes.push_back(parts.at(0));
          classes_str.append(parts.at(0));
          if (nfirst)
          {
            nfirst = false;
          }
          else
          {
            classes_str.append(",");
          }
        }
        bag_files.push_back(parts.at(1));
      } 
    }
    if(vm.count("pctopic"))
    {
      pctopic = vm["pctopic"].as<string>();
    }
    if(vm.count("imgtopic"))
    {
      imgtopic = vm["imgtopic"].as<string>();
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

  std::string package_path = ros::package::getPath("suturo_perception_rosnode");
  stringstream ss;
  ss << package_path << "/data/milestone3_2d_db.yml";
  
  SuturoPerceptionKnowledgeROSNode spr(nh, ss.str());

  create_arff(dest_arff, classes_str);
  
  BOOST_FOREACH(string bag_file, bag_files)
  {
    cout << "processing bag file " << bag_file << endl;
    rosbag::Bag bag(bag_file.c_str());
    vector<string> topics;
    topics.push_back(pctopic);
    topics.push_back(imgtopic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int i = 0;
    sensor_msgs::PointCloud2::ConstPtr inputCloud;
    sensor_msgs::Image::ConstPtr inputImg;
    bool gotInputCloud = false;
    bool gotInputImg = false;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == pctopic)
      {
        gotInputCloud = true;
        inputCloud = m.instantiate<sensor_msgs::PointCloud2>();
        if (inputCloud == NULL)
        {
          cout << "x";
          gotInputCloud = false;
          gotInputImg = false;
          continue;
        }
      }
      if (m.getTopic() == imgtopic)
      {
        inputImg = m.instantiate<sensor_msgs::Image>();
        gotInputImg = true;
        if (inputImg == NULL)
        {
          cout << "X";
          gotInputCloud = false;
          gotInputImg = false;
          continue;
        }
      }

      if (gotInputCloud && gotInputImg) {
        gotInputCloud = false;
        gotInputImg = false;
        
        std::vector<suturo_perception_msgs::PerceivedObject> percObjs = spr.receive_image_and_cloud(inputImg, inputCloud);

        int i = 0;
        BOOST_FOREACH(suturo_perception_msgs::PerceivedObject obj, percObjs)
        {
          add_to_arff(dest_arff, obj, classes.at(i));
          i++;
        }
        i++;
        cout << ".";
      }
    }
  }

  cout << endl << "done" << endl;
  
  return (0);
}



void create_arff(std::string filename, std::string classes)
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
                            "@attribute class {";
  arff_header.append(classes);
  arff_header.append(       "}\n" \
                            "@data\n");
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
