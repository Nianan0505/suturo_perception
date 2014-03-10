#include "ros/ros.h"
#include <ros/package.h>
#include "suturo_perception_knowledge_rosnode.h"
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perceived_object.h"
#include <math.h>
#include <string>
#include <boost/algorithm/string.hpp>

#define PI 3.14159265

using namespace boost;
using namespace boost::filesystem;
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
  string bag_directory;
  vector<string> bags_parm;
  string pctopic = "/kinect_head/depth_registered/points";
  string imgtopic = "/kinect_head/rgb/image_color";
  vector<string> classes;
  string classes_str;

  stringstream debuginfo;
  
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("dest,d", po::value<string>()->required(), "Destination directory for generated data")
    ("bag,b", po::value<string>()->required(), "Directory structure containing bag files. Each subdirectory in this given path represents one class and has to contain bag files that correspond to that class. All bag files must supply the data on the same topic")
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
      bag_directory = vm["bag"].as<string>();
      path main_path(bag_directory.c_str());
      try
      {
        if (!exists(main_path))
        {
          cout << "Directory doesn't exist: " << bag_directory.c_str() << endl;
          return -1;
        }
        if (!is_directory(main_path))
        {
          cout << bag_directory.c_str() << " is not a directory" << endl;
          return -1;
        }

        vector<path> subdirs;
        copy(directory_iterator(main_path), directory_iterator(), back_inserter(subdirs));

        bool first = true;
        for (vector<path>::const_iterator it (subdirs.begin()); it != subdirs.end(); ++it)
        {
          if (is_directory(*it))
          {
            cout << "Entering " << *it << endl;
            vector<path> subfiles;
            string cls = (*it).filename().string(); 
            if (!(std::find(classes.begin(), classes.end(), cls) != classes.end()))
            {
              classes.push_back(cls);
              if (first)
              {
                first = false;
              }
              else
              {
                classes_str.append(",");
              }
              classes_str.append(cls);
            }
            copy(directory_iterator(*it), directory_iterator(), back_inserter(subfiles));
            for (vector<path>::const_iterator bagfile (subfiles.begin()); bagfile != subfiles.end(); ++bagfile)
            {
              if (is_regular_file(*bagfile))
              {
                string bag_parm = "";
                bag_parm.append(cls);
                bag_parm.append(":");
                bag_parm.append((*bagfile).c_str()); 
                bags_parm.push_back(bag_parm);
              }
              else 
              {
                cout << *it << " is not a regular file, skipping" << endl;
              }
            }
          }
          else
          {
            cout << "Not a directory, skipping: " << *it << endl;
          } 
        }
      }
      catch (const filesystem_error& ex)
      {
        cout << ex.what() << endl;
        return -1;
      }
      /*
      bool first = true;
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
          if (first)
          {
            first = false;
          }
          else
          {
            classes_str.append(",");
          }
          classes_str.append(parts.at(0));
        }
        bag_files.push_back(parts.at(1));
        debuginfo << parts.at(0) << " -> " << parts.at(1) << endl;
      } 
      */
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

  int i = 0;
  BOOST_FOREACH(string cls_bag_file, bags_parm)
  {
    vector<string> parts;
    split(parts, cls_bag_file, is_any_of(":"));
    string cls = parts.at(0);
    string bag_file = parts.at(1);
    debuginfo << endl << "processing bag file " << bag_file << endl;
    rosbag::Bag bag(bag_file.c_str());
    vector<string> topics;
    topics.push_back(pctopic);
    topics.push_back(imgtopic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
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
          debuginfo << "x";
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
          debuginfo << "X";
          gotInputCloud = false;
          gotInputImg = false;
          continue;
        }
      }

      if (gotInputCloud && gotInputImg) {
        gotInputCloud = false;
        gotInputImg = false;
        
        std::vector<suturo_perception_msgs::PerceivedObject> percObjs = spr.receive_image_and_cloud(inputImg, inputCloud);

        if (percObjs.size() != 1) {
          debuginfo << "found more than one perceived object in one scene... skipping" << endl;
          continue;
        } else {
          BOOST_FOREACH(suturo_perception_msgs::PerceivedObject obj, percObjs)
          {
            add_to_arff(dest_arff, obj, cls);
          }
          debuginfo << ".";
        }
      }
    }
    i++;
  }

  debuginfo << endl << "done" << endl;
  cout << debuginfo.str().c_str();
  
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
                            "@attribute label_2d {baguette,corny,wlanadapter,dlink,cafetfilter}\n" \
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
  if (isnan(l1) || isnan(l2) || isnan(l3) || isnan(maxl) || isnan(midl) || isnan(minl) || isnan(maxl / minl) || isnan(maxl / midl)) 
  {
    arff_sink << "?,?,?,?,?,";
  }
  else
  {
    arff_sink << maxl << ",";
    arff_sink << midl << ",";
    arff_sink << minl << ",";
    arff_sink << (maxl / midl) << ",";
    arff_sink << (maxl / minl) << ",";
  }
  arff_sink << label_2d.c_str() << ",";
  arff_sink << obj.c_shape << ",";
  arff_sink << cls << "\n";

  arff_sink.close();
}


// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
