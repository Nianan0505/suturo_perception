#include "ros/ros.h"
#include "suturo_perception_knowledge_rosnode.h"
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perceived_object.h"

using namespace boost;
using namespace std;
using namespace suturo_perception_msgs;
namespace po = boost::program_options;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "suturo_perception_knowledge");
  ros::NodeHandle nh;

  string dest_dir = "/tmp";
  string cls = "baguette";
  string bag_file = "test.bag";
  string topic = "/kinect_head/depth_registered/points";

  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("dest,t", po::value<std::vector<std::string> >()->required(), "Destination directory for generated data")
      ("class,t", po::value<std::vector<std::string> >()->required(), "Class of input data")
      ("bag,t", po::value<std::vector<std::string> >()->required(), "Bag file to process")
      ("topic,t", po::value<std::vector<std::string> >()->required(), "Bag file to process")
    ;
  
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
      dest_dir = vm["dest"].as<std::vector<std::string> >().at(0);
    }
    if(vm.count("class"))
    {
      cls = vm["class"].as<std::vector<std::string> >().at(0);
    }
    if(vm.count("bag"))
    {
      bag_file = vm["bag"].as<std::vector<std::string> >().at(0);
    }
    if(vm.count("topic"))
    {
      topic = vm["topic"].as<std::vector<std::string> >().at(0);
    }
  }
  catch(std::exception& e)
  {
    cout << "Usage: knowledge_gen_node [OPTIONS]" << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  // ros strangeness strikes again. don't try to && these!
  SuturoPerceptionKnowledgeROSNode spr(nh, "");

  rosbag::Bag bag(bag_file.c_str());
  rosbag::View view(bag, rosbag::TopicQuery(topic.c_str()));
  int i = 0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr inputCloud = m.instantiate<sensor_msgs::PointCloud2>();
    if (inputCloud == NULL)
    {
      continue;
    }

    spr.receive_cloud(inputCloud);

    i++;
  }

  return (0);
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
