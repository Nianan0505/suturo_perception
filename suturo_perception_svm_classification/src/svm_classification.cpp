#include "svm_classification.h"

#include "suturo_perception_ml_classifiers_msgs/CreateClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/AddClassData.h"
#include "suturo_perception_ml_classifiers_msgs/TrainClassifier.h"
#include "suturo_perception_ml_classifiers_msgs/ClassifyData.h"

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

using namespace suturo_perception_svm_classification;

SVMClassification::SVMClassification() 
{
  logger = suturo_perception_utils::Logger("svm_classification");
  service_wait_timeout = ros::Duration(10, 0);
}

bool 
SVMClassification::createClassifier(std::string identifier)
{
  if (!ros::service::waitForService("/ml_classifiers/create_classifier", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/create_classifier");
    return false;
  }
  ros::ServiceClient create_classifier = n.serviceClient<suturo_perception_ml_classifiers_msgs::CreateClassifier>("ml_classifiers/create_classifier", true);
  suturo_perception_ml_classifiers_msgs::CreateClassifier cc_srv;
  cc_srv.request.identifier = identifier;
  //cc_srv.request.class_type = "ml_classifiers/SVMClassifier";
  cc_srv.request.class_type = "ml_classifiers/NearestNeighborClassifier";
  if (create_classifier.call(cc_srv))
  {
    //logger.logInfo("call to create_classifier successful!");
  }
  else
  {
    logger.logError("Failed to call service create_classifier");
    return false;
  }

  //logger.logInfo(cc_srv.response.success?"create: true":"create: false");
  return true;

}

bool 
SVMClassification::addData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv)
{
  if (!ros::service::waitForService("/ml_classifiers/add_class_data", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/add_class_data");
    return false;
  }
  ros::ServiceClient add_class_data = n.serviceClient<suturo_perception_ml_classifiers_msgs::AddClassData>("/ml_classifiers/add_class_data", true);
  suturo_perception_ml_classifiers_msgs::AddClassData acd_srv;
  acd_srv.request.identifier = identifier;

  acd_srv.request.data = dpv;
  if (add_class_data.call(acd_srv))
  {
    //logger.logInfo("call to add_class_data successful!");
    return true;
  }
  else
  {
    logger.logError("Failed to call service add_class_data");
    return false;
  }

}

bool 
SVMClassification::trainClassifier(std::string identifier)
{
  if (!ros::service::waitForService("/ml_classifiers/train_classifier", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/train_classifier");
    return false;
  }
  ros::ServiceClient train_classifier = n.serviceClient<suturo_perception_ml_classifiers_msgs::TrainClassifier>("/ml_classifiers/train_classifier", true);
  suturo_perception_ml_classifiers_msgs::TrainClassifier t_srv;
  t_srv.request.identifier = identifier;
  if (train_classifier.call(t_srv))
  {
    //logger.logInfo("call to train_classifier successful!");
    return true;
  }
  else
  {
    logger.logError("Failed to call service train_classifier");
    return false;
  }
}

std::vector<std::string> 
SVMClassification::classifyData(std::string identifier, std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> dpv)
{
  std::vector<std::string> empty_ret;
  if (!ros::service::waitForService("/ml_classifiers/classify_data", service_wait_timeout))
  {
    logger.logError("timeout when waiting for service /ml_classifiers/classify_data");
    return empty_ret;
  }
  ros::ServiceClient classify_data = n.serviceClient<suturo_perception_ml_classifiers_msgs::ClassifyData>("/ml_classifiers/classify_data", true);
  suturo_perception_ml_classifiers_msgs::ClassifyData cd_srv;
  cd_srv.request.identifier = identifier;

  cd_srv.request.data = dpv;
  if (classify_data.call(cd_srv))
  {
    //logger.logInfo("call to classify_data successful!");
    return cd_srv.response.classifications;
  }
  else
  {
    //logger.logError("Failed to call service classify_data");
    return empty_ret;
  }
  
}

/**
 * there MUST be a directory called vfh_data containing directories and files with the following structure:
 * in each numbered directory there must be the SAME number of files!
 * an object MUST not be called "general"
 * vfh_data/
 *   objectname/
 *     1/
 *       00000_seg_cloud.pcd_vfh
 *       00001_seg_cloud.pcd_vfh
 *       ...
 *     2/
 *       ...
 *     ...
 */
bool
SVMClassification::trainVFHData()
{
  std::string package_path = ros::package::getPath("suturo_perception_svm_classification");
  ROS_INFO("svm classification path containing vfh training data = %s\n", package_path.c_str());
  package_path.append("/vfh_data");

  if (!boost::filesystem::exists (package_path) && !boost::filesystem::is_directory (package_path))
  {
    ROS_ERROR("can't find vfh training data path (doesn't exist or isn't a directory)");
    return false;
  }

  createClassifier("general");
  // iterating objectname directories
  for (boost::filesystem::directory_iterator it (package_path); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path().string();
      //ROS_INFO("Loading directory %s", ss.str().c_str());
      std::string noquot = ss.str();
      boost::replace_all(noquot, "\"", "");
      std::vector<std::string> dir_split = split(noquot, '/');
      std::string pose_classifier = dir_split.at(dir_split.size()-1);
      //ROS_INFO("Creating pose classifier %s", pose_classifier.c_str());
      createClassifier(pose_classifier.c_str());
      if (!loadVFHData(ss.str()))
      {
        ROS_ERROR("Failed to load directory %s", ss.str().c_str());
        return false;
      }
      ROS_INFO("training classifier %s", pose_classifier.c_str());
      trainClassifier(pose_classifier.c_str());
      ROS_INFO("done");
    }
    if (boost::filesystem::is_regular_file (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      ROS_INFO("unexpected file %s", ss.str().c_str());
    }
  }
  ROS_INFO("training classifier general");
  trainClassifier("general");
  ROS_INFO("done");
}

bool
SVMClassification::loadVFHData(std::string directory)
{
  boost::replace_all(directory, "\"", "");
  if (!boost::filesystem::exists (boost::filesystem::status(directory)))
  {
    ROS_ERROR("can't find vfh training data path (doesn't exist) (sub) %s", directory.c_str());
    return false;
  }

  if (!boost::filesystem::is_directory (directory))
  {
    ROS_ERROR("can't find vfh training data path (isn't a directory) (sub) %s", directory.c_str());
    return false;
  }

  std::vector<std::string> dir_split_tmp = split(directory, '/');
  std::string object_classifier = dir_split_tmp.at(dir_split_tmp.size()-1);

  std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> train_points;
  std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> pose_train_points;
  bool first_round = true;

  // iterating over numbered directories each containing one turn table round
  for (boost::filesystem::directory_iterator it (directory); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      //ROS_INFO("Loading directory %s", ss.str().c_str());
      int entry_cnt = 0;
      for (boost::filesystem::directory_iterator it2 (it->path()); it2 != boost::filesystem::directory_iterator (); ++it2)
      {
        if (boost::filesystem::is_regular_file (it2->status ()))
        {
          // load pcd file
          std::stringstream ss2;
          ss2 << it2->path ();
          //ROS_INFO("Loading file %s", ss2.str().c_str());
          std::string f = ss2.str();
          boost::replace_all(f, "\"", "");
          pcl::PointCloud<pcl::VFHSignature308> point;
          pcl::io::loadPCDFile(f, point);

          // convert content
          suturo_perception_ml_classifiers_msgs::ClassDataPoint cdp;
          suturo_perception_ml_classifiers_msgs::ClassDataPoint cdp2;
          for (int i = 0; i < 308; i++)
          {
            cdp.point.push_back(point.points[0].histogram[i]);
            cdp2.point.push_back(point.points[0].histogram[i]);
          }

          // add point for general matching
          cdp.target_class = object_classifier;
          //ROS_INFO("generated identifier: %s", cdp.target_class.c_str());
          train_points.push_back(cdp);

          // add point for pose matching
          std::stringstream target_class_ss;
          target_class_ss << object_classifier << entry_cnt;
          cdp2.target_class = target_class_ss.str();
          //ROS_INFO("pose estimation target class: %s", cdp2.target_class.c_str());

          pose_train_points.push_back(cdp2);

          // DEBUG stuff:
          /*
          std::stringstream ss3;
          for (int i = 0; i < 308; i++)
          {
            ss3 << point.points[0].histogram[i];
          }
          ROS_INFO("vfh descriptor: %s", ss3.str().c_str());
          */
          entry_cnt++;
        }
      }
      first_round = false;
    }
    if (boost::filesystem::is_regular_file (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      ROS_INFO("unexpected file %s", ss.str().c_str());
    }
  }

  addData(object_classifier, pose_train_points);

  addData("general", train_points);

  return true;
}

std::string
SVMClassification::classifyVFHSignature308(pcl::VFHSignature308 sig)
{
  std::string ret;

  // general
  suturo_perception_ml_classifiers_msgs::ClassDataPoint cdp;
  for (int i = 0; i < 308; i++)
  {
    cdp.point.push_back(sig.histogram[i]);
  }
  std::vector<suturo_perception_ml_classifiers_msgs::ClassDataPoint> arr;
  arr.push_back(cdp);
  std::vector<std::string> res = classifyData("general", arr);
  ret = res.at(0);

  // pose
  std::vector<std::string> res2 = classifyData(res.at(0), arr);
  ret.append("/");
  ret.append(res2.at(0));

  return ret;
}

// taken from: http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> &
SVMClassification::split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) 
  {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> 
SVMClassification::split(const std::string &s, char delim) 
{
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

