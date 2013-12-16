#include "suturo_perception_rosnode.h"

const std::string SuturoPerceptionROSNode::TABLE_PLANE_TOPIC = "suturo_perception_table";
const std::string SuturoPerceptionROSNode::ALL_OBJECTS_ON_PLANE_TOPIC = "suturo_perception_objects_ontable"; // TODO use /suturo/objects_on_table
const std::string SuturoPerceptionROSNode::COLLISION_CLOUD_TOPIC = "suturo_perception_collision_cloud";
const std::string SuturoPerceptionROSNode::IMAGE_PREFIX_TOPIC= "/suturo/cluster_image/";
const std::string SuturoPerceptionROSNode::CROPPED_IMAGE_PREFIX_TOPIC= "/suturo/cropped_cluster_image/";
const std::string SuturoPerceptionROSNode::HISTOGRAM_PREFIX_TOPIC= "/suturo/cluster_histogram/";

namespace enc = sensor_msgs::image_encodings;

/*
 * Constructor
 */
SuturoPerceptionROSNode::SuturoPerceptionROSNode(ros::NodeHandle& n, std::string pt, std::string fi, std::string rd) : 
  nh(n), 
  pointTopic(pt), 
  frameId(fi),
  recognitionDir(rd),
  ph(n),
  visualizationPublisher(n, fi)
{
  logger = Logger("perception_rosnode");
  clusterService = nh.advertiseService("GetClusters", 
    &SuturoPerceptionROSNode::getClusters, this);
  
  is_edible_service = nh.serviceClient<suturo_perception_msgs::PrologQuery>("json_prolog/simple_query");
  is_edible_service_next = nh.serviceClient<suturo_perception_msgs::PrologNextSolution>("json_prolog/next_solution");
  is_edible_service_finish = nh.serviceClient<suturo_perception_msgs::PrologFinish>("json_prolog/finish");
  objectID = 0;

  // Init the topic for the plane segmentation result
	ph.advertise<sensor_msgs::PointCloud2>(TABLE_PLANE_TOPIC);

  // Init the topic for the segmented objects on the plane
	ph.advertise<sensor_msgs::PointCloud2>(ALL_OBJECTS_ON_PLANE_TOPIC);
  // Init collision cloud topic
	ph.advertise<sensor_msgs::PointCloud2>(COLLISION_CLOUD_TOPIC);
  // init 6 topics each for images and histograms
  for(int i = 0; i <= 6; ++i)
  {
    std::stringstream ss;
    ss << i;
    ph.advertise<sensor_msgs::Image>(IMAGE_PREFIX_TOPIC + ss.str());
    ph.advertise<sensor_msgs::Image>(CROPPED_IMAGE_PREFIX_TOPIC + ss.str());
    ph.advertise<sensor_msgs::Image>(HISTOGRAM_PREFIX_TOPIC + ss.str());
  }

  // Initialize dynamic reconfigure
  reconfCb = boost::bind(&SuturoPerceptionROSNode::reconfigureCallback, this, _1, _2);
  reconfSrv.setCallback(reconfCb);
  
  if(!recognitionDir.empty())
    object_matcher_.readTrainImagesFromDatabase(recognitionDir);

  object_matcher_.setVerboseLevel(0); // TODO use constant
  object_matcher_.setMinGoodMatches(7);

}

/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void SuturoPerceptionROSNode::receive_image_and_cloud(const sensor_msgs::ImageConstPtr& inputImage, const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // process only one cloud
  logger.logInfo("Receiving cloud");
  if(processing)
  {
    logger.logInfo("processing...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*inputCloud,*cloud_in);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

    // Make a deep copy of the passed cv::Mat and set a new
    // boost pointer to it.
    boost::shared_ptr<cv::Mat> img(new cv::Mat(cv_ptr->image.clone()));
    
    std::stringstream ss;
    ss << "Received a new point cloud: size = " << cloud_in->points.size();
    logger.logInfo((boost::format("Received a new point cloud: size = %s") % cloud_in->points.size()).str());
    sp.setOriginalRGBImage(img);
    sp.setOriginalCloud(cloud_in);
    sp.processCloudWithProjections(cloud_in);
    processing = false;
    logger.logInfo("Cloud processed. Lock buffer and return the results");      
  }
}

/*
 * Implementation of the GetClusters Service.
 *
 * This method will subscribe to the /camera/depth_registered/points topic, 
 * wait for the processing of a single point cloud, and return the result from
 * the calulations as a list of PerceivedObjects.
 */
bool SuturoPerceptionROSNode::getClusters(suturo_perception_msgs::GetClusters::Request &req,
  suturo_perception_msgs::GetClusters::Response &res)
{
  // ros::Subscriber sub;
  processing = true;

  // signal failed call, if request string does not match
  if (req.s.compare("get") != 0)
  {
    return false;
  }

  // TODO Make this configurable!!
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, pointTopic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pc_sub);

  sync.registerCallback(boost::bind(&SuturoPerceptionROSNode::receive_image_and_cloud,this, _1, _2));

  logger.logInfo("Waiting for processed cloud");
  ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud is received after 10s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
  while(processing)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime) processing = false;
    ros::spinOnce();
    r.sleep();
  }

  std::vector<cv::Mat> perceived_cluster_images;
  std::vector<cv::Mat> perceived_cluster_histograms;

  mutex.lock();
  perceivedObjects = sp.getPerceivedObjects();
  perceived_cluster_images = sp.getPerceivedClusterImages();
  
  // TODO: get this from new style pipeline later
  //perceived_cluster_histograms = sp.getPerceivedClusterHistograms();

  // ****** DEMO: Demo using Coloranalysis for handling of a capability in the new style pipeline ******
  // TODO: iterate over objects and fill perceived_cluster_histograms for publishing
  
  
  // Execution pipeline
  // Each capability provides an enrichment for the
  // returned PerceivedObject
  for (int i = 0; i < perceivedObjects.size(); i++) {
    ColorAnalysis ca(perceivedObjects[i], mutex);
    ca.execute();

    // Is 2d recognition enabled?
    if(!recognitionDir.empty())
    {
      // perceivedObjects[i].c_recognition_label_2d="";
      suturo_perception_2d_capabilities::LabelAnnotator2D la(perceivedObjects[i], mutex, sp.getOriginalRGBImage(), object_matcher_);
      la.execute();
    }
    else
    {
      // Set an empty label
      perceivedObjects[i].c_recognition_label_2d="";
    }

    // Publish the ROI-cropped images
    suturo_perception_2d_capabilities::ROIPublisher 
      rp(perceivedObjects.at(i), mutex, ph,sp.getOriginalRGBImage(),frameId);
    std::stringstream ss;
    ss << i;
    rp.setTopicName(CROPPED_IMAGE_PREFIX_TOPIC + ss.str());
    rp.execute();
  }

  // ColorAnalysis ca(perceivedObjects[0]);
  // ca.execute();
  
  res.perceivedObjs = *convertPerceivedObjects(&perceivedObjects); // TODO handle images in this method

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_publish = sp.getPlaneCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_publish = sp.getObjectsOnPlaneCloud();

	ph.publish_pointcloud(TABLE_PLANE_TOPIC,plane_cloud_publish, frameId);
	ph.publish_pointcloud(ALL_OBJECTS_ON_PLANE_TOPIC,object_cloud_publish, frameId);
  logger.logInfo((boost::format(" Extracted images vector: %s vs. Extracted PointCloud Vector: %s vs. Extracted histogram vector: %s") % perceived_cluster_images.size() % perceivedObjects.size() % perceived_cluster_histograms.size()).str());

  // Publish the images of the clusters
  for(int i = 0; i < perceived_cluster_images.size(); i++)
  {
    if (i > 6)
      continue;
    std::stringstream ss;
    ss << i;
    ph.publish_cv_mat(IMAGE_PREFIX_TOPIC + ss.str() , perceived_cluster_images.at(i), frameId);

  }

  // publish histograms
  for (int i = 0; i < perceived_cluster_histograms.size(); i++)
  {
    if (i > 6)
      continue;
    std::stringstream ss;
    ss << i;
    ph.publish_cv_mat(HISTOGRAM_PREFIX_TOPIC + ss.str(), perceived_cluster_histograms.at(i), frameId);
  }

  mutex.unlock();

  logger.logInfo("Shutting down subscriber");
  image_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  pc_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  // sync.shutdown();
  visualizationPublisher.publishMarkers(res.perceivedObjs);

  // ===============================================================
  // dirty demo hack. get objects and plane to merge collision_cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr collision_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  collision_cloud = sp.getPlaneCloud();

  if(collision_cloud != NULL)
  {
    // get edible
    int queryId = 0;
    for (std::vector<suturo_perception_msgs::PerceivedObject>::iterator it = res.perceivedObjs.begin(); 
      it != res.perceivedObjs.end(); ++it)
    {
      suturo_perception_msgs::PrologQuery pq;
      suturo_perception_msgs::PrologNextSolution pqn;
      suturo_perception_msgs::PrologFinish pqf;
      pq.request.mode = 0;
      pq.request.id = queryId;
      std::stringstream ss;
      ss << "is_edible([[" << queryId << ", '', '" << it->c_volume << "', 0]], Out)"; 
      logger.logDebug((boost::format("Knowledge query for collision_cloud: %s") % ss.str().c_str()).str()); 
      pq.request.query = ss.str();
      if(is_edible_service.call(pq))
      {
        
        pqn.request.id = queryId;
        is_edible_service_next.call(pqn);
        logger.logDebug((boost::format("SOLUTION: %s") % pqn.response.solution.c_str()).str());

        if(pqn.response.solution.empty()) logger.logDebug("Prolog returned fishy results");
        else
        {
          if(pqn.response.solution.substr(8, 1).compare("]") == 0)
          {
            logger.logDebug("Added to collision_cloud");
            *collision_cloud += *sp.collision_objects[queryId];  
          }
        }
        pqf.request.id = queryId;
        is_edible_service_finish.call(pqf);
      }
      else
      logger.logError("Knowledge not reachable");
      queryId++;
    }
    ph.publish_pointcloud(COLLISION_CLOUD_TOPIC, collision_cloud, frameId);
  }
  else
  {
    logger.logError("collision cloud (aka plane) is NULL ... skipping iteration");
  }

  logger.logInfo("Service call finished. return");
  return true;
}

/*
 * Callback for the dynamic reconfigure service
 */
void SuturoPerceptionROSNode::reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level)
{
  logger.logInfo((boost::format("Reconfigure request : \n"
            "zAxisFilterMin: %f \n"
            "zAxisFilterMax: %f \n"
            "downsampleLeafSize: %f \n"
            "planeMaxIterations: %i \n"
            "planeDistanceThreshold: %f \n"
            "ecClusterTolerance: %f \n"
            "ecMinClusterSize: %i \n"
            "ecMaxClusterSize: %i \n"
            "prismZMin: %f \n"
            "prismZMax: %f \n"
            "ecObjClusterTolerance: %f \n"
            "ecObjMinClusterSize: %i \n"
            "ecObjMaxClusterSize: %i \n") %
            config.zAxisFilterMin % config.zAxisFilterMax % config.downsampleLeafSize %
            config.planeMaxIterations % config.planeDistanceThreshold % config.ecClusterTolerance %
            config.ecMinClusterSize % config.ecMaxClusterSize % config.prismZMin % config.prismZMax %
            config.ecObjClusterTolerance % config.ecObjMinClusterSize % config.ecObjMaxClusterSize).str());
  /*while(processing) // wait until current processing run is completed 
  { 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }*/
  sp.setZAxisFilterMin(config.zAxisFilterMin);
  sp.setZAxisFilterMax(config.zAxisFilterMax);
  sp.setDownsampleLeafSize(config.downsampleLeafSize);
  sp.setPlaneMaxIterations(config.planeMaxIterations);
  sp.setPlaneDistanceThreshold(config.planeDistanceThreshold);
  sp.setEcClusterTolerance(config.ecClusterTolerance);
  sp.setEcMinClusterSize(config.ecMinClusterSize);
  sp.setEcMaxClusterSize(config.ecMaxClusterSize);
  sp.setPrismZMin(config.prismZMin);
  sp.setPrismZMax(config.prismZMax);
  sp.setEcObjClusterTolerance(config.ecObjClusterTolerance);
  sp.setEcObjMinClusterSize(config.ecObjMinClusterSize);
  sp.setEcObjMaxClusterSize(config.ecObjMaxClusterSize);
  logger.logInfo("Reconfigure successful");
}

// private methods

/*
 * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
 */
std::vector<suturo_perception_msgs::PerceivedObject> *SuturoPerceptionROSNode::convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects)
{
  std::vector<suturo_perception_msgs::PerceivedObject> *result = new std::vector<suturo_perception_msgs::PerceivedObject>();
  for (std::vector<suturo_perception_lib::PerceivedObject>::iterator it = objects->begin(); it != objects->end(); ++it)
  {
    suturo_perception_msgs::PerceivedObject *msgObj = new suturo_perception_msgs::PerceivedObject();
    msgObj->c_id = it->c_id;
    msgObj->c_shape = it->c_shape;
    msgObj->c_volume = it->c_volume;
    msgObj->c_centroid.x = it->c_centroid.x;
    msgObj->c_centroid.y = it->c_centroid.y;
    msgObj->c_centroid.z = it->c_centroid.z;
    msgObj->frame_id = frameId;
    msgObj->c_color_average_r = it->c_color_average_r;
    msgObj->c_color_average_g = it->c_color_average_g;
    msgObj->c_color_average_b = it->c_color_average_b;
    msgObj->c_color_average_h = it->c_color_average_h;
    msgObj->c_color_average_s = it->c_color_average_s;
    msgObj->c_color_average_v = it->c_color_average_v;
    msgObj->c_roi_origin.x = it->c_roi.origin.x;
    msgObj->c_roi_origin.y = it->c_roi.origin.y;
    msgObj->c_roi_width = it->c_roi.width;
    msgObj->c_roi_height = it->c_roi.height;
    msgObj->c_hue_histogram = it->c_hue_histogram;
    msgObj->c_hue_histogram_quality = it->c_hue_histogram_quality;
    msgObj->recognition_label_2d = it->c_recognition_label_2d;

    // these are not set for now
    msgObj->recognition_label_3d = "";
    result->push_back(*msgObj);
  }
  return result;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
