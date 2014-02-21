#include "suturo_perception_knowledge_rosnode.h"

/*
 * Constructor
 */
SuturoPerceptionKnowledgeROSNode::SuturoPerceptionKnowledgeROSNode(ros::NodeHandle& n, std::string rd) : 
  nh(n), 
  recognitionDir(rd)
{
  logger = Logger("perception_knowledge_rosnode");
  
  objectID = 0;

  // Initialize dynamic reconfigure
  reconfCb = boost::bind(&SuturoPerceptionKnowledgeROSNode::reconfigureCallback, this, _1, _2);
  reconfSrv.setCallback(reconfCb);
  
  if(!recognitionDir.empty())
    object_matcher_.readTrainImagesFromDatabase(recognitionDir);

  object_matcher_.setVerboseLevel(VERBOSE_MINIMAL);
  object_matcher_.setMinGoodMatches(7);

  numThreads = 8;

  // set default values for color_analysis if no reconfigure callback happens
  color_analysis_lower_s = 0.2;
  color_analysis_upper_s = 0.8;
  color_analysis_lower_v = 0.2;
  color_analysis_upper_v = 0.8;
}

/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
std::vector<suturo_perception_msgs::PerceivedObject> SuturoPerceptionKnowledgeROSNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // process only one cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloud,*cloud_in);
  logger.logInfo((boost::format("Received a new point cloud: size = %s") % cloud_in->points.size()).str());

  // Gazebo sends us unorganized pointclouds!
  // Reorganize them to be able to compute the ROI of the objects
  // This workaround is only tested for gazebo 1.9!
  if(!cloud_in->isOrganized ())
  {
    logger.logInfo((boost::format("Received an unorganized PointCloud: %d x %d .Convert it to a organized one ...") % cloud_in->width % cloud_in->height ).str());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    org_cloud->width = 640;
    org_cloud->height = 480;
    org_cloud->is_dense = false;
    org_cloud->points.resize(640 * 480);

    for (int i = 0; i < cloud_in->points.size(); i++) {
        pcl::PointXYZRGB result;
        result.x = 0;
        result.y = 0;
        result.z = 0;
        org_cloud->points[i]=cloud_in->points[i];
    }

    cloud_in = org_cloud;
  }
  
  logger.logInfo("processing...");
  sp.setOriginalCloud(cloud_in);
  sp.processCloudWithProjections(cloud_in);
  logger.logInfo("Cloud processed. Lock buffer and return the results");      

  mutex.lock();
  perceivedObjects = sp.getPerceivedObjects();

  // Execution pipeline
  // Each capability provides an enrichment for the
  // returned PerceivedObject

  // initialize threadpool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < numThreads; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
      );
  }

  for (int i = 0; i < perceivedObjects.size(); i++) 
  {
    // Initialize Capabilities
    ColorAnalysis ca(perceivedObjects[i]);
    ca.setLowerSThreshold(color_analysis_lower_s);
    ca.setUpperSThreshold(color_analysis_upper_s);
    ca.setLowerVThreshold(color_analysis_lower_v);
    ca.setUpperVThreshold(color_analysis_upper_v);
    suturo_perception_shape_detection::RandomSampleConsensus sd(perceivedObjects[i]);
    //suturo_perception_vfh_estimation::VFHEstimation vfhe(perceivedObjects[i]);
    // suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i]);
    // Init the cuboid matcher with the table coefficients
    suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i], sp.getTableCoefficients() );

    // post work to threadpool
    ioService.post(boost::bind(&ColorAnalysis::execute, ca));
    ioService.post(boost::bind(&suturo_perception_shape_detection::RandomSampleConsensus::execute, sd));
    //ioService.post(boost::bind(&suturo_perception_vfh_estimation::VFHEstimation::execute, vfhe));
    ioService.post(boost::bind(&suturo_perception_3d_capabilities::CuboidMatcherAnnotator::execute, cma));

    // Is 2d recognition enabled?
    // Set an empty label
    perceivedObjects[i].set_c_recognition_label_2d("");
  }
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();

  std::vector<suturo_perception_msgs::PerceivedObject> perceivedObjs = *convertPerceivedObjects(&perceivedObjects); // TODO handle images in this method

  mutex.unlock();

  return perceivedObjs;
}

/*
 * Callback for the dynamic reconfigure service
 */
void SuturoPerceptionKnowledgeROSNode::reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level)
{
  logger.logInfo((boost::format("Reconfigure request : \n"
            "segmenter: zAxisFilterMin: %f \n"
            "segmenter: zAxisFilterMax: %f \n"
            "segmenter: downsampleLeafSize: %f \n"
            "segmenter: planeMaxIterations: %i \n"
            "segmenter: planeDistanceThreshold: %f \n"
            "segmenter: ecClusterTolerance: %f \n"
            "segmenter: ecMinClusterSize: %i \n"
            "segmenter: ecMaxClusterSize: %i \n"
            "segmenter: prismZMin: %f \n"
            "segmenter: prismZMax: %f \n"
            "segmenter: ecObjClusterTolerance: %f \n"
            "segmenter: ecObjMinClusterSize: %i \n"
            "segmenter: ecObjMaxClusterSize: %i \n"
            "colorAnalysis: hsvFilterLowerSThreshold: %f \n"
            "colorAnalysis: hsvFilterUpperSThreshold: %f \n"
            "colorAnalysis: hsvFilterLowerVThreshold: %f \n"
            "colorAnalysis: hsvFilterUpperVThreshold: %f \n"
            "general: numThreads: %i \n") %
            config.zAxisFilterMin % config.zAxisFilterMax % config.downsampleLeafSize %
            config.planeMaxIterations % config.planeDistanceThreshold % config.ecClusterTolerance %
            config.ecMinClusterSize % config.ecMaxClusterSize % config.prismZMin % config.prismZMax %
            config.ecObjClusterTolerance % config.ecObjMinClusterSize % config.ecObjMaxClusterSize % 
            config.hsvFilterLowerSThreshold % config.hsvFilterUpperSThreshold % 
            config.hsvFilterLowerVThreshold % config.hsvFilterUpperVThreshold % 
            config.numThreads).str());
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
  color_analysis_lower_s = config.hsvFilterLowerSThreshold;
  color_analysis_upper_s = config.hsvFilterUpperSThreshold;
  color_analysis_lower_v = config.hsvFilterLowerVThreshold;
  color_analysis_upper_v = config.hsvFilterUpperVThreshold;
  logger.logInfo("Reconfigure successful");
}

// private methods

/*
 * Convert suturo_perception_lib::PerceivedObject list to suturo_perception_msgs:PerceivedObject list
 */
//std::vector<suturo_perception_msgs::PerceivedObject> *SuturoPerceptionKnowledgeROSNode::convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects)
std::vector<suturo_perception_msgs::PerceivedObject> *SuturoPerceptionKnowledgeROSNode::convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject, Eigen::aligned_allocator<suturo_perception_lib::PerceivedObject> > *objects)
{
  std::vector<suturo_perception_msgs::PerceivedObject> *result = new std::vector<suturo_perception_msgs::PerceivedObject>();
  for (std::vector<suturo_perception_lib::PerceivedObject, Eigen::aligned_allocator<suturo_perception_lib::PerceivedObject> >::iterator it = objects->begin(); it != objects->end(); ++it)
  {
    suturo_perception_msgs::PerceivedObject *msgObj = new suturo_perception_msgs::PerceivedObject();
    msgObj->c_id = it->get_c_id();
    msgObj->c_shape = it->get_c_shape();
    msgObj->c_volume = it->get_c_volume();
    msgObj->c_centroid.x = it->get_c_centroid().x;
    msgObj->c_centroid.y = it->get_c_centroid().y;
    msgObj->c_centroid.z = it->get_c_centroid().z;
    msgObj->frame_id = "";
    msgObj->c_color_average_r = it->get_c_color_average_r();
    msgObj->c_color_average_g = it->get_c_color_average_g();
    msgObj->c_color_average_b = it->get_c_color_average_b();
    msgObj->c_color_average_h = it->get_c_color_average_h();
    msgObj->c_color_average_s = it->get_c_color_average_s();
    msgObj->c_color_average_v = it->get_c_color_average_v();
    msgObj->c_color_average_qh = it->get_c_color_average_qh();
    msgObj->c_color_average_qs = it->get_c_color_average_qs();
    msgObj->c_color_average_qv = it->get_c_color_average_qv();
    msgObj->c_roi_origin.x = it->get_c_roi().origin.x;
    msgObj->c_roi_origin.y = it->get_c_roi().origin.y;
    msgObj->c_roi_width = it->get_c_roi().width;
    msgObj->c_roi_height = it->get_c_roi().height;
    msgObj->c_hue_histogram = *it->get_c_hue_histogram();
    msgObj->c_hue_histogram_quality = it->get_c_hue_histogram_quality();
    msgObj->recognition_label_2d = it->get_c_recognition_label_2d();
  
    Cuboid c = it->get_c_cuboid();
    msgObj->matched_cuboid.length1  = c.length1;
    msgObj->matched_cuboid.length2  = c.length2;
    msgObj->matched_cuboid.length3  = c.length3;
    msgObj->matched_cuboid.volume   = c.volume;
    msgObj->matched_cuboid.pose.position.x   = c.center(0);
    msgObj->matched_cuboid.pose.position.y   = c.center(1);
    msgObj->matched_cuboid.pose.position.z   = c.center(2);
    msgObj->matched_cuboid.pose.orientation.x   = c.orientation.x();
    msgObj->matched_cuboid.pose.orientation.y   = c.orientation.y();
    msgObj->matched_cuboid.pose.orientation.y   = c.orientation.z();
    msgObj->matched_cuboid.pose.orientation.w   = c.orientation.w();

    for (int i = 0; i < 308; i++) 
    {
      msgObj->c_vfh_estimation.push_back(it->get_c_vfhs().histogram[i]);
    }
    msgObj->c_svm_result = ""; //svm_classification.classifyVFHSignature308(it->get_c_vfhs());
    msgObj->c_pose = 0; //svm_classification.classifyPoseVFHSignature308(it->get_c_vfhs(), msgObj->c_svm_result);

    // these are not set for now
    msgObj->recognition_label_3d = "";
    
    result->push_back(*msgObj);
  }
  return result;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
