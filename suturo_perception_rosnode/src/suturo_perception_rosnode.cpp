#include "suturo_perception_rosnode.h"

const std::string SuturoPerceptionROSNode::TABLE_PLANE_TOPIC = "suturo/table";
const std::string SuturoPerceptionROSNode::ALL_OBJECTS_ON_PLANE_TOPIC = "suturo/objects_on_table"; // TODO use /suturo/objects_on_table
const std::string SuturoPerceptionROSNode::COLLISION_CLOUD_TOPIC = "suturo/collision_cloud";
const std::string SuturoPerceptionROSNode::IMAGE_PREFIX_TOPIC= "/suturo/cluster_image/";
const std::string SuturoPerceptionROSNode::CROPPED_IMAGE_PREFIX_TOPIC= "/suturo/cropped_cluster_image/";
const std::string SuturoPerceptionROSNode::HISTOGRAM_PREFIX_TOPIC= "/suturo/cluster_histogram/";

namespace enc = sensor_msgs::image_encodings;

/*
 * Constructor
 */
SuturoPerceptionROSNode::SuturoPerceptionROSNode(ros::NodeHandle& n, std::string pt, std::string ct, std::string fi, std::string rd) : 
  nh(n), 
  pointTopic(pt),
  colorTopic(ct), 
  frameId(fi),
  recognitionDir(rd),
  ph(n),
  visualizationPublisher(n, fi)
{
  logger = Logger("perception_rosnode");
  clusterService = nh.advertiseService("/suturo/GetClusters", 
    &SuturoPerceptionROSNode::getClusters, this);
  
  is_edible_service = nh.serviceClient<suturo_perception_msgs::PrologQuery>("json_prolog/simple_query");
  is_edible_service_next = nh.serviceClient<suturo_perception_msgs::PrologNextSolution>("json_prolog/next_solution");
  is_edible_service_finish = nh.serviceClient<suturo_perception_msgs::PrologFinish>("json_prolog/finish");
  objectID = 0;

  // VFH + SVM stuff
  logger.logInfo("Loading VFH training data for SVM...");
  svm_classification.trainVFHData();

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

  object_matcher_.setVerboseLevel(VERBOSE_MINIMAL);
  object_matcher_.setMinGoodMatches(7);

  numThreads = 8;
  callback_called = false;
  fallback_enabled = false;

  // set default values for color_analysis if no reconfigure callback happens
  color_analysis_lower_s = 0.2;
  color_analysis_upper_s = 0.8;
  color_analysis_lower_v = 0.2;
  color_analysis_upper_v = 0.8;
}

/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void SuturoPerceptionROSNode::receive_image_and_cloud(const sensor_msgs::ImageConstPtr& inputImage, 
                                                      const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // process only one cloud
  callback_called = true;
  if(processing)
  {
    logger.logInfo("Receiving cloud");
    logger.logInfo("processing...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*inputCloud,*cloud_in);

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
    if(!fallback_enabled)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

      // Make a deep copy of the passed cv::Mat and set a new
      // boost pointer to it.
      boost::shared_ptr<cv::Mat> img(new cv::Mat(cv_ptr->image.clone()));
      sp.setOriginalRGBImage(img);
    }
    
    std::stringstream ss;
    ss << "Received a new point cloud: size = " << cloud_in->points.size();
    logger.logInfo((boost::format("Received a new point cloud: size = %s") % cloud_in->points.size()).str());
    sp.setOriginalCloud(cloud_in);
    sp.processCloudWithProjections(cloud_in);
    processing = false;
    logger.logInfo("Cloud processed. Lock buffer and return the results");      
  }
  callback_called = false;
}

/*
 * Fallback, if only pointcloud data is available.
 */
void SuturoPerceptionROSNode::fallback_receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  if(processing)
  {
    logger.logInfo("Received a pointcloud message");
    sensor_msgs::ImageConstPtr nullPtr; // boost::shared_ptr NullPtr
    receive_image_and_cloud(nullPtr, inputCloud);
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
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();

  processing = true;

  // signal failed call, if request string does not match
  if (req.s.compare("get") != 0)
  {
    return false;
  }

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, colorTopic, 1);
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
    if(boost::posix_time::second_clock::local_time() >= cancelTime && !callback_called)
    {
      processing = false;
      // register normal callback just for the cloud topic and retry, if no color data is available
      if(!fallback_enabled)
      {
        processing = true;
        logger.logWarn("No color image received. Falling back to point clouds only.");
        image_sub.unsubscribe();
        pc_sub.unsubscribe();
        sub_cloud = nh.subscribe(pointTopic, 1, 
          &SuturoPerceptionROSNode::fallback_receive_cloud, this);
        fallback_enabled = true;
        cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
      }
      else // fallback failed as well. no data available. Abort service call.
      {
        logger.logError("No sensor data available. Aborting.");
        return false;
      }
    } 
    ros::spinOnce();
    r.sleep();
  }

  std::vector<cv::Mat> perceived_cluster_images;

  mutex.lock();
  perceivedObjects = sp.getPerceivedObjects();
  perceived_cluster_images = sp.getPerceivedClusterImages();

  // If the image dimension is bigger then
  // the dimension of the pointcloud, we have to adjust the ROI of every
  // perceived object
  if (sp.getOriginalRGBImage() != NULL)
  {
    if(sp.getOriginalRGBImage()->cols != sp.getOriginalCloud()->width
        && sp.getOriginalRGBImage()->rows != sp.getOriginalCloud()->height)
    {
      // std::cout << "Image dimensions differ from PC dimensions: ";
      // std::cout << "Image " <<  sp.getOriginalRGBImage()->cols << "x" << sp.getOriginalRGBImage()->rows;
      // std::cout << "vs. Cloud " <<  sp.getOriginalCloud()->width << "x" << sp.getOriginalCloud()->height << std::endl;

      // Adjust the ROI if the image is at 1280x1024 and the pointcloud is at 640x480
      // Adjust the ROI if the image is at 1280x960 and the pointcloud is at 640x480 (Gazebo Mode)
      if( (sp.getOriginalRGBImage()->cols == 1280 && sp.getOriginalRGBImage()->rows == 1024) ||
      (sp.getOriginalRGBImage()->cols == 1280 && sp.getOriginalRGBImage()->rows == 960) )
      {
        for (int i = 0; i < perceivedObjects.size(); i++) {
            ROI roi = perceivedObjects.at(i).get_c_roi();
            roi.origin.x*=2;
            roi.origin.y*=2;
            roi.width*=2;
            roi.height*=2;
            perceivedObjects.at(i).set_c_roi(roi);
        }
      }
      else
      {
        logger.logError("UNSUPPORTED MIXTURE OF IMAGE AND POINTCLOUD DIMENSIONS");
      }

    }
  }
  
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
    suturo_perception_vfh_estimation::VFHEstimation vfhe(perceivedObjects[i]);

    // post work to threadpool
    ioService.post(boost::bind(&ColorAnalysis::execute, ca));
    ioService.post(boost::bind(&suturo_perception_shape_detection::RandomSampleConsensus::execute, sd));
    ioService.post(boost::bind(&suturo_perception_vfh_estimation::VFHEstimation::execute, vfhe));

    // Is 2d recognition enabled?
    if(!recognitionDir.empty() && !fallback_enabled)
    {
      // perceivedObjects[i].c_recognition_label_2d="";
      suturo_perception_2d_capabilities::LabelAnnotator2D la(perceivedObjects[i], sp.getOriginalRGBImage(), object_matcher_);
      la.execute();
    }
    else
    {
      // Set an empty label
      perceivedObjects[i].set_c_recognition_label_2d("");
    }

    // Publish the ROI-cropped images
    if(!fallback_enabled)
    {
      suturo_perception_2d_capabilities::ROIPublisher 
        rp(perceivedObjects.at(i), ph,sp.getOriginalRGBImage(),frameId);
      std::stringstream ss;
      ss << i;
      rp.setTopicName(CROPPED_IMAGE_PREFIX_TOPIC + ss.str());
      rp.execute();
    }
  }
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();

  res.perceivedObjs = *convertPerceivedObjects(&perceivedObjects); // TODO handle images in this method

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_publish = sp.getPlaneCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_publish = sp.getObjectsOnPlaneCloud();

	ph.publish_pointcloud(TABLE_PLANE_TOPIC,plane_cloud_publish, frameId);
	ph.publish_pointcloud(ALL_OBJECTS_ON_PLANE_TOPIC,object_cloud_publish, frameId);
  logger.logInfo((boost::format(" Extracted images vector: %s vs. Extracted PointCloud Vector: %s") % perceived_cluster_images.size() % perceivedObjects.size()).str());

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
  for (int i = 0; i < perceivedObjects.size(); i++)
  {
    if (i <= 6 && perceivedObjects.at(i).get_c_hue_histogram_image() != NULL)
    {
      std::stringstream ss;
      ss << i;
      ph.publish_cv_mat(HISTOGRAM_PREFIX_TOPIC + ss.str(), *(perceivedObjects.at(i).get_c_hue_histogram_image()), frameId);
    }
  }

  mutex.unlock();

  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  logger.logTime(start, end, "TOTAL");

  logger.logInfo("Shutting down subscriber");
  image_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  pc_sub.unsubscribe(); // shutdown subscriber, to mitigate funky behavior
  // sync.shutdown();
  visualizationPublisher.publishMarkers(res.perceivedObjs);

  logger.logInfo("Service call finished. return");
  return true;
}

/*
 * Callback for the dynamic reconfigure service
 */
void SuturoPerceptionROSNode::reconfigureCallback(suturo_perception_rosnode::SuturoPerceptionConfig &config, uint32_t level)
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
std::vector<suturo_perception_msgs::PerceivedObject> *SuturoPerceptionROSNode::convertPerceivedObjects(std::vector<suturo_perception_lib::PerceivedObject> *objects)
{
  std::vector<suturo_perception_msgs::PerceivedObject> *result = new std::vector<suturo_perception_msgs::PerceivedObject>();
  for (std::vector<suturo_perception_lib::PerceivedObject>::iterator it = objects->begin(); it != objects->end(); ++it)
  {
    suturo_perception_msgs::PerceivedObject *msgObj = new suturo_perception_msgs::PerceivedObject();
    msgObj->c_id = it->get_c_id();
    msgObj->c_shape = it->get_c_shape();
    msgObj->c_volume = it->get_c_volume();
    msgObj->c_centroid.x = it->get_c_centroid().x;
    msgObj->c_centroid.y = it->get_c_centroid().y;
    msgObj->c_centroid.z = it->get_c_centroid().z;
    msgObj->frame_id = frameId;
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
    for (int i = 0; i < 308; i++) 
    {
      msgObj->c_vfh_estimation.push_back(it->get_c_vfhs().histogram[i]);
    }
    msgObj->c_svm_result = svm_classification.classifyVFHSignature308(it->get_c_vfhs());
    msgObj->c_pose = svm_classification.classifyPoseVFHSignature308(it->get_c_vfhs(), msgObj->c_svm_result);

    // these are not set for now
    msgObj->recognition_label_3d = "";
    
    result->push_back(*msgObj);
  }
  return result;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
