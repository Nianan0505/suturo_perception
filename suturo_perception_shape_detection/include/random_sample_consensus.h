#ifndef RANDOM_SAMPLE_CONSENSUS_H
#define RANDOM_SAMPLE_CONSENSUS_H
#include <pcl/point_types.h>

#include "capability.h"
#include "suturo_perception_utils.h"

namespace suturo_perception_shape_detection
{
    enum Shape {None, Box, Cylinder, Sphere}; 
    class RandomSampleConsensus : public suturo_perception_lib::Capability
    {
        public:
            RandomSampleConsensus(suturo_perception_lib::PerceivedObject &obj);
            void detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);
            Shape getShape();

            // capability method
            void execute();
        private:
            Shape shape;
            suturo_perception_utils::Logger logger;
    };
}
#endif


