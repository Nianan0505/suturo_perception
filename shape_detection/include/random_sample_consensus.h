#ifndef RANDOM_SAMPLE_CONSENSUS_H
#define RANDOM_SAMPLE_CONSENSUS_H
#include <pcl/point_types.h>


namespace suturo_perception_shape_detection
{
    enum Shape {None, Box, Cylinder, Sphere}; 
    class RandomSampleConsensus
    {
        public:
            RandomSampleConsensus();
            void detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);
            Shape getShape();
        private:
            Shape shape;
    };
}
#endif


