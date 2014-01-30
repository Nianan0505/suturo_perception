#ifndef CUBOID_H
#define CUBOID_H

// This rectangle will be defined by its three edge lengths
class Cuboid
{
  public:
    float length1;
    float length2;
    float length3;
    Eigen::Vector3f center;
    float volume;
};
#endif
