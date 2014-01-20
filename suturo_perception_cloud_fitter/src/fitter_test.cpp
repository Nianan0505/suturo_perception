#include "mesh_loader.h"

using namespace suturo_perception_cloud_fitter;

int test_ref(const std::string& filename, std::vector<double> &vertices, std::vector<int> &triangles)
{

}

int main(int argc, char const *argv[])
{
  PLYModelLoader plm = PLYModelLoader();
  std::vector<double> vertices;
  std::vector<int>    triangles;
  std::string plyFile("/tmp/pringles.ply");
  plm.readFromFile(plyFile, vertices, triangles);
  test_ref(plyFile, vertices, triangles);

  return 0;
}