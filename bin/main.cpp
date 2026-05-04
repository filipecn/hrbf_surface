#include <hrbf_surf/io.h>
#include <hrbf_surf/point_cloud.h>
#include <hrbf_surf/pou_surface.h>

int main() {
  Eigen::setNbThreads(1);

  HERMES_INFO("Reading Point Cloud");
  auto pcl = hrbf_surf::io::loadPCLFromPly(
      "C:\\dev\\hrbf_surface\\assets\\pcd_culled.ply");

  HERMES_INFO("Computing HRBF Surface");
  auto ps = hrbf_surf::PoUSurface::from(*pcl, 0.1, 0.2);

  HERMES_INFO("Approximating Surface");
  hrbf_surf::io::writeSurface("surf.obj", *ps, pcl->computeBounds(), 0.01);

  return 0;
}