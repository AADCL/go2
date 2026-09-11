// Offline probe: reuse production connectivity and height estimation.
// Input: point count followed by x/y/z triples; output: selected cells and fit.
#include <iostream>
#include <iomanip>
#include "go2_terrain/terrain_model.hpp"
int main() {
  namespace gt = go2_terrain;
  gt::GroundConnectivityParameters g;
  g.width=67; g.height=54; g.resolution=0.15;
  g.origin_x=-5.0; g.origin_y=-4.0;
  gt::GroundHealthParameters health;
  health.minimum_covered_sectors=2;
  int n;
  while (std::cin >> n) {
    std::vector<std::vector<float>> cells(g.width*g.height);
    for (int i=0;i<n;++i) {
      double x,y,z; std::cin >> x >> y >> z;
      if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
          x < -5 || x >= 5 || y < -4 || y >= 4 ||
          std::hypot(x,y)<0.35 || std::hypot(x,y)>5) continue;
      cells[int(std::floor((y+4)/0.15))*67+int(std::floor((x+5)/0.15))].push_back(z);
    }
    std::vector<float> heights(cells.size(),std::numeric_limits<float>::quiet_NaN());
    for(std::size_t i=0;i<cells.size();++i)
      if(!cells[i].empty()) heights[i]=gt::robustLowSupportHeight(cells[i],0.10,0.15);
    auto connected=gt::connectedGroundMask(heights,g);
    auto selected=gt::selectGroundComponent(connected,g,health);
    auto fit=gt::estimateConnectedGroundPlane(heights,selected.mask,g,gt::GroundPlaneFitParameters());
    std::cout << std::setprecision(9) << fit.sensor_height_m << ' ' << fit.a << ' ' << fit.b << ' ' << fit.rmse_m;
    for(std::size_t i=0;i<selected.mask.size();++i)
      if(selected.mask[i]) std::cout << ' ' << i;
    std::cout << std::endl;
  }
}
