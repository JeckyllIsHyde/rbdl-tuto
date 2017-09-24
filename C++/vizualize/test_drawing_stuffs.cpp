#include <iostream>
#include <cmath>

#include "drawing_stuffs.h"

int main(void) {

  double q[2] = {M_PI/6,M_PI/4},
    L[2] = {0.5,0.3},
    p[2] = {0.2,0.0};

  std::vector<double> q2 = {M_PI/4,-M_PI/2,M_PI/4},
    L2 = {0.5,0.3,0.2};
  double p2[2] = {0.2,0.0};

  gp_draw_init("[-0.1:1.1]","[-0.1:1.1]");
  gp_draw_init_frame("Planar Robot 2R");
  gp_draw_robot2R( q, L, p );
  gp_draw_robotNR( q2, L2, p2 );
  gp_draw_end();
  
  return 0;
}
