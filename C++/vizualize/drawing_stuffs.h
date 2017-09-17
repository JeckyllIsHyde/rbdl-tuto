#ifndef DRAWING_STUFFS_H_
#define DRAWING_STUFFS_H_

#include <iostream>
#include <string>

void gp_draw_link( double* p1, double* p2 ) {
  std::cout << ",(" << p2[0] << "-" << p1[0] <<")*t/7+" << p1[0]
	    << ",(" << p2[1] << "-" << p1[1] <<")*t/7+" << p1[1] << " lw 3 lc 'blue'";
  std::cout << ",0.01*cos(t) + " << p1[0] << ", 0.01*sin(t) + " << p1[1] << " lw 5 lc 'yellow'";
}

#endif //DRAWING_STUFFS_H_
