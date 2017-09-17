#ifndef DRAWING_STUFFS_H_
#define DRAWING_STUFFS_H_

#include <iostream>
#include <string>

void gp_draw_link( double* p1, double* p2 );
void gp_draw_point( double* p );
void gp_draw_init( const char* xrange = "[-1.1:1.1]",
		   const char* yrange = "[-1.1:1.1]");
void gp_draw_set_gif_output( const char* filename );
void gp_draw_init_frame( const std::string& str );
void gp_draw_end_frame( void ) {  std::cout << std::endl; }
void gp_draw_end( void ) { std::cout << std::endl; }
void gp_draw_robot2R( double* q, double* L, double* p );

void gp_draw_link( double* p1, double* p2 ) {
  std::cout << ",(" << p2[0] << "-" << p1[0] <<")*t/7+" << p1[0]
	    << ",(" << p2[1] << "-" << p1[1] <<")*t/7+" << p1[1] << " lw 3 lc 'blue'";
  std::cout << ",0.01*cos(t) + " << p1[0] << ", 0.01*sin(t) + " << p1[1] << " lw 5 lc 'yellow'";
}

void gp_draw_point( double* p ) {
  std::cout << ",0.01*cos(t) + " << p[0] << ", 0.01*sin(t) + " << p[1] << " lw 5 lc 'red'";
}

void gp_draw_init( const char* xrange, const char* yrange) {
  std::cout << "unset key" << std::endl; 
  std::cout << "set xrange " << xrange << std::endl;
  std::cout << "set yrange " << yrange << std::endl;
  std::cout << "set size ratio -1" << std::endl;
  std::cout << "set parametric" << std::endl;
  std::cout << "set trange [0:7]" << std::endl;
  std::cout << "set isosamples 12" << std::endl;
}

void gp_draw_set_gif_output( const char* filename ) {
  std::cout << "set terminal gif animate" << std::endl; 
  std::cout << "set output '" << filename << ".gif'" << std::endl; 
}

void gp_draw_init_frame( const std::string& str ) {
  std::cout << "set style line 12 lc rgb '#808080' lt 0 lw 1" << std::endl;
  std::cout << "set grid back ls 12" << std::endl;
  std::cout << "set title '" << str << "'" << std::endl;
  std::cout << "plot 0,0 ";
}

void gp_draw_robot2R( double* q, double* L, double* p ) {
  double b[2] = {L[0]*cos(q[0])+p[0], L[0]*sin(q[0])+p[1]},
    c[2] = {L[1]*cos(q[0]+q[1])+b[0],L[1]*sin(q[0]+q[1])+b[1]};
  gp_draw_link(p,b);
  gp_draw_link(b,c);
  gp_draw_point(c);
}

#endif //DRAWING_STUFFS_H_
