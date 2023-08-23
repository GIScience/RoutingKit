#ifndef POINT_IN_POLYGON
#define POINT_IN_POLYGON

#include <vector>
#include <iostream>
#include <assert.h>

namespace RoutingKit{

// This function implements a point in polygon test by ray-casting
// to the right. The main idea of the algorithm is to cast a 
// horizontal ray, i.e., a semi-infinite straight line, from the
// query point. Following the ray backwards from infinity, it starts
// outside of the polygon. Each time it crosses a polygon edge, it 
// switches between inside and outside until it reaches the query
// point. Hence, we can reduce this problem to identifying the
// edge crossings that lie right of the query point. The
// implementation is derived as follows:
// 
// Given: 
//   a query point q = (q_x,q_y),
//   a polygon edge e given by two points (x_1,y_1) and j = (x_2,y_2).
//
// Question:
//   Does an infinite ray r cast from q to the right cross e?
//
// Derivation:
//   Let g be the straight line defined by i,j, and let h be the horizontal
//   straight line through q. We have
//   (1) g(x) = mx + c, where m = (y_2-y_1)/(x_2-x_1) and c = y_1 - mx_1
//   (2) h(x) = q_y
//   We consider the crossing point (x,y) where g(x) = h(x).
//
//   Case 1: m != 0
//         g(x) = h(x)
//     <=> mx + y_1 - mx_1 = q_y  (by (1), (2))
//     <=> mx = q_y - y_1 + mx_1  (by equational reasoning)
//     <=>  x = (q_y-y_1)/m + x_1 (by case 1 and equational reasoning)
//
//     Hence, the ray r only crosses g if q_x < x. In order to cross
//     e, q_y has to be between y_1 and y_2.
//
//   Case 2: m = 0
//     g and h are parallel, hence they share a point if g=h, i.e., if
//     q_y = y_1 = y_2. In this case there are infinitely many shared
//     points, none of which contributes to a switch between inside and
//     outside. 

bool point_in_polygon(float query_x, float query_y, std::vector<float> &poly)
{
  assert(poly.size() % 2 == 0);     // even number of coordinates expected
  bool inside = false;
  std::size_t num_edges = poly.size()/2; 
  std::size_t curr = 0;             // current vertex id
  std::size_t prev = num_edges - 1; // previous vertex id

  while (curr < num_edges){
    float curr_x = poly[curr * 2];
    float curr_y = poly[curr * 2 + 1];
    float prev_x = poly[prev * 2];
    float prev_y = poly[prev * 2 + 1];
    if (((curr_y>query_y)!=(prev_y>query_y)) &&
     (query_x < (query_y-curr_y) * (prev_x-curr_x) / (prev_y-curr_y) + curr_x) ) {
       inside = !inside;
    }
    prev = curr++;
  }
  return inside;
}

}
#endif
