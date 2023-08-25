#ifndef EDGE_CROSSES_POLYGON
#define EDGE_CROSSES_POLYGON

#include <vector>
#include <iostream>
#include <assert.h>

using namespace std;

namespace RoutingKit{

bool edge_crosses_polygon(float x_a, float y_a, float x_b, float y_b, std::vector<float> &poly)
{
  assert(poly.size() % 2 == 0);     // even number of coordinates expected
  
  float m1 = (y_b - y_a) / (x_b - x_a);
  float c1 = y_a - m1 * x_a;

  std::size_t num_edges = poly.size()/2; 
  std::size_t curr = 0;             // current vertex id
  std::size_t prev = num_edges - 1; // previous vertex id

  while (curr < num_edges){
    float x_d = poly[curr * 2];
    float y_d = poly[curr * 2 + 1];
    float x_c = poly[prev * 2];
    float y_c = poly[prev * 2 + 1];
    prev = curr++;

    float m2 = (y_d - y_c) / (x_d - x_c);
    float c2 = y_c - m2 * x_c;
    float x, y;

    // do not consider parallel horizontal or vertical edges 
    if ( (x_a==x_b && x_c==x_d) || (y_a==y_b && y_c==y_d) )
      continue;

    if (x_a==x_b) {
      x = x_a;
      y = m2 * x + c2;
    } else if (x_c==x_d) {
      x = x_c;
      y = m1 * x + c1;
    } else if (y_a==y_b) {
      y = y_a;
      x = (y - y_c) * (x_d - x_c) / (y_d - y_c) + x_c;
    } else if (y_c==y_d) {
      y = y_c;
      x = (y - y_a) * (x_b - x_a) / (y_b - y_a) + x_a;
    } else {
      x = (c2 - c1) / (m1 - m2);
      y = m1 * x + c1;
    }

    if ( ((x_a > x)!=(x_b > x) || x_a==x_b)
      && ((x_c > x)!=(x_d > x) || x_c==x_d)
      && ((y_a > y)!=(y_b > y) || y_a==y_b)
      && ((y_c > y)!=(y_d > y) || y_c==y_d) )
      return true;
  }
  return false;
}

}
#endif
