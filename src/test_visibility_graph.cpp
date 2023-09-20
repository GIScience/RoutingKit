#include <routingkit/visibility_graph.h>
#include "expect.h"

#include <iostream>

using namespace RoutingKit;
using namespace std;

int main(){
  vector<vector<float>> polys = {
    { 0.0, 0.0, 0.5, 1.0, 3.2, -0.9, -0.1, -2.0, -1.0, 0.2, -0.5, 0.8 },
    { 0.2, 1.2, -0.25, 1.8, 0.0, 2.5, 0.8, 2.0 }
  };
  
  VisibilityGraph vg = VisibilityGraph(polys);
  vg.visibility_naive();
  vg.sort_graph_for_routing();

  // Total number
  EXPECT_CMP(vg.arc_count(), ==, 42);

  // Polygon 0
  EXPECT(vg.has_arc(0,1));
  EXPECT(vg.has_arc(1,0));
  EXPECT(vg.has_arc(1,2));
  EXPECT(vg.has_arc(2,1));
  EXPECT(vg.has_arc(2,3));
  EXPECT(vg.has_arc(3,2));
  EXPECT(vg.has_arc(3,4));
  EXPECT(vg.has_arc(4,3));
  EXPECT(vg.has_arc(4,5));
  EXPECT(vg.has_arc(5,4));
  EXPECT(vg.has_arc(5,0));
  EXPECT(vg.has_arc(0,5));
  EXPECT(vg.has_arc(1,5));
  EXPECT(vg.has_arc(5,1));

  // Polygon 1
  EXPECT(vg.has_arc(6,7));
  EXPECT(vg.has_arc(7,6));
  EXPECT(vg.has_arc(7,8));
  EXPECT(vg.has_arc(8,7));
  EXPECT(vg.has_arc(8,9));
  EXPECT(vg.has_arc(9,8));
  EXPECT(vg.has_arc(9,6));
  EXPECT(vg.has_arc(6,9));
  
  // Interpolygonal edges
  EXPECT(vg.has_arc(0,6));
  EXPECT(vg.has_arc(6,0));
  EXPECT(vg.has_arc(0,7));
  EXPECT(vg.has_arc(7,0));
  EXPECT(vg.has_arc(0,9));
  EXPECT(vg.has_arc(9,0));
  EXPECT(vg.has_arc(6,1));
  EXPECT(vg.has_arc(1,6));
  EXPECT(vg.has_arc(9,1));
  EXPECT(vg.has_arc(1,9));
  EXPECT(vg.has_arc(9,2));
  EXPECT(vg.has_arc(2,9));
  EXPECT(vg.has_arc(4,7));
  EXPECT(vg.has_arc(7,4));
  EXPECT(vg.has_arc(4,8));
  EXPECT(vg.has_arc(8,4));
  EXPECT(vg.has_arc(5,6));
  EXPECT(vg.has_arc(6,5));
  EXPECT(vg.has_arc(5,7));
  EXPECT(vg.has_arc(7,5));

  return expect_failed;
}
