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

  EXPECT_CMP(vg.size(), ==, 42);

  // TODO: test that all expected edges are present

  return expect_failed;
}
