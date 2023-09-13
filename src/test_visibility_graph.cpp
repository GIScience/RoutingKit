#include <routingkit/visibility_graph.h>
#include "expect.h"

#include <iostream>

using namespace RoutingKit;
using namespace std;

int main(){
  vector<vector<float>> polys = {
    { 0.0, 0.0, 0.5, 1.0, 3.2, -0.9, -0.1, -2.0, -1.0, 0.2, -0.5, 0.8 },
    { 0.2, 1.2, 0.8, 2.0, 0.0, 2.5, -0.25, 1.8 }
  };
  
  VisibilityGraph vg = VisibilityGraph(polys);
  vg.visibility_naive();

	EXPECT(vg.size()==21);
	return expect_failed;
}
