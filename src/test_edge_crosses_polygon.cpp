#include <routingkit/edge_crosses_polygon.h>
#include "expect.h"

using namespace RoutingKit;
using namespace std;

int main(){
	vector<float> poly = { 0.0, 0.0, 1.0, 1.0, 2.0, 1.0, 2.0, 0.5, 1.1, 0.0, 1.8, -0.9, 0.8, 0.2, 0.0, -1.0 };

	EXPECT(edge_crosses_polygon(0.0, 0.5, 1.0, 0.5, poly));
	EXPECT(!edge_crosses_polygon(0.5, -0.5, 1.0, -0.5, poly));
	EXPECT(edge_crosses_polygon(1.5, 0.5, 1.5, 1.5, poly));
	EXPECT(!edge_crosses_polygon(0.5, -0.5, 0.5, -1.0, poly));
	EXPECT(edge_crosses_polygon(1.0, -1.0, 2.0, 0.0, poly));
	EXPECT(!edge_crosses_polygon(1.0, 0.6, 1.4, 0.2, poly));
	EXPECT(edge_crosses_polygon(-0.5, -0.5, 0.5, -0.5, poly));
	return expect_failed;
}
