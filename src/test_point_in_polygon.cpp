#include <routingkit/point_in_polygon.h>
#include "expect.h"

using namespace RoutingKit;
using namespace std;

int main(){
	vector<float> poly = { 0.0, 0.0, 1.0, 1.0, 2.0, 0.5, 1.1, 0.0, 1.8, -0.9, 0.8, 0.2, 0.0, -1.0 };

	EXPECT(point_in_polygon(0.5, 0.1, poly));
	EXPECT(!point_in_polygon(0.0, 1.0, poly));
	EXPECT(point_in_polygon(0.8, 0.2, poly));
	EXPECT(!point_in_polygon(0.8, 0.1, poly));

	return expect_failed;
}
