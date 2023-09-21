#include <routingkit/astar.h>
#include <routingkit/inverse_vector.h>

#include <vector>
#include <iostream>

#include "expect.h"
#include "verify.h"

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    // We consider the following graph with the rectangular avoid
    // area marked by X, routing from node 1 to node 8.
    //
    // 10 -- 9 -- 8 -- 7 -- 6
    //  |    XXXXXXXXXX|XXX |
    // 11    XXXXXXXX 12 XX 5
    //  |    XXXXXXXXXX|XXX |
    //  0 -- 1 -- 2 -- 3 -- 4

    vector<unsigned> first_out = { 0, 2, 4, 6, 9, 11, 13, 15, 18, 20, 22, 24, 26, 28};
    vector<unsigned> head = { 1, 11, 0, 2, 1, 3, 2, 4, 12, 3, 5, 4, 6, 5, 7, 6, 8, 12, 7, 9, 8, 10, 9, 11, 0, 10, 3, 7};
    // At equator, 0.001 degree is approximately 111m
    vector<unsigned> weight = {111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111};
    vector<float> latitude  = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.002, 0.002, 0.002, 0.002, 0.002, 0.001, 0.001 };
    vector<float> longitude = { 0.0, 0.001, 0.002, 0.003, 0.004, 0.004, 0.004, 0.003, 0.002, 0.001, 0.0, 0.0, 0.003};

    unsigned arc_count = head.size();

	try{
		cout << "Validity tests ... " << flush;
		check_if_graph_is_valid(first_out, head);
		cout << "done" << endl;

		vector<unsigned>tail = invert_inverse_vector(first_out);

		Astar astar(first_out, tail, head);
		EXPECT(astar.is_finished());

		cout << "Process avoid polygons ... " << flush;
        vector<vector<float>> polygons = {{0.0005,0.0005, 0.0005,0.0035, 0.0015,0.0035, 0.0015,0.0005 }};
        BitVector avoid_edges(arc_count);
        for (auto p: polygons) {
            for (unsigned i = 0, j=p.size()/2; i < p.size()/2; j=++i) {
                
                for (size_t a = 0; a < arc_count; a++) {
                    unsigned tail_id = tail[a];
                    unsigned head_id = head[a];
                    if (avoid_edges.is_set(a)) continue;
                    bool intersect = segments_intersect(
                        latitude[tail_id],longitude[tail_id], latitude[head_id],longitude[head_id],
                        p[2*i],p[2*i+1], p[2*j],p[2*j+1]);
                    if (intersect)
                        std::cout << tail_id << "->" << head_id << " intersects: " << intersect << std::endl;
                    avoid_edges.set_if(a, intersect);
                }
            }
        }
		cout << "done" << endl;

        cout << "Testing ..." << flush;
        unsigned source_node = 1;
        unsigned target_node = 8;

        VisibilityGraph vg(polygons); 
        vg.visibility_naive();
        EXPECT_CMP(vg.arc_count(), ==, 8);
        vg.add_target(latitude[target_node],longitude[target_node]);
        cout << __FILE__ << ":" << __LINE__ << " arc-count: " << vg.arc_count() << endl;
        vg.sort_graph_for_routing();

        astar.add_source(source_node, 0).set_avoid_edges(&avoid_edges);

        EspHeuristic heuristic(latitude, longitude, target_node, vg);
        
        auto ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 1);
        ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 0);
        ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 11);
        ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 10);
        ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 9);
        ret = astar.settle(ScalarGetWeight(weight),heuristic);
        EXPECT_CMP(ret.node, ==, 8);
		cout << "done" << endl;
	}catch(exception&err){
		cout << "Stopped on exception : "<< err.what() << endl;
		return 1;
	}
	return expect_failed;
}
