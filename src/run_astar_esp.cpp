#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/min_max.h>
#include <routingkit/astar.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/polygon_io.h>
#include <routingkit/geojson.h>

#include "verify.h"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){

	try{
		string first_out_file;
		string head_file;
		string weight_file;
		string source_file;
		string target_file;
		string lat_file;
		string lon_file;
		string polygons_file;
		string avoid_file;
		string distance_file;
		string settlings_file;
        bool save_settlings;

		if(argc < 11 || argc > 12){
			cerr << argv[0] << " first_out_file head_file weight_file source_file target_file lat_file lon_file polygons_file avoid_file distance_file" << endl;
			cerr << argv[0] << " first_out_file head_file weight_file source_file target_file lat_file lon_file polygons_file avoid_file distance_file settlings_file" << endl;
			return 1;
		}else{
			first_out_file = argv[1];
			head_file = argv[2];
			weight_file = argv[3];
			source_file = argv[4];
			target_file = argv[5];
			lat_file = argv[6];
			lon_file = argv[7];
			polygons_file = argv[8];
			avoid_file = argv[9];
			distance_file = argv[10];
            if (argc == 12) {
                settlings_file = argv[11];
                save_settlings = true;
            } else {
                save_settlings = false;
            }
		}

		cout << "Loading graph ... " << flush;

		vector<unsigned>first_out = load_vector<unsigned>(first_out_file);
		vector<unsigned>head = load_vector<unsigned>(head_file);
		vector<unsigned>weight = load_vector<unsigned>(weight_file);
		vector<float>latitude = load_vector<float>(lat_file);
		vector<float>longitude = load_vector<float>(lon_file);
	    BitVector avoid_edges = load_bit_vector(avoid_file);
	
		cout << "done" << endl;

		cout << "Loading polygons ... " << flush;
        vector<vector<float>>polygons = load_polygons(polygons_file);
		cout << "done" << endl;

		cout << "Validity tests ... " << flush;
		check_if_graph_is_valid(first_out, head);
		cout << "done" << endl;

		auto tail = invert_inverse_vector(first_out);

		const unsigned node_count = first_out.size()-1;
		const unsigned arc_count = head.size();

		if(first_out.front() != 0)
			throw runtime_error("The first element of first out must be 0.");
		if(first_out.back() != arc_count)
			throw runtime_error("The last element of first out must be the arc count.");
		if(max_element_of(head) >= node_count)
			throw runtime_error("The head vector contains an out-of-bounds node id.");
		if(weight.size() != arc_count)
			throw runtime_error("The weight vector must be as long as the number of arcs");
        if(avoid_edges.size() != arc_count)
			throw runtime_error("The avoid vector must be as long as the number of arcs");

		Astar astar(first_out, tail, head);

		cout << "Loading test queries ... " << flush;

		vector<unsigned>source = load_vector<unsigned>(source_file);
		vector<unsigned>target = load_vector<unsigned>(target_file);

		cout << "done" << endl;

		if(source.size() != target.size())
			throw runtime_error("The source and target vectors must be of the same length.");
			

		unsigned query_count = source.size();

		cout << "Loaded " << query_count << " test queries" << endl;

		vector<unsigned>distance(query_count);
		vector<unsigned>settlings(query_count);

		cout << "Running test queries ... " << endl;

		long long time_max = 0;
		long long time_sum = 0;

        BitVector settled_nodes = BitVector(first_out.size() - 1);

		for(unsigned i=0; i<query_count; ++i){
			long long vg_time = get_micro_time();
			VisibilityGraph vg(polygons);
			vg.visibility_naive(); // TODO: optimization: we do not need to recompute the whole graph in each round

			vg.add_target_bw(latitude[target[i]], longitude[target[i]]); 
			vg.sort_graph_for_routing();
            
            vg_time = get_micro_time() - vg_time;
            vg.save_graph();

			auto heuristic = new EspHeuristic(latitude, longitude, target[i], vg);
            settled_nodes.reset_all();

			long long time = -get_micro_time();

			astar.reset().add_source(source[i]).set_avoid_edges(&avoid_edges);
			while(!astar.is_finished()){
				auto x = astar.settle(ScalarGetWeight(weight),*heuristic).node;
                settled_nodes.set(x); 
				if(x == target[i])
					break;
			}

			distance[i] = astar.get_distance_to(target[i]);
			settlings[i] = astar.get_settle_count();
			time += get_micro_time();

			time_max = std::max(time_max, time);
			time_sum += time;

            auto node_path = astar.get_node_path_to(target[i]);
            path_to_geojson(node_path, latitude, longitude);
            if (save_settlings) {
                save_bit_vector(settlings_file + std::to_string(i), settled_nodes);
            } 

            cout << "VG building time       : " << vg_time << "musec" << endl;
            cout << "source integration time: " << heuristic->time_set_source << "musec" << endl;
            cout << "esp solving time       : " << heuristic->time_solve_esp << "musec" << endl;
            cout << "integration/esp        : " << heuristic->time_set_source/heuristic->time_solve_esp << endl;
            cout << "running time           : " << time << "musec" << endl;
		}

		cout << "done" << endl;

		cout << "max running time : " << time_max << "musec" << endl;
		cout << "avg running time : " << time_sum/query_count << "musec" << endl;

		save_vector(distance_file, distance);
        if (save_settlings) {
            save_vector(settlings_file, settlings);
        }

	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}

