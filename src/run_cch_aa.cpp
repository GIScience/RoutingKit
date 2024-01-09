#include <routingkit/inverse_vector.h>
#include <climits>
#include <routingkit/vector_io.h>
#include <routingkit/bit_vector.h>
#include <routingkit/permutation.h>
#include <routingkit/customizable_contraction_hierarchy.h>
#include <routingkit/min_max.h>
#include <routingkit/timer.h>

#include <iostream>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){

	try{
		string first_out_file;
		string head_file;
		string node_order_file;
		string weight_file;
		string source_file;
		string target_file;
		string avoid_edges_file;
		string distance_file;

		if(argc != 9){
			cerr << argv[0] << " first_out_file head_file node_order_file weight_file source_file target_file avoid_edges distance_file" << endl;
			return 1;
		}else{
			first_out_file = argv[1];
			head_file = argv[2];
			node_order_file = argv[3];
			weight_file = argv[4];
			source_file = argv[5];
			target_file = argv[6];
			avoid_edges_file = argv[7];
			distance_file = argv[8];
		}

		cout << "Loading graph ... " << flush;

		std::vector<unsigned> first_out = load_vector<unsigned>(first_out_file);
		std::vector<unsigned> head = load_vector<unsigned>(head_file);
		std::vector<unsigned> tail = invert_inverse_vector(first_out);

		std::vector<unsigned> weight = load_vector<unsigned>(weight_file);
		std::vector<unsigned> node_order = load_vector<unsigned>(node_order_file);
		cout << "done" << endl;

		cout << "Building CCH ... " << flush;

		// TODO option: speed up by using the addditional params
		// https://github.com/RoutingKit/RoutingKit/blob/master/doc/CustomizableContractionHierarchy.md
		CustomizableContractionHierarchy cch(node_order, tail, head); 

		cout << "done" << endl;


		cout << "Loading avoidables ... " << flush;

		BitVector avoid_edges = load_bit_vector(avoid_edges_file);
		long long avoid_prep_time = -get_micro_time();
		unsigned num_edges = head.size();
		for (unsigned edge_id = 0; edge_id < num_edges; edge_id++) {
			if (avoid_edges.is_set(edge_id)) {
				weight[edge_id] = UINT_MAX;
			}
		}
		avoid_prep_time += get_micro_time();

		cout << "done" << endl;


		cout << "Customizing CCH ... " << flush;

		long long customization_time = -get_micro_time();
		CustomizableContractionHierarchyMetric metric(cch, weight);
		metric.customize();
		customization_time += get_micro_time();

		cout << "done" << endl;

		cout << "Loading test queries ... " << flush;

		vector<unsigned>source = load_vector<unsigned>(source_file);
		vector<unsigned>target = load_vector<unsigned>(target_file);

		cout << "done" << endl;

		const unsigned query_count = source.size();

		cout << "Loaded " << query_count << " test queries" << endl;

		vector<unsigned>distance(query_count);
		CustomizableContractionHierarchyQuery query(metric);

		cout << "Running test queries ... " << flush;

		long long time_max = 0;
		long long time_sum = 0;

		for(unsigned i=0; i<query_count; ++i){

			long long time = -get_micro_time();
			distance[i] = query.reset().add_source(source[i]).add_target(target[i]).run().get_distance();
			time += get_micro_time();

			time_max = std::max(time_max, time);
			time_sum += time;
		}

		cout << "done" << endl;

		cout << "avoid prep time     : " << avoid_prep_time << "musec" << endl;
		cout << "custiomization time : " << customization_time << "musec" << endl;
		cout << "max running time    : " << time_max << "musec" << endl;
		cout << "avg running time    : " << time_sum/query_count << "musec" << endl;

		save_vector(distance_file, distance);

	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}

