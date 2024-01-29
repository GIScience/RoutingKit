#include <routingkit/vector_io.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geojson.h>

#include "verify.h"

#include <iostream>
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
		string avoid_nodes_file;
		string avoid_edges_file;
		string settled_nodes_file;


		if(argc != 9){
			cerr << argv[0] << " first_out_file head_file weight_file lat_file lon_file avoid_nodes_file avoid_edges_file settled_nodes_file" << endl;
			return 1;
		}else{
			first_out_file = argv[1];
			head_file = argv[2];
			weight_file = argv[3];
			lat_file = argv[4];
			lon_file = argv[5];
			avoid_nodes_file = argv[6];
			avoid_edges_file = argv[7];
			settled_nodes_file = argv[8];
        }

		cerr << "Loading graph ... " << flush;

		vector<unsigned>first_out = load_vector<unsigned>(first_out_file);
		vector<unsigned>head = load_vector<unsigned>(head_file);
		vector<unsigned>weight = load_vector<unsigned>(weight_file);
		vector<float>latitude = load_vector<float>(lat_file);
		vector<float>longitude = load_vector<float>(lon_file);
	    BitVector avoid_nodes = load_bit_vector(avoid_nodes_file);
	    BitVector avoid_edges = load_bit_vector(avoid_edges_file);
	    BitVector settled_nodes = load_bit_vector(settled_nodes_file);
	
		cerr << "done" << endl;

		cerr << "Validity tests ... " << flush;
		check_if_graph_is_valid(first_out, head);
		cerr << "done" << endl;

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

		cerr << "done" << endl;

        cout << "{\"type\":\"FeatureCollection\",\"features\":[";
        for (unsigned n = 0; n < first_out.size() - 1; n++) {
            geojson_point(longitude[n], latitude[n], n, avoid_nodes.is_set(n), settled_nodes.is_set(n));
            cout << "," << endl;
        }
        for (unsigned a = 0; a < tail.size(); a++) {
            if (a > 0)
                cout << "," << endl;
            vector<float> arc = { longitude[tail[a]], latitude[tail[a]], 
                                   longitude[head[a]], latitude[head[a]] };
            geojson_linestring(arc, a, weight[a], avoid_edges.is_set(a));
        }
        cout << "]}" << endl;


	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}

