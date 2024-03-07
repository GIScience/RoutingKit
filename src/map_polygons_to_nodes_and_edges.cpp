#include <routingkit/vector_io.h>
#include <routingkit/bit_vector.h>
#include <routingkit/point_in_polygon.h>
#include <routingkit/edge_crosses_polygon.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/min_max.h>
#include <routingkit/polygon_io.h>

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){

	try{
		string first_out_file;
		string head_file;
		string lat_file;
		string lon_file;
		string polygons_file;
		string weights_in_file;
		string weights_out_file;
		string avoid_nodes_file;
		string avoid_edges_file;

		if(argc != 10){
			cerr << argv[0] << " first_out head latitudes longitudes polygons weights_in weights_out avoid_nodes_file avoid_edges_file" << endl;
			return 1;
		} else {
			first_out_file = argv[1];
			head_file = argv[2];
			lat_file = argv[3];
			lon_file = argv[4];
			polygons_file = argv[5];
			weights_in_file = argv[6];
			weights_out_file = argv[7];
			avoid_nodes_file = argv[8];
			avoid_edges_file = argv[9];
		}

		cout << "Loading data ... " << flush;

		vector<unsigned>first_out = load_vector<unsigned>(first_out_file);
		vector<unsigned>head = load_vector<unsigned>(head_file);
		vector<float> lat = load_vector<float>(lat_file);
		vector<float> lon = load_vector<float>(lon_file);
		vector<unsigned> weights = load_vector<unsigned>(weights_in_file);

		cout << "done" << endl;

		if (lat.size() != lon.size()) {
			throw std::runtime_error("Latitudes and longitudes are of unequal length.");
		}

		auto tail = invert_inverse_vector(first_out);

		const unsigned node_count = first_out.size()-1;
		const unsigned arc_count = head.size();

		if(first_out.front() != 0)
			throw runtime_error("The first element of first out must be 0.");
		if(first_out.back() != arc_count)
			throw runtime_error("The last element of first out must be the arc count.");
		if(max_element_of(head) >= node_count)
			throw runtime_error("The head vector contains an out-of-bounds node id.");

		BitVector avoid_nodes(node_count);
		BitVector avoid_edges(arc_count);

		// TODO: do we need these three lines?
		ifstream in(polygons_file);
		if(!in)
			throw runtime_error("Can not open \""+polygons_file+"\"");

		cout << "Mapping nodes and edges ... " << flush;

		vector<vector<float>> polys = load_polygons(polygons_file);

		for (auto poly:polys) {
			// Check point in polygon
			for (size_t i = 0; i < node_count; i++) {
				if (avoid_nodes.is_set(i)) continue;
				avoid_nodes.set_if(i, point_in_polygon(lat[i], lon[i], poly));
			}
			// Check edge in polygon
			for (size_t i = 0; i < arc_count; i++) {
				unsigned tail_id = tail[i];
				unsigned head_id = head[i];
				if (avoid_edges.is_set(i)) continue;
				if (avoid_nodes.is_set(tail_id)
					|| avoid_nodes.is_set(head_id) 
					|| edge_crosses_polygon(lat[tail_id], lon[tail_id], lat[head_id],
				lon[head_id], poly)) 
				{
					avoid_edges.set(i);
					weights[i] = inf_weight;
				}
			}
		}
		cout << "done" << endl;

		cout << "Saving files ... " << flush;
		save_bit_vector(avoid_nodes_file, avoid_nodes);
		save_bit_vector(avoid_edges_file, avoid_edges);
		save_vector<unsigned>(weights_out_file, weights);
		cout << "done" << endl;

	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}
