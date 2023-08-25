#include <routingkit/vector_io.h>
#include <routingkit/bit_vector.h>
#include <routingkit/point_in_polygon.h>
#include <routingkit/edge_crosses_polygon.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/min_max.h>

#include <iostream>
#include <fstream>
#include <sstream>
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
		string avoid_file;

		if(argc != 7){
			cerr << argv[0] << " first_out head latitudes longitudes polygons avoid_file" << endl;
			return 1;
		}else{
			first_out_file = argv[1];
			head_file = argv[2];
			lat_file = argv[3];
			lon_file = argv[4];
			polygons_file = argv[5];
			avoid_file = argv[6];
		}

		cout << "Loading data ... " << flush;

		vector<unsigned>first_out = load_vector<unsigned>(first_out_file);
		vector<unsigned>head = load_vector<unsigned>(head_file);
		vector<float> lat = load_vector<float>(lat_file);
		vector<float> lon = load_vector<float>(lon_file);
		
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

		ifstream in(polygons_file);
		if(!in)
			throw runtime_error("Can not open \""+polygons_file+"\"");

		cout << "Mapping nodes ... " << flush;

		string line;
        unsigned line_num = 0;

		while(std::getline(in, line)){
			++line_num;
			if(line.empty())
				continue;

			std::istringstream lin(line);
			
			vector<float> poly;

			// Read polygon
			while (!lin.eof()) {
				float latitude, longitude;
				if(!(lin >> latitude >> longitude))
					throw std::runtime_error("Can not parse line num "+std::to_string(line_num)+" \""+line+"\" in polygon file.");
				poly.push_back(latitude);
				poly.push_back(longitude);
			}

			// Check point in polygon
			for (size_t i = 0; i < node_count; i++) {
				if (avoid_nodes.is_set(i)) continue;
				avoid_nodes.set_if(i, point_in_polygon(lat[i], lon[i], poly));
			}
		}
		cout << "done" << endl << "Mapping edges ... " << flush;

		in.clear();
		in.seekg(0);
		line_num = 0;

		while(std::getline(in, line)){
			++line_num;
			if(line.empty())
				continue;

			std::istringstream lin(line);
			
			vector<float> poly;

			// Read polygon
			while (!lin.eof()) {
				float latitude, longitude;
				if(!(lin >> latitude >> longitude))
					throw std::runtime_error("Can not parse line num "+std::to_string(line_num)+" \""+line+"\" in polygon file.");
				poly.push_back(latitude);
				poly.push_back(longitude);
			}

			// Check edge in polygon
			for (size_t i = 0; i < arc_count; i++) {
				unsigned tail_id = tail[i];
				unsigned head_id = head[i];
				if (avoid_edges.is_set(i)) continue;
				avoid_edges.set_if(i, avoid_nodes.is_set(tail_id) || avoid_nodes.is_set(head_id) 
					|| edge_crosses_polygon(lat[tail_id], lon[tail_id], lat[head_id], lon[head_id], poly));
			}
		}
		cout << "done" << endl;

		cout << "Saving file ... " << flush;
		save_bit_vector(avoid_file, avoid_edges);
		cout << "done" << endl;

	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}
