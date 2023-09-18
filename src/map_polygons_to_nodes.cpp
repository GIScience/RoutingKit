#include <routingkit/vector_io.h>
#include <routingkit/bit_vector.h>
#include <routingkit/point_in_polygon.h>

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
		string lat_file;
		string lon_file;
		string polygons_file;
		string avoid_file;

		if(argc != 5){
			cerr << argv[0] << " latitudes longitudes polygons avoid_file" << endl;
			return 1;
		}else{
			lat_file = argv[1];
			lon_file = argv[2];
			polygons_file = argv[3];
			avoid_file = argv[4];
		}

		cout << "Loading data ... " << flush;

		vector<float> lat = load_vector<float>(lat_file);
		vector<float> lon = load_vector<float>(lon_file);

		if (lat.size() != lon.size()) {
			throw std::runtime_error("Latitudes and longitudes are of unequal length.");
		}

		BitVector avoid_nodes(lat.size());

		ifstream in(polygons_file);
		if(!in)
			throw runtime_error("Can not open \""+polygons_file+"\"");

		string line;
        unsigned line_num = 0;

        // TODO: refactoring: maybe reuse polygon_io
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
			for (size_t i = 0; i < lat.size(); i++) {
				if (avoid_nodes.is_set(i)) continue;
				avoid_nodes.set_if(i,point_in_polygon(lat[i],lon[i],poly));
			}
		}
		cout << "done" << endl;

		cout << "Saving file ... " << flush;
		save_bit_vector(avoid_file, avoid_nodes);
		cout << "done" << endl;

	}catch(exception&err){
		cerr << "Stopped on exception : " << err.what() << endl;
	}
}
