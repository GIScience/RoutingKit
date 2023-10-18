#include <routingkit/geo_position_to_node.h>
#include <routingkit/vector_io.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    try {
        string lat_file;
        string lon_file;
        float lat_query;
        float lon_query;
    
        if (argc != 5) {
            cerr << argv[0] << " latitude_file longitude_file query_lat query_lon" << endl;
            return 1;
        } else {
            lat_file = argv[1];
            lon_file = argv[2];
            istringstream slat(argv[3]);
            if (!(slat >> lat_query)) {
                throw runtime_error("Cannot parse query latitude from command line.");
            }
            istringstream slon(argv[4]);
            if (!(slon >> lon_query)) {
                throw runtime_error("Cannot parse query longitude from command line.");
            }
        }

        vector<float>latitude = load_vector<float>(lat_file);
        vector<float>longitude = load_vector<float>(lon_file);

        GeoPositionToNode mapmatcher(latitude, longitude);
        auto node = mapmatcher.find_nearest_neighbor_within_radius(lat_query, lon_query, 300);
        cout << "Nearest node is (query):" 
             << "\n  id:   " << node.id 
             << "\n  lat:  " << latitude[node.id] << " (" << lat_query << ")"
             << "\n  lon:  " << longitude[node.id] << " (" << lon_query << ")"
             << "\n  dist: " << node.distance << endl;
    } catch (exception&err) {
        cerr << "Stopped on exception : " << err.what() << endl;
    }
}
