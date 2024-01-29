#ifndef GEOJSON_H
#define GEOJSON_H

#include <vector>
#include <iostream>

// TODO: this file is just a quick messy hack, cleanup!

void path_to_geojson(std::vector<unsigned> node_path,
    std::vector<float>latitude, std::vector<float>longitude) {
    std::cout << "{\"type\":\"LineString\",\"coordinates\":[";
    for (unsigned n = 0; n < node_path.size(); n++) {
        if (n > 0)
            std::cout << ",";
        std::cout << "[" << longitude[node_path[n]] << ","
             << latitude[node_path[n]] << "]";
    }
    std::cout << "]}" << std::endl;
}


void geojson_point(float longitude, float latitude, unsigned id, bool avoid, bool settled) {
    std::cout << "{\"type\":\"Feature\",\"geometry\":";
    std::cout << "{\"type\":\"Point\",\"coordinates\":["
        << longitude << "," << latitude << "]}";
    std::cout << ",\"properties\":{\"id\":" << id << ",\"avoid\":" << avoid 
        << ",\"settled\":" << settled << "}"; 
    std::cout << "}";
}

void geojson_linestring(std::vector<float> coordinates, unsigned id, unsigned weight, bool avoid) {
    std::cout << "{\"type\":\"Feature\",\"geometry\":";
    std::cout << "{\"type\":\"LineString\",\"coordinates\":[";
    for (unsigned c = 0; c < coordinates.size() ; c+=2) {
        if (c > 0)
            std::cout << ",";
        std::cout << "[" << coordinates[c] << "," << coordinates[c+1] << "]";
    }
    std::cout << "]}";
    std::cout << ",\"properties\":{\"id\":" << id << ",\"weight\":" << weight 
              << ",\"avoid\":" << avoid << "}"; 
    std::cout << "}";
}

#endif
