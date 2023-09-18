#ifndef POLYGON_IO_H
#define POLYGON_IO_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

namespace RoutingKit{

std::vector<std::vector<float>>load_polygons(const std::string&file_name){
	std::vector<std::vector<float>>ret;

    std::ifstream in(file_name);
    if(!in)
        throw std::runtime_error("Can not open \""+file_name+"\"");

    std::string line;
    unsigned line_num = 0;

    while(std::getline(in, line)){
        ++line_num;
        if(line.empty())
            continue;

        std::istringstream lin(line);

        std::vector<float> poly;

        // Read polygon
        while (!lin.eof()) {
            float latitude, longitude;
            if(!(lin >> latitude >> longitude))
                throw std::runtime_error("Cannot parse line number "+std::to_string(line_num)+" \""+line+"\" in polygon file.");
            poly.push_back(latitude);
            poly.push_back(longitude);
        }
        poly.shrink_to_fit();
        ret.push_back(poly);
    }

	ret.shrink_to_fit();
	return ret;
}

} // RoutingKit


#endif
