#ifndef VISIBILITY_GRAPH
#define VISIBILITY_GRAPH

#include <routingkit/segments_intersect.h>
#include <routingkit/geo_dist.h>
#include <vector>
#include <iostream>

namespace RoutingKit {

class VisibilityGraph {
  public:
    VisibilityGraph(std::vector<std::vector<float>> polygons):
      polygons(polygons){

      // TODO: remove polygon representation in favor of the following
      //       first_vertex/lats/longs representation
      num_nodes = 0;
      for (auto p: polygons) {
        num_nodes += p.size();
      }

      first_vertex = std::vector<unsigned>(num_nodes);
      latitudes = std::vector<float>(num_nodes); // TODO: need 2 more ?
      longitudes = std::vector<float>(num_nodes);// TODO: need 2 more ?
      unsigned vertex_id;
      for (unsigned poly_id = 0; poly_id < polygons.size(); poly_id++) {
        first_vertex[poly_id] = vertex_id;
        for (unsigned i = 0; i <polygons[poly_id].size(); i+=2) {
          latitudes[vertex_id] = polygons[poly_id][i];
          longitudes[vertex_id] = polygons[poly_id][i+1];
          vertex_id++;
        }
      }
      first_vertex[polygons.size()]=vertex_id;
    }
  
    // TODO: this is a very naive implementation of the naive O(n³) algo
    //       and has lots of potential for performance improvements. For
    //       now, performance is secondary, because the graph is only build
    //       once for many queries.
    void visibility_naive() {
      unsigned v_id = 0;
      // For each pair of vertices v, w ...
      for (auto p: polygons) {
        for (std::size_t v = 0; v < p.size()/2; v++, v_id++) {
          unsigned w_id = 0;
          for (auto q: polygons) {
            for (std::size_t w = 0; w < q.size()/2; w++, w_id++) {
              if (v_id == w_id) continue; // Nothing to check
              bool invisible = false;
              std::cout << "Segment: " << v_id << " " << w_id << std::endl;
              // ... check intersection of segment vw with polygon edge ij
              for (auto poly:polygons) {
                for (unsigned i = 0, j = poly.size()/2 - 1; i < poly.size()/2; j=i++) {
                  invisible = segments_intersect(p[2*v],p[2*v+1],q[2*w],q[2*w+1],
                    poly[i*2],poly[i*2+1],poly[j*2],poly[j*2+1]);
                  if (invisible) goto skip_further_obstacles;
                  if ((p == poly) && (q == poly)) {
                    // vw might be completely in or out of the polygon
                    // compare signs of determinants of v+1 - v, v-1 - v, w - v
                    float a_x = poly[(v+1)*2] - poly[v*2];
                    float a_y = poly[(v+1)*2+1] - poly[v*2+1];
                    float b_x = poly[(((v+poly.size()/2-1))%(poly.size()/2))*2] - poly[v*2];
                    float b_y = poly[(((v+poly.size()/2-1))%(poly.size()/2))*2+1] - poly[v*2+1];
                    float c_x = poly[w*2] - poly[v*2];
                    float c_y = poly[w*2+1] - poly[v*2+1];
                    float ab = a_x*b_y-b_x*a_y;
                    float ac = a_x*c_y-c_x*a_y;
                    float cb = c_x*b_y-b_x*c_y;
                    // TODO: remove debugging output once FP-arithmetic issues are solved
                    if ((ab < 0  && ac <  0 && cb < 0) || ((ab >= 0) && (ac < 0 || cb < 0))) {
                      std::cout << "  rejected: " << ab << " " << ac << " " << cb << std::endl;
                      invisible = true;
                      goto skip_further_obstacles; 
                    }
                                        
                  } 
                }
              }
           skip_further_obstacles:  
              if (!invisible) {
                // TODO: remove debugging output once FP-arithmetic issues are solved
                std::cout << "  appending (" << v_id << "," << w_id << ")" << std::endl;
                tails.push_back(v_id);
                heads.push_back(w_id);
                weights.push_back(geo_dist(latitudes[v_id],longitudes[v_id],latitudes[w_id],longitudes[w_id]));
              }
            }
          }
        }
      } 
    }

    // TODO: refactor: this partially overlaps with visibility_naive()
    void add_location(float lat, float lon) {
      unsigned v_id = 0;
      for (auto p: polygons) {
        for (std::size_t v = 0; v < p.size()/2; v++) {
          bool invisible = false;
          for (auto poly:polygons) {
            for (unsigned i = 0, j = poly.size()/2 - 1; i < poly.size()/2; j=i++) {
               invisible = segments_intersect(p[2*v],p[2*v+1],lat,lon,
                 poly[i*2],poly[i*2+1],poly[j*2],poly[j*2+1]);
               if (invisible) goto skip_further_obstacles;
            }
          }
        skip_further_obstacles:
          if (!invisible) {
            tails.push_back(v_id);
            heads.push_back(num_nodes); 
            weights.push_back(geo_dist(lat,lon,latitudes[v_id],longitudes[v_id]));
          }
          v_id++;
        }
      }
      latitudes.push_back(lat);
      longitudes.push_back(lon);
      num_nodes++;
    }

    int size() {
        return heads.size();
    }

  private:
    std::vector<std::vector<float>> polygons;
    std::vector<unsigned>first_vertex;
    std::vector<float> latitudes;
    std::vector<float> longitudes;

    unsigned num_nodes = 0;
    std::vector<unsigned> heads;
    std::vector<unsigned> tails;
    std::vector<float> weights;
};
}
#endif
