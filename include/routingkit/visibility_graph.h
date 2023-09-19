#ifndef VISIBILITY_GRAPH
#define VISIBILITY_GRAPH

#include <routingkit/dijkstra.h>
#include <routingkit/geo_dist.h>
#include <routingkit/graph_util.h>
#include <routingkit/permutation.h>
#include <routingkit/segments_intersect.h>
#include <routingkit/inverse_vector.h>
#include <vector>
#include <iostream>

namespace RoutingKit {


// TODO:
// The polygons and the graph share the coordinates. However, after
// sorting the graph for routing, the graph ids // differ from the ids
// used for lat and lon, which still correspond
// to the order in the polygons. We need to decide how to deal with this
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

      first_vertex = std::vector<unsigned>(num_nodes + 1);
      latitudes = std::vector<float>(num_nodes); // TODO: need 2 more ?
      longitudes = std::vector<float>(num_nodes);// TODO: need 2 more ?
      unsigned vertex_id = 0;
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
    // TODO: refactor this partially overlaps with visible_vertices
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
                weights.push_back(0.5 + geo_dist(latitudes[v_id],longitudes[v_id],latitudes[w_id],longitudes[w_id]));
              }
            }
          }
        }
      } 
    }

    // Add links from visible nodes to target node
    void add_target(float lat, float lon) {
      for (unsigned vertex: visible_vertices(lat, lon)) {
        tails.push_back(vertex);
        heads.push_back(num_nodes);
        weights.push_back(0.5 + geo_dist(lat,lon,latitudes[vertex],longitudes[vertex]));
      }
      latitudes.push_back(lat);
      longitudes.push_back(lon);
      num_nodes++;
    }

    int size() {
        return heads.size();
    }

    void sort_graph_for_routing() {
        // TODO: Check whether this is the right permutation to be used
        this->permutation = compute_inverse_sort_permutation_first_by_tail_then_by_head_and_apply_sort_to_tail(this->num_nodes,this->tails, this->heads);
        this->heads = apply_inverse_permutation(permutation, std::move(this->heads));
        this->weights = apply_inverse_permutation(permutation, std::move(this->weights));
        this->first_out = invert_vector(this->tails, num_nodes);
    }


    unsigned target() {
        assert(permutation[num_nodes - 1] == num_nodes - 1);
        return num_nodes - 1; 
    }

    std::vector<unsigned> visible_vertices(float lat, float lon) {
       std::vector<unsigned> ret;
        for (unsigned p = 0; p < first_vertex.size(); p++) {
            for (unsigned v = first_vertex[p]; v<first_vertex[p+1]; v++) {
                bool invisible = false;
                for (unsigned q = 0; q < first_vertex.size(); q++) {
                    for (unsigned i = first_vertex[q], j=first_vertex[q+1] - 1; i<first_vertex[q+1]; j=i++) {
                        invisible = segments_intersect(lat, lon, latitudes[v],longitudes[v],
                            latitudes[i],longitudes[i],latitudes[j],longitudes[j]);
                        if (invisible) goto skip_further_obstacles;
                    }
                }
            skip_further_obstacles:
                if (!invisible) {
                   ret.push_back(v);
                }
            } 
        }
        return ret;
    }

    //
    void set_source(float lat, float lon) {
        // Cleanup old source
        for (unsigned i = 0; i < first_out[num_nodes+1] - first_out[num_nodes]; i++) {
            tails.pop_back();
            heads.pop_back();
            weights.pop_back();  
        } 
        // Add new source
        std::vector<unsigned> visibles = visible_vertices(lat, lon);
        for (unsigned vertex: visibles) {
            tails.push_back(num_nodes);
            heads.push_back(vertex); // TODO: Need to apply permutation here?
            weights.push_back(0.5 + geo_dist(lat,lon,latitudes[vertex],longitudes[vertex]));
        }
        first_out[num_nodes + 1] = first_out[num_nodes] + visibles.size();
        // TODO: need to append coordinates? If yes, also clean up
    }

    Dijkstra get_router() {
        return Dijkstra(first_out, this->tails, this->heads);
    }

    std::vector<unsigned> weights; // TODO: encapsulation!
  private:
    std::vector<std::vector<float>> polygons;
    std::vector<unsigned>first_vertex;
    std::vector<float> latitudes;
    std::vector<float> longitudes;
    std::vector<unsigned> permutation;

    unsigned num_nodes = 0;
    std::vector<unsigned> first_out;
    std::vector<unsigned> heads;
    std::vector<unsigned> tails;
};
} // RoutingKit
#endif