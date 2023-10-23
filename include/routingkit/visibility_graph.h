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
// sorting the graph for routing, the graph ids differ from the ids
// used for lat and lon, which still correspond to the order in the
// polygons. We need to decide how to deal with this.
class VisibilityGraph {
  public:
    VisibilityGraph(std::vector<std::vector<float>> polygons) {
      num_nodes = 0;
      for (auto p: polygons) {
        num_nodes += p.size() / 2;
      }

      first_vertex = std::vector<unsigned>(polygons.size() + 1);
      latitudes = std::vector<float>(num_nodes + 1);
      longitudes = std::vector<float>(num_nodes + 1);
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
      // For each pair of vertices v, w ...
      for (unsigned p_id = 0; p_id < first_vertex.size()-1; p_id++) {
        for (unsigned v = first_vertex[p_id]; v < first_vertex[p_id+1]; v++) {
          for (unsigned q_id = 0; q_id < first_vertex.size()-1; q_id++) {
            for (unsigned w = first_vertex[q_id]; w < first_vertex[q_id+1]; w++) {
              if (v == w) continue; // Nothing to check
              bool invisible = false;
              // ... check intersection of segment vw with polygon edge ij
              for (unsigned poly = 0; poly < first_vertex.size()-1; poly++) {
                for (unsigned i = first_vertex[poly], j = first_vertex[poly+1]-1; i < first_vertex[poly+1]; j=i++) {
                  float y1 = latitudes[v], x1 = longitudes[v];
                  float y2 = latitudes[w], x2 = longitudes[w];
                  float y3 = latitudes[i], x3 = longitudes[i];
                  float y4 = latitudes[j], x4 = longitudes[j];
                  // check for intersections only when all endpoints are different
                  if ((x1!=x3 || y1!=y3) && (x1!=x4 || y1!=y4) && (x2!=x3 || y2!=y3) && (x2!=x4 || y2!=y4)) {
                    invisible = segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4);
                    if (invisible) goto skip_further_obstacles;
                  } else if ((p_id == poly) && (q_id == poly) && (v+1 != w) && (v-1 != w)) {
                    // vw might be completely in or out of the polygon
                    // compare signs of determinants of v+1 - v, v-1 - v, w - v
                    unsigned pred_v = (v > first_vertex[poly])?v-1:first_vertex[poly+1]-1;
                    unsigned succ_v = (v < first_vertex[poly+1]-1)?v+1:first_vertex[poly];
                    float a_x = latitudes[succ_v] - latitudes[v];
                    float a_y = longitudes[succ_v] - longitudes[v];
                    float b_x = latitudes[pred_v] - latitudes[v];
                    float b_y = longitudes[pred_v] - longitudes[v];
                    float c_x = latitudes[w] - latitudes[v];
                    float c_y = longitudes[w] - longitudes[v];
                    float ab = a_x*b_y-b_x*a_y;
                    float ac = a_x*c_y-c_x*a_y;
                    float cb = c_x*b_y-b_x*c_y;
                    if ((ab < 0  && ac <  0 && cb < 0) || ((ab >= 0) && (ac < 0 || cb < 0))) {
                      invisible = true;
                      goto skip_further_obstacles; 
                    }
                  } 
                }
              }
           skip_further_obstacles:  
              if (!invisible) {
                tails.push_back(v);
                heads.push_back(w);
                weights.push_back(0.5 + geo_dist(latitudes[v],longitudes[v],latitudes[w],longitudes[w]));
              }
            }
          }
        }
      } 
    }

    // Add links from visible nodes to target node
    void add_target(float lat, float lon) {
      auto visibles = visible_vertices(lat, lon);
      for (unsigned vertex: visibles) {
        tails.push_back(vertex);
        heads.push_back(num_nodes);
        weights.push_back(0.5 + geo_dist(lat,lon,latitudes[vertex],longitudes[vertex]));
      }
      latitudes[num_nodes] = lat;
      longitudes[num_nodes] = lon;
      num_nodes++;
    }

    int node_count() {
        return num_nodes;
    }

    int arc_count() {
        return heads.size();
    }

    void sort_graph_for_routing() {
        // TODO: Check whether this is the right permutation to be used
        this->permutation = compute_inverse_sort_permutation_first_by_tail_then_by_head_and_apply_sort_to_tail(this->num_nodes,this->tails, this->heads);
        this->heads = apply_inverse_permutation(this->permutation, std::move(this->heads));
        this->weights = apply_inverse_permutation(this->permutation, std::move(this->weights));
        this->first_out = invert_vector(this->tails, num_nodes);
        this->first_out[num_nodes]=first_out[num_nodes-1]; // reserve space for source node
        this->first_out.push_back(first_out[num_nodes]); // mark end
    }

    bool has_arc(unsigned tail, unsigned head) {
      for (unsigned i = first_out[tail]; i < first_out[tail+1]; i++) {
        if (tails[i] == tail && heads[i] == head)
            return true;
      }
      return false;
    }

    unsigned target() {
        assert(permutation[num_nodes - 1] == num_nodes - 1);
        return num_nodes - 1; 
    }

    // This methods returns the IDs of the visible vertices as
    // stored in the polygon. These may be different from the
    // vertex IDs in the visibilty graph after prepared for routing.
    std::vector<unsigned> visible_vertices(float lat, float lon) {
        std::vector<unsigned> ret;
        for (unsigned v = 0; v < num_nodes; v++) {
            bool invisible = false;
            for (unsigned q = 0; q < first_vertex.size() - 1; q++) {
                for (unsigned i = first_vertex[q], j=first_vertex[q+1] - 1; i<first_vertex[q+1]; j=i++) {
                    float y1=lat, x1=lon;
                    float y2=latitudes[v], x2=longitudes[v];
                    float y3=latitudes[i], x3=longitudes[i];
                    float y4=latitudes[j], x4=longitudes[j];
                    if ((x1!=x3 || y1!=y3) && (x1!=x4 || y1!=y4) && (x2!=x3 || y2!=y3) && (x2!=x4 || y2!=y4)) {
                        invisible = segments_intersect(x1,y1,x2,y2,x3,y3,x4,y4);
                        if (invisible) goto skip_further_obstacles;
                    }
                }
            }
        skip_further_obstacles:
            if (!invisible) {
               ret.push_back(v);
            }
        }
        return ret;
    }

    // Caution: need to call add_target and sort_graph_for_routing before calling this method
    void set_source(float lat, float lon) {
        // Cleanup old source
        for (unsigned i = first_out[num_nodes]; i < first_out[num_nodes+1]; i++) {
            this->tails.pop_back();
            this->heads.pop_back();
            this->weights.pop_back();
        }
        // Add new source
        std::vector<unsigned> visibles = visible_vertices(lat, lon);
        for (unsigned vertex: visibles) {
            this->tails.push_back(num_nodes);
            this->heads.push_back(vertex); // TODO: find out why it crashes here
            this->weights.push_back(0.5 + geo_dist(lat,lon,latitudes[vertex],longitudes[vertex]));
        }
        first_out[num_nodes+1] = first_out[num_nodes]+ visibles.size();
    }

    Dijkstra get_router() {
        Dijkstra dij(first_out, this->tails, this->heads);
        dij.add_source(num_nodes);
        return dij;
    }

    void print_graph(bool all) {
        std::cout << "n = " << num_nodes << std::endl;
        std::cout << "first_out = {";
        for (unsigned i = 0; i < first_out.size(); i++)
            std::cout << first_out[i] << ", ";
        std::cout << "}" << std::endl;
        std::cout << "heads = {";
        for (unsigned i = 0; i < heads.size(); i++)
            std::cout << heads[i] << ", ";
        std::cout << "}" << std::endl;
        if (!all) return;

        std::cout << "tails = {";
        for (unsigned i = 0; i < tails.size(); i++)
            std::cout << tails[i] << ", ";
        std::cout << "}" << std::endl;
        std::cout << "weights = {";
        for (unsigned i = 0; i < weights.size(); i++)
            std::cout << weights[i] << ", ";
        std::cout << "}" << std::endl;
        std::cout << "latitudes = {";
        for (unsigned i = 0; i < latitudes.size(); i++)
            std::cout << latitudes[i] << ", ";
        std::cout << "}" << std::endl;
        std::cout << "longitudes = {";
        for (unsigned i = 0; i < longitudes.size(); i++)
            std::cout << longitudes[i] << ", ";
        std::cout << "}" << std::endl;
    }

    std::vector<unsigned> weights; // TODO: encapsulation!
  private:
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
