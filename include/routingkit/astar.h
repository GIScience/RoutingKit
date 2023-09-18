#ifndef ASTAR_H
#define ASTAR_H

#include <routingkit/id_queue.h>
#include <routingkit/constants.h>
#include <routingkit/timestamp_flag.h>
#include <routingkit/geo_dist.h>
#include <routingkit/bit_vector.h>
#include <routingkit/visibility_graph.h>
#include <vector>

#include <iostream>

namespace RoutingKit{

class Astar{
public:
	Astar():first_out(nullptr){}

	Astar(const std::vector<unsigned>&first_out, const std::vector<unsigned>&tail, const std::vector<unsigned>&head):
		tentative_distance(first_out.size()-1,inf_weight),
		predecessor_arc(first_out.size()-1),
		was_popped(first_out.size()-1),
		queue(first_out.size()-1),
		first_out(&first_out),
		tail(&tail),
		head(&head){
		assert(!first_out.empty());
		assert(first_out.front() == 0);
		assert(first_out.back() == tail.size());
		assert(first_out.back() == head.size());
	}

	Astar&reset(){
        std::fill(tentative_distance.begin(), tentative_distance.end(), inf_weight);
		queue.clear();
		was_popped.reset_all();
        avoid_edges = nullptr;
		return *this;
	}

	Astar&reset(const std::vector<unsigned>&first_out, const std::vector<unsigned>&tail, const std::vector<unsigned>&head){
		assert(!first_out.empty());
		assert(first_out.front() == 0);
		assert(first_out.back() == tail.size());
		assert(first_out.back() == head.size());

		if(this->first_out != nullptr && first_out.size() == this->first_out->size()){
			this->first_out = &first_out;
			this->head = &head;
			this->tail = &tail;
            std::fill(tentative_distance.begin(), tentative_distance.end(), inf_weight);
			queue.clear();
			was_popped.reset_all();
            avoid_edges = nullptr;
			return *this;
		}else{
			this->first_out = &first_out;
			this->head = &head;
			this->tail = &tail;
			tentative_distance.resize(first_out.size()-1);
            std::fill(tentative_distance.begin(), tentative_distance.end(), inf_weight);
			predecessor_arc.resize(first_out.size()-1);
			was_popped = TimestampFlags(first_out.size()-1);
			queue = MinIDQueue(first_out.size()-1);
			return *this;
		}
	}

	Astar&add_source(unsigned id, unsigned departure_time = 0){
		assert(id < first_out->size()-1);
		tentative_distance[id] = departure_time;
		predecessor_arc[id] = invalid_id;
		queue.push({id, departure_time});
		return *this;
	}

    Astar&set_avoid_edges(BitVector *avoid_edges){
        assert(avoid_edges->size() == this->head->size());
        this->avoid_edges = avoid_edges;
        return *this;
    }

	bool is_finished()const{
		return queue.empty();
	}

	bool was_node_reached(unsigned x)const{
		assert(x < first_out->size()-1);
		return was_popped.is_set(x);
	}

	struct SettleResult{
		unsigned node;
		unsigned distance;
	};

	template<class GetWeightFunc, class HeuristicFunc>
	SettleResult settle(const GetWeightFunc&get_weight, HeuristicFunc&heuristic){
		assert(!is_finished());

		auto p = queue.pop();
		was_popped.set(p.id);
        
		for(unsigned a=(*first_out)[p.id]; a<(*first_out)[p.id+1]; ++a){
			if((!was_popped.is_set((*head)[a]))&&(!avoid_edges->is_set(a))){
				unsigned w = get_weight(a, p.key);
				if(w < inf_weight){
					unsigned score = tentative_distance[p.id] + w;
					if (score < tentative_distance[(*head)[a]]) {
						if(queue.contains_id((*head)[a])){
							queue.decrease_key({(*head)[a], score + heuristic((*head)[a])});
						} else {
							queue.push({(*head)[a], score + heuristic((*head)[a])});
						}
                        predecessor_arc[(*head)[a]] = a;
						tentative_distance[(*head)[a]] = score;
					}
				}
			}
		}
		return SettleResult{p.id, tentative_distance[p.id]};
	}

	unsigned get_distance_to(unsigned x) const {
		assert(x < first_out->size()-1);
		if(was_popped.is_set(x))
			return tentative_distance[x];
		else
			return inf_weight;
	}

	std::vector<unsigned>get_node_path_to(unsigned x) const {
		assert(x < first_out->size()-1);
		std::vector<unsigned>path;
		if(was_node_reached(x)){
			assert(was_node_reached(x));

			unsigned p;
			while(p = predecessor_arc[x], p != invalid_id){
				path.push_back(x);
				x = (*tail)[p];
			}
			path.push_back(x);
			std::reverse(path.begin(), path.end());
		}
		return path;
	}

	std::vector<unsigned>get_arc_path_to(unsigned x) const {
		assert(x < first_out->size()-1);
		std::vector<unsigned>path;
		if(was_node_reached(x)){
			unsigned p;
			while(p = predecessor_arc[x], p != invalid_id){
				path.push_back(p);
				x = (*tail)[p];
			}
			std::reverse(path.begin(), path.end());
		}
		return path;
	}

private:
	std::vector<unsigned>tentative_distance;
	std::vector<unsigned>predecessor_arc;

	TimestampFlags was_popped;
	MinIDQueue queue;

	const std::vector<unsigned>*first_out;
	const std::vector<unsigned>*tail;
	const std::vector<unsigned>*head;
	const BitVector *avoid_edges;
};

class ScalarGetWeight{
public:
	explicit ScalarGetWeight(const std::vector<unsigned>&weight):weight(&weight){}

	unsigned operator()(unsigned arc, unsigned departure_time)const{
		(void)departure_time;
		return (*weight)[arc];
	}

private:
	const std::vector<unsigned>*weight;
};

class ZeroHeuristic {
public:
	explicit ZeroHeuristic(){}

	unsigned operator ()(unsigned id) const {
		(void)id;
		return 0;
	}
};


class BeelineDistanceHeuristic{
public:
	explicit BeelineDistanceHeuristic(const std::vector<float>&latitude, const std::vector<float>&longitude, unsigned target_id):
		latitude(&latitude),
		longitude(&longitude),
		target_id(target_id)
	{}

	unsigned operator()(unsigned id) const {
		return (unsigned)(0.5+geo_dist((*latitude)[id],(*longitude)[id],(*latitude)[target_id],(*longitude)[target_id]));
	}

private:
	const std::vector<float>*latitude;
	const std::vector<float>*longitude;
	const unsigned target_id;
};

class EspHeuristic {
    public:
        explicit EspHeuristic(VisibilityGraph &vg):
            visibility(&vg)
        {}
        unsigned operator()(unsigned id) const {
            // TODO: on each call, extend vg with lat/lon of id and
            //       run dijkstra to get ESP distance (the actual path
            //       is not needed)
            visibility->size();// dummy call to avoid compilation warning "private field 'visibility' is not used"
            return 0; 
        }
    private:
        VisibilityGraph *visibility;
        
};

} // RoutingKit
#endif
