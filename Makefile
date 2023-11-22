# This makefile was automatically generated. Run ./generate_make_file to regenerate the file.
CC=g++
AR=ar
CFLAGS=-Wall -DNDEBUG -march=native -ffast-math -std=c++11 -O3 -DROUTING_KIT_NO_ALIGNED_ALLOC -fPIC -Iinclude
LDFLAGS=
OMP_CFLAGS=
OMP_LDFLAGS=

all: bin/compare_vector bin/compute_contraction_hierarchy bin/compute_geographic_distance_weights bin/compute_nested_dissection_order bin/convert_road_dimacs_coordinates bin/convert_road_dimacs_graph bin/decode_vector bin/encode_vector bin/examine_ch bin/export_geojson_graph bin/export_road_dimacs_graph bin/export_simple_geojson_graph bin/generate_constant_vector bin/generate_dijkstra_rank_test_queries bin/generate_random_node_list bin/generate_random_source_times bin/generate_test_queries bin/graph_to_dot bin/graph_to_svg bin/map_polygons_to_nodes_and_edges bin/osm_extract bin/randomly_permute_nodes bin/run_astar bin/run_astar_esp bin/run_astar_zero bin/run_contraction_hierarchy_query bin/run_dijkstra bin/run_geo_position_to_node bin/show_path bin/test_astar_esp bin/test_basic_features bin/test_bit_vector bin/test_buffered_asynchronous_reader bin/test_contraction_hierarchy_extra_weight bin/test_contraction_hierarchy_path_query bin/test_contraction_hierarchy_pinned_query bin/test_customizable_contraction_hierarchy bin/test_customizable_contraction_hierarchy_customization bin/test_customizable_contraction_hierarchy_path_query bin/test_customizable_contraction_hierarchy_perfect_customization bin/test_customizable_contraction_hierarchy_pinned_query bin/test_customizable_contraction_hierarchy_reset bin/test_dijkstra bin/test_edge_crosses_polygon bin/test_geo_dist bin/test_id_mapper bin/test_id_set_queue bin/test_inverse_vector bin/test_nearest_neighbor bin/test_nested_dissection bin/test_osm_simple bin/test_permutation bin/test_point_in_polygon bin/test_protobuf bin/test_sort bin/test_strongly_connected_component bin/test_tag_map bin/test_visibility_graph lib/libroutingkit.a lib/libroutingkit.so

build/bit_select.o: src/bit_select.cpp src/bit_select.h src/emulate_gcc_builtin.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/bit_select.cpp -o build/bit_select.o

build/bit_vector.o: include/routingkit/bit_vector.h src/bit_vector.cpp src/emulate_gcc_builtin.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/bit_vector.cpp -o build/bit_vector.o

build/buffered_asynchronous_reader.o: src/buffered_asynchronous_reader.cpp src/buffered_asynchronous_reader.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/buffered_asynchronous_reader.cpp -o build/buffered_asynchronous_reader.o

build/compare_vector.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/compare_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/compare_vector.cpp -o build/compare_vector.o

build/compute_contraction_hierarchy.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/compute_contraction_hierarchy.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/compute_contraction_hierarchy.cpp -o build/compute_contraction_hierarchy.o

build/compute_geographic_distance_weights.o: include/routingkit/bit_vector.h include/routingkit/geo_dist.h include/routingkit/min_max.h include/routingkit/timer.h include/routingkit/vector_io.h src/compute_geographic_distance_weights.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/compute_geographic_distance_weights.cpp -o build/compute_geographic_distance_weights.o

build/compute_nested_dissection_order.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/nested_dissection.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/compute_nested_dissection_order.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/compute_nested_dissection_order.cpp -o build/compute_nested_dissection_order.o

build/contraction_hierarchy.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/contraction_hierarchy.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/contraction_hierarchy.cpp -o build/contraction_hierarchy.o

build/convert_road_dimacs_coordinates.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/convert_road_dimacs_coordinates.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/convert_road_dimacs_coordinates.cpp -o build/convert_road_dimacs_coordinates.o

build/convert_road_dimacs_graph.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/convert_road_dimacs_graph.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/convert_road_dimacs_graph.cpp -o build/convert_road_dimacs_graph.o

build/customizable_contraction_hierarchy.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/filter.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/id_queue.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h src/customizable_contraction_hierarchy.cpp src/emulate_gcc_builtin.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS) $(OMP_CFLAGS) -c src/customizable_contraction_hierarchy.cpp -o build/customizable_contraction_hierarchy.o

build/decode_vector.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/decode_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/decode_vector.cpp -o build/decode_vector.o

build/encode_vector.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/encode_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/encode_vector.cpp -o build/encode_vector.o

build/examine_ch.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/examine_ch.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/examine_ch.cpp -o build/examine_ch.o

build/expect.o: src/expect.cpp src/expect.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/expect.cpp -o build/expect.o

build/export_geojson_graph.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/geojson.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/export_geojson_graph.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/export_geojson_graph.cpp -o build/export_geojson_graph.o

build/export_road_dimacs_graph.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/export_road_dimacs_graph.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/export_road_dimacs_graph.cpp -o build/export_road_dimacs_graph.o

build/export_simple_geojson_graph.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/geojson.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/export_simple_geojson_graph.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/export_simple_geojson_graph.cpp -o build/export_simple_geojson_graph.o

build/file_data_source.o: src/file_data_source.cpp src/file_data_source.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/file_data_source.cpp -o build/file_data_source.o

build/generate_constant_vector.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/generate_constant_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/generate_constant_vector.cpp -o build/generate_constant_vector.o

build/generate_dijkstra_rank_test_queries.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/generate_dijkstra_rank_test_queries.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/generate_dijkstra_rank_test_queries.cpp -o build/generate_dijkstra_rank_test_queries.o

build/generate_random_node_list.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/generate_random_node_list.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/generate_random_node_list.cpp -o build/generate_random_node_list.o

build/generate_random_source_times.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/generate_random_source_times.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/generate_random_source_times.cpp -o build/generate_random_source_times.o

build/generate_test_queries.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/generate_test_queries.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/generate_test_queries.cpp -o build/generate_test_queries.o

build/geo_position_to_node.o: include/routingkit/constants.h include/routingkit/geo_dist.h include/routingkit/geo_position_to_node.h src/geo_position_to_node.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/geo_position_to_node.cpp -o build/geo_position_to_node.o

build/graph_to_dot.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/permutation.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/graph_to_dot.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/graph_to_dot.cpp -o build/graph_to_dot.o

build/graph_to_svg.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/graph_to_svg.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/graph_to_svg.cpp -o build/graph_to_svg.o

build/graph_util.o: include/routingkit/constants.h include/routingkit/graph_util.h include/routingkit/permutation.h include/routingkit/sort.h src/graph_util.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/graph_util.cpp -o build/graph_util.o

build/id_mapper.o: include/routingkit/constants.h include/routingkit/id_mapper.h src/bit_select.h src/emulate_gcc_builtin.h src/id_mapper.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/id_mapper.cpp -o build/id_mapper.o

build/map_polygons_to_nodes_and_edges.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/edge_crosses_polygon.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/point_in_polygon.h include/routingkit/polygon_io.h include/routingkit/sort.h include/routingkit/vector_io.h src/map_polygons_to_nodes_and_edges.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/map_polygons_to_nodes_and_edges.cpp -o build/map_polygons_to_nodes_and_edges.o

build/nested_dissection.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/filter.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/nested_dissection.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h src/nested_dissection.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/nested_dissection.cpp -o build/nested_dissection.o

build/osm_decoder.o: include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/osm_decoder.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h include/routingkit/timer.h src/buffered_asynchronous_reader.h src/file_data_source.h src/osm_decoder.cpp src/protobuf.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/osm_decoder.cpp -o build/osm_decoder.o

build/osm_extract.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/id_mapper.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/osm_profile.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h include/routingkit/timer.h include/routingkit/vector_io.h src/osm_extract.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/osm_extract.cpp -o build/osm_extract.o

build/osm_graph_builder.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/filter.h include/routingkit/geo_dist.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h include/routingkit/timer.h src/osm_graph_builder.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/osm_graph_builder.cpp -o build/osm_graph_builder.o

build/osm_profile.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/osm_profile.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h src/osm_profile.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/osm_profile.cpp -o build/osm_profile.o

build/osm_simple.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/osm_profile.h include/routingkit/osm_simple.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h src/osm_simple.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/osm_simple.cpp -o build/osm_simple.o

build/protobuf.o: src/protobuf.cpp src/protobuf.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/protobuf.cpp -o build/protobuf.o

build/randomly_permute_nodes.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/vector_io.h src/randomly_permute_nodes.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/randomly_permute_nodes.cpp -o build/randomly_permute_nodes.o

build/run_astar.o: include/routingkit/astar.h include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/geo_dist.h include/routingkit/geojson.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/segments_intersect.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h include/routingkit/visibility_graph.h src/run_astar.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_astar.cpp -o build/run_astar.o

build/run_astar_esp.o: include/routingkit/astar.h include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/geo_dist.h include/routingkit/geojson.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/polygon_io.h include/routingkit/segments_intersect.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h include/routingkit/visibility_graph.h src/run_astar_esp.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_astar_esp.cpp -o build/run_astar_esp.o

build/run_astar_zero.o: include/routingkit/astar.h include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/geo_dist.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/segments_intersect.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h include/routingkit/visibility_graph.h src/run_astar_zero.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_astar_zero.cpp -o build/run_astar_zero.o

build/run_contraction_hierarchy_query.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/run_contraction_hierarchy_query.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_contraction_hierarchy_query.cpp -o build/run_contraction_hierarchy_query.o

build/run_dijkstra.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/run_dijkstra.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_dijkstra.cpp -o build/run_dijkstra.o

build/run_geo_position_to_node.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/geo_position_to_node.h include/routingkit/vector_io.h src/run_geo_position_to_node.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/run_geo_position_to_node.cpp -o build/run_geo_position_to_node.o

build/show_path.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/show_path.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/show_path.cpp -o build/show_path.o

build/strongly_connected_component.o: include/routingkit/min_max.h include/routingkit/strongly_connected_component.h src/strongly_connected_component.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/strongly_connected_component.cpp -o build/strongly_connected_component.o

build/test_astar_esp.o: include/routingkit/astar.h include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/geo_dist.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/segments_intersect.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h include/routingkit/visibility_graph.h src/expect.h src/test_astar_esp.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_astar_esp.cpp -o build/test_astar_esp.o

build/test_basic_features.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/id_mapper.h include/routingkit/id_queue.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/nested_dissection.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/osm_profile.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h include/routingkit/timer.h include/routingkit/timestamp_flag.h src/expect.h src/test_basic_features.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_basic_features.cpp -o build/test_basic_features.o

build/test_bit_vector.o: include/routingkit/bit_vector.h src/expect.h src/test_bit_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_bit_vector.cpp -o build/test_bit_vector.o

build/test_buffered_asynchronous_reader.o: src/buffered_asynchronous_reader.h src/test_buffered_asynchronous_reader.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_buffered_asynchronous_reader.cpp -o build/test_buffered_asynchronous_reader.o

build/test_contraction_hierarchy_extra_weight.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/expect.h src/test_contraction_hierarchy_extra_weight.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_contraction_hierarchy_extra_weight.cpp -o build/test_contraction_hierarchy_extra_weight.o

build/test_contraction_hierarchy_path_query.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/test_contraction_hierarchy_path_query.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_contraction_hierarchy_path_query.cpp -o build/test_contraction_hierarchy_path_query.o

build/test_contraction_hierarchy_pinned_query.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/test_contraction_hierarchy_pinned_query.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_contraction_hierarchy_pinned_query.cpp -o build/test_contraction_hierarchy_pinned_query.o

build/test_customizable_contraction_hierarchy.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/id_mapper.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/test_customizable_contraction_hierarchy.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy.cpp -o build/test_customizable_contraction_hierarchy.o

build/test_customizable_contraction_hierarchy_customization.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/test_customizable_contraction_hierarchy_customization.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy_customization.cpp -o build/test_customizable_contraction_hierarchy_customization.o

build/test_customizable_contraction_hierarchy_path_query.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/test_customizable_contraction_hierarchy_path_query.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy_path_query.cpp -o build/test_customizable_contraction_hierarchy_path_query.o

build/test_customizable_contraction_hierarchy_perfect_customization.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/graph_util.h include/routingkit/id_mapper.h include/routingkit/id_queue.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/test_customizable_contraction_hierarchy_perfect_customization.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy_perfect_customization.cpp -o build/test_customizable_contraction_hierarchy_perfect_customization.o

build/test_customizable_contraction_hierarchy_pinned_query.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/id_mapper.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/test_customizable_contraction_hierarchy_pinned_query.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy_pinned_query.cpp -o build/test_customizable_contraction_hierarchy_pinned_query.o

build/test_customizable_contraction_hierarchy_reset.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/id_mapper.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/expect.h src/test_customizable_contraction_hierarchy_reset.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_customizable_contraction_hierarchy_reset.cpp -o build/test_customizable_contraction_hierarchy_reset.o

build/test_dijkstra.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h src/expect.h src/test_dijkstra.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_dijkstra.cpp -o build/test_dijkstra.o

build/test_edge_crosses_polygon.o: include/routingkit/edge_crosses_polygon.h src/expect.h src/test_edge_crosses_polygon.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_edge_crosses_polygon.cpp -o build/test_edge_crosses_polygon.o

build/test_geo_dist.o: include/routingkit/geo_dist.h include/routingkit/timer.h src/expect.h src/test_geo_dist.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_geo_dist.cpp -o build/test_geo_dist.o

build/test_id_mapper.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/id_mapper.h include/routingkit/timer.h src/bit_select.h src/expect.h src/test_id_mapper.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_id_mapper.cpp -o build/test_id_mapper.o

build/test_id_set_queue.o: include/routingkit/constants.h include/routingkit/id_set_queue.h include/routingkit/min_max.h src/expect.h src/test_id_set_queue.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_id_set_queue.cpp -o build/test_id_set_queue.o

build/test_inverse_vector.o: include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h src/expect.h src/test_inverse_vector.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_inverse_vector.cpp -o build/test_inverse_vector.o

build/test_nearest_neighbor.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/geo_dist.h include/routingkit/geo_position_to_node.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h include/routingkit/vector_io.h src/expect.h src/test_nearest_neighbor.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_nearest_neighbor.cpp -o build/test_nearest_neighbor.o

build/test_nested_dissection.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/nested_dissection.h include/routingkit/permutation.h include/routingkit/sort.h src/expect.h src/test_nested_dissection.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_nested_dissection.cpp -o build/test_nested_dissection.o

build/test_osm_simple.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/contraction_hierarchy.h include/routingkit/customizable_contraction_hierarchy.h include/routingkit/id_mapper.h include/routingkit/id_queue.h include/routingkit/id_set_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/nested_dissection.h include/routingkit/osm_decoder.h include/routingkit/osm_graph_builder.h include/routingkit/osm_simple.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h include/routingkit/timer.h include/routingkit/timestamp_flag.h src/expect.h src/test_osm_simple.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_osm_simple.cpp -o build/test_osm_simple.o

build/test_permutation.o: include/routingkit/constants.h include/routingkit/permutation.h src/expect.h src/test_permutation.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_permutation.cpp -o build/test_permutation.o

build/test_point_in_polygon.o: include/routingkit/point_in_polygon.h src/expect.h src/test_point_in_polygon.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_point_in_polygon.cpp -o build/test_point_in_polygon.o

build/test_protobuf.o: src/expect.h src/protobuf.h src/test_protobuf.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_protobuf.cpp -o build/test_protobuf.o

build/test_sort.o: include/routingkit/constants.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/timer.h src/expect.h src/test_sort.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_sort.cpp -o build/test_sort.o

build/test_strongly_connected_component.o: include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/strongly_connected_component.h src/expect.h src/test_strongly_connected_component.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_strongly_connected_component.cpp -o build/test_strongly_connected_component.o

build/test_tag_map.o: include/routingkit/constants.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/sort.h include/routingkit/tag_map.h src/expect.h src/test_tag_map.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_tag_map.cpp -o build/test_tag_map.o

build/test_visibility_graph.o: include/routingkit/bit_vector.h include/routingkit/constants.h include/routingkit/dijkstra.h include/routingkit/geo_dist.h include/routingkit/graph_util.h include/routingkit/id_queue.h include/routingkit/inverse_vector.h include/routingkit/min_max.h include/routingkit/permutation.h include/routingkit/segments_intersect.h include/routingkit/sort.h include/routingkit/timestamp_flag.h include/routingkit/vector_io.h include/routingkit/visibility_graph.h src/expect.h src/test_visibility_graph.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/test_visibility_graph.cpp -o build/test_visibility_graph.o

build/timer.o: include/routingkit/timer.h src/timer.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/timer.cpp -o build/timer.o

build/vector_io.o: include/routingkit/bit_vector.h include/routingkit/vector_io.h src/vector_io.cpp generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/vector_io.cpp -o build/vector_io.o

build/verify.o: src/verify.cpp src/verify.h generate_make_file
	@mkdir -p build
	$(CC) $(CFLAGS)  -c src/verify.cpp -o build/verify.o

bin/compare_vector: build/bit_vector.o build/compare_vector.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/compare_vector.o build/vector_io.o -pthread  -o bin/compare_vector

bin/compute_contraction_hierarchy: build/bit_vector.o build/compute_contraction_hierarchy.o build/contraction_hierarchy.o build/graph_util.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/compute_contraction_hierarchy.o build/contraction_hierarchy.o build/graph_util.o build/timer.o build/vector_io.o build/verify.o -pthread  -o bin/compute_contraction_hierarchy

bin/compute_geographic_distance_weights: build/bit_vector.o build/compute_geographic_distance_weights.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/compute_geographic_distance_weights.o build/timer.o build/vector_io.o build/verify.o -lm -pthread  -o bin/compute_geographic_distance_weights

bin/compute_nested_dissection_order: build/bit_select.o build/bit_vector.o build/compute_nested_dissection_order.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/compute_nested_dissection_order.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/timer.o build/vector_io.o -pthread  -o bin/compute_nested_dissection_order

bin/convert_road_dimacs_coordinates: build/bit_vector.o build/convert_road_dimacs_coordinates.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/convert_road_dimacs_coordinates.o build/vector_io.o -pthread  -o bin/convert_road_dimacs_coordinates

bin/convert_road_dimacs_graph: build/bit_vector.o build/convert_road_dimacs_graph.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/convert_road_dimacs_graph.o build/vector_io.o -pthread  -o bin/convert_road_dimacs_graph

bin/decode_vector: build/bit_vector.o build/decode_vector.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/decode_vector.o build/vector_io.o -pthread  -o bin/decode_vector

bin/encode_vector: build/bit_vector.o build/encode_vector.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/encode_vector.o build/vector_io.o -pthread  -o bin/encode_vector

bin/examine_ch: build/bit_vector.o build/contraction_hierarchy.o build/examine_ch.o build/graph_util.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/examine_ch.o build/graph_util.o build/timer.o build/vector_io.o build/verify.o -pthread  -o bin/examine_ch

bin/export_geojson_graph: build/bit_vector.o build/export_geojson_graph.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/export_geojson_graph.o build/vector_io.o build/verify.o -pthread  -o bin/export_geojson_graph

bin/export_road_dimacs_graph: build/bit_vector.o build/export_road_dimacs_graph.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/export_road_dimacs_graph.o build/vector_io.o -pthread  -o bin/export_road_dimacs_graph

bin/export_simple_geojson_graph: build/bit_vector.o build/export_simple_geojson_graph.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/export_simple_geojson_graph.o build/vector_io.o build/verify.o -pthread  -o bin/export_simple_geojson_graph

bin/generate_constant_vector: build/bit_vector.o build/generate_constant_vector.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/generate_constant_vector.o build/vector_io.o -pthread  -o bin/generate_constant_vector

bin/generate_dijkstra_rank_test_queries: build/bit_vector.o build/generate_dijkstra_rank_test_queries.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/generate_dijkstra_rank_test_queries.o build/vector_io.o build/verify.o -pthread  -o bin/generate_dijkstra_rank_test_queries

bin/generate_random_node_list: build/bit_vector.o build/generate_random_node_list.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/generate_random_node_list.o build/vector_io.o -pthread  -o bin/generate_random_node_list

bin/generate_random_source_times: build/bit_vector.o build/generate_random_source_times.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/generate_random_source_times.o build/vector_io.o -pthread  -o bin/generate_random_source_times

bin/generate_test_queries: build/bit_vector.o build/generate_test_queries.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/generate_test_queries.o build/vector_io.o -pthread  -o bin/generate_test_queries

bin/graph_to_dot: build/bit_vector.o build/contraction_hierarchy.o build/graph_to_dot.o build/graph_util.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_to_dot.o build/graph_util.o build/timer.o build/vector_io.o -pthread  -o bin/graph_to_dot

bin/graph_to_svg: build/bit_vector.o build/contraction_hierarchy.o build/graph_to_svg.o build/graph_util.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_to_svg.o build/graph_util.o build/timer.o build/vector_io.o -pthread  -o bin/graph_to_svg

bin/map_polygons_to_nodes_and_edges: build/bit_vector.o build/map_polygons_to_nodes_and_edges.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/map_polygons_to_nodes_and_edges.o build/vector_io.o -pthread  -o bin/map_polygons_to_nodes_and_edges

bin/osm_extract: build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/osm_decoder.o build/osm_extract.o build/osm_graph_builder.o build/osm_profile.o build/protobuf.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/osm_decoder.o build/osm_extract.o build/osm_graph_builder.o build/osm_profile.o build/protobuf.o build/timer.o build/vector_io.o -lm -lz -pthread  -o bin/osm_extract

bin/randomly_permute_nodes: build/bit_vector.o build/randomly_permute_nodes.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/randomly_permute_nodes.o build/vector_io.o -pthread  -o bin/randomly_permute_nodes

bin/run_astar: build/bit_vector.o build/graph_util.o build/run_astar.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/graph_util.o build/run_astar.o build/timer.o build/vector_io.o build/verify.o -lm -pthread  -o bin/run_astar

bin/run_astar_esp: build/bit_vector.o build/graph_util.o build/run_astar_esp.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/graph_util.o build/run_astar_esp.o build/timer.o build/vector_io.o build/verify.o -lm -pthread  -o bin/run_astar_esp

bin/run_astar_zero: build/bit_vector.o build/graph_util.o build/run_astar_zero.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/graph_util.o build/run_astar_zero.o build/timer.o build/vector_io.o build/verify.o -lm -pthread  -o bin/run_astar_zero

bin/run_contraction_hierarchy_query: build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/run_contraction_hierarchy_query.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/run_contraction_hierarchy_query.o build/timer.o build/vector_io.o -pthread  -o bin/run_contraction_hierarchy_query

bin/run_dijkstra: build/bit_vector.o build/run_dijkstra.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/run_dijkstra.o build/timer.o build/vector_io.o build/verify.o -pthread  -o bin/run_dijkstra

bin/run_geo_position_to_node: build/bit_vector.o build/geo_position_to_node.o build/run_geo_position_to_node.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/geo_position_to_node.o build/run_geo_position_to_node.o build/vector_io.o -lm -pthread  -o bin/run_geo_position_to_node

bin/show_path: build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/show_path.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/show_path.o build/timer.o build/vector_io.o -pthread  -o bin/show_path

bin/test_astar_esp: build/bit_vector.o build/expect.o build/graph_util.o build/test_astar_esp.o build/timer.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/expect.o build/graph_util.o build/test_astar_esp.o build/timer.o build/vector_io.o build/verify.o -lm -pthread  -o bin/test_astar_esp

bin/test_basic_features: build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/protobuf.o build/test_basic_features.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/protobuf.o build/test_basic_features.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -lm -lz -pthread  -o bin/test_basic_features

bin/test_bit_vector: build/bit_vector.o build/expect.o build/test_bit_vector.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/expect.o build/test_bit_vector.o -pthread  -o bin/test_bit_vector

bin/test_buffered_asynchronous_reader: build/buffered_asynchronous_reader.o build/test_buffered_asynchronous_reader.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/buffered_asynchronous_reader.o build/test_buffered_asynchronous_reader.o -pthread  -o bin/test_buffered_asynchronous_reader

bin/test_contraction_hierarchy_extra_weight: build/bit_vector.o build/contraction_hierarchy.o build/expect.o build/graph_util.o build/test_contraction_hierarchy_extra_weight.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/expect.o build/graph_util.o build/test_contraction_hierarchy_extra_weight.o build/timer.o build/vector_io.o -pthread  -o bin/test_contraction_hierarchy_extra_weight

bin/test_contraction_hierarchy_path_query: build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/test_contraction_hierarchy_path_query.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/test_contraction_hierarchy_path_query.o build/timer.o build/vector_io.o -pthread  -o bin/test_contraction_hierarchy_path_query

bin/test_contraction_hierarchy_pinned_query: build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/test_contraction_hierarchy_pinned_query.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/contraction_hierarchy.o build/graph_util.o build/test_contraction_hierarchy_pinned_query.o build/timer.o build/vector_io.o -pthread  -o bin/test_contraction_hierarchy_pinned_query

bin/test_customizable_contraction_hierarchy: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy

bin/test_customizable_contraction_hierarchy_customization: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_customization.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_customization.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy_customization

bin/test_customizable_contraction_hierarchy_path_query: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_path_query.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_path_query.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy_path_query

bin/test_customizable_contraction_hierarchy_perfect_customization: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_perfect_customization.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_perfect_customization.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy_perfect_customization

bin/test_customizable_contraction_hierarchy_pinned_query: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_pinned_query.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_pinned_query.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy_pinned_query

bin/test_customizable_contraction_hierarchy_reset: build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_reset.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/graph_util.o build/id_mapper.o build/test_customizable_contraction_hierarchy_reset.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -pthread  -o bin/test_customizable_contraction_hierarchy_reset

bin/test_dijkstra: build/bit_vector.o build/expect.o build/test_dijkstra.o build/vector_io.o build/verify.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/expect.o build/test_dijkstra.o build/vector_io.o build/verify.o -pthread  -o bin/test_dijkstra

bin/test_edge_crosses_polygon: build/expect.o build/test_edge_crosses_polygon.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_edge_crosses_polygon.o  -o bin/test_edge_crosses_polygon

bin/test_geo_dist: build/expect.o build/test_geo_dist.o build/timer.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_geo_dist.o build/timer.o -lm  -o bin/test_geo_dist

bin/test_id_mapper: build/bit_select.o build/bit_vector.o build/expect.o build/id_mapper.o build/test_id_mapper.o build/timer.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/expect.o build/id_mapper.o build/test_id_mapper.o build/timer.o -pthread  -o bin/test_id_mapper

bin/test_id_set_queue: build/expect.o build/test_id_set_queue.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_id_set_queue.o  -o bin/test_id_set_queue

bin/test_inverse_vector: build/expect.o build/test_inverse_vector.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_inverse_vector.o  -o bin/test_inverse_vector

bin/test_nearest_neighbor: build/bit_vector.o build/expect.o build/geo_position_to_node.o build/test_nearest_neighbor.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/expect.o build/geo_position_to_node.o build/test_nearest_neighbor.o build/timer.o build/vector_io.o -lm -pthread  -o bin/test_nearest_neighbor

bin/test_nested_dissection: build/bit_select.o build/bit_vector.o build/expect.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/test_nested_dissection.o build/timer.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/expect.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/test_nested_dissection.o build/timer.o -pthread  -o bin/test_nested_dissection

bin/test_osm_simple: build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/test_osm_simple.o build/timer.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/expect.o build/file_data_source.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/test_osm_simple.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -lm -lz -pthread  -o bin/test_osm_simple

bin/test_permutation: build/expect.o build/test_permutation.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_permutation.o  -o bin/test_permutation

bin/test_point_in_polygon: build/expect.o build/test_point_in_polygon.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_point_in_polygon.o  -o bin/test_point_in_polygon

bin/test_protobuf: build/expect.o build/protobuf.o build/test_protobuf.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/protobuf.o build/test_protobuf.o  -o bin/test_protobuf

bin/test_sort: build/expect.o build/test_sort.o build/timer.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_sort.o build/timer.o  -o bin/test_sort

bin/test_strongly_connected_component: build/expect.o build/strongly_connected_component.o build/test_strongly_connected_component.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/strongly_connected_component.o build/test_strongly_connected_component.o  -o bin/test_strongly_connected_component

bin/test_tag_map: build/expect.o build/test_tag_map.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/expect.o build/test_tag_map.o  -o bin/test_tag_map

bin/test_visibility_graph: build/bit_vector.o build/expect.o build/graph_util.o build/test_visibility_graph.o build/vector_io.o
	@mkdir -p bin
	$(CC) $(LDFLAGS) build/bit_vector.o build/expect.o build/graph_util.o build/test_visibility_graph.o build/vector_io.o -lm -pthread  -o bin/test_visibility_graph

lib/libroutingkit.a: build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/file_data_source.o build/geo_position_to_node.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/strongly_connected_component.o build/timer.o build/vector_io.o
	@mkdir -p lib
	$(AR) rcs lib/libroutingkit.a build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/file_data_source.o build/geo_position_to_node.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/strongly_connected_component.o build/timer.o build/vector_io.o

lib/libroutingkit.so: build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/file_data_source.o build/geo_position_to_node.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/strongly_connected_component.o build/timer.o build/vector_io.o
	@mkdir -p lib
	$(CC) -shared $(LDFLAGS) build/bit_select.o build/bit_vector.o build/buffered_asynchronous_reader.o build/contraction_hierarchy.o build/customizable_contraction_hierarchy.o build/file_data_source.o build/geo_position_to_node.o build/graph_util.o build/id_mapper.o build/nested_dissection.o build/osm_decoder.o build/osm_graph_builder.o build/osm_profile.o build/osm_simple.o build/protobuf.o build/strongly_connected_component.o build/timer.o build/vector_io.o $(OMP_LDFLAGS) -lm -lz -pthread -o lib/libroutingkit.so

.PHONY:
clean:
	$(RM) -rf bin build lib
