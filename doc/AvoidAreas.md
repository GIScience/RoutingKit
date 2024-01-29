# Avoid Areas Routing

In order to work with avoid areas routing, you need to perform the following steps:

1. Create a graph, e.g., with `osm_extract`
1. Create a set of avoid polygons in a text file. 
   Each line of this file represents one polygon given as a space-seperated 
   list of `lat lon lat lon ...`, where each `lat lon` pair represents one
   polygon vertex. The polygon is closed automatically, i.e. you SHOULD NOT
   repeat the first vertex at the end of the list.
1. Use `map_polygons_to_nodes_and_edges` to mark graph edges to be avoided.
1. Use `run_astar`, `run_esp` or  `run_cch_aa`
