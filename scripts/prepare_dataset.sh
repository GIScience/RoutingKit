#!/bin/bash

if [ $# -ne 2 ];
    then echo Usage: $0 RoutingKit_path pbf_file
    exit
fi;

BIN_PATH=$1
PBF_FILE=$2

$BIN_PATH/osm_extract $PBF_FILE first_out head geo_dist travel_time lat lon
cp geo_dist weights_original
$BIN_PATH/map_polygons_to_nodes_and_edges first_out head lat lon polygons weights_original weights_avoidables avoid_nodes avoid_edges


