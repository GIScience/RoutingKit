#!/bin/bash
if [ "$#" -le 2 ]; then
	echo "Usage: pip.sh latitudes_file longitudes_file geojson_file [output_file]"
	exit 1		
fi
RScript --vanilla ./pip.R $1 $2 $3 $4
