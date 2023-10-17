#!/usr/local/bin/Rscript
library(jsonlite)

args <- commandArgs(trailingOnly = TRUE)

if (length(args) < 1)
  stop("Usage: ./extract_multipolygon_coordinates geojson_file [output]", call. = FALSE)

geojson <- fromJSON(args[1], simplifyDataFrame = FALSE, simplifyMatrix = FALSE)

lines <- sapply(geojson$features, function(feature) {
  coordinates <- do.call(rbind, feature$geometry$coordinates[[1]][[1]])
  lon <- coordinates[,1]
  lat <- coordinates[,2]
  paste(lat, lon, sep = " ", collapse = " ")
})

if (length(args) == 2) {
  writeLines(lines, args[2])
} else {
  cat(lines, sep = "\n")
}
