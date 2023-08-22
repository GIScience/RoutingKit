library(sf)
library(geojsonsf)

args <- commandArgs(trailingOnly = TRUE)

if (length(args) < 3)
  stop("Usage: pip.sh latitudes longitudes geojson [output]", call. = FALSE)

lat <- readBin(args[1], "numeric", n = 1000000L, size = 4L)
lon <- readBin(args[2], "numeric", n = 1000000L, size = 4L)
polygon <- geojson_sf(readLines(args[3]))

coords <- data.frame(lat = lat, lon = lon)

points <- st_as_sf(coords, coords = c("lon", "lat"), crs = st_crs(polygon))

in_polygon <- as.integer(st_within(points, polygon, sparse = FALSE)[, 1])

if (length(args) == 3) {
  cat(in_polygon, sep = "\n")
} else {
  writeBin(in_polygon, args[4], size = 2)## encode as int16
}
