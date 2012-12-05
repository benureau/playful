cmake_minimum_required(VERSION 2.8)

project( NeighborController )
find_package( OpenCV REQUIRED )

# OpenCV C++ API kmeans sample 
add_executable( kmeanscv kmeanscv )
target_link_libraries( kmeanscv ${OpenCV_LIBS} )
