#!/bin/bash
# Script to 
# - clean the tree from all compiled files
# - build
# - run path planning!
rm -rf build
mkdir build
cd build
cmake ..
make
./path_planning
