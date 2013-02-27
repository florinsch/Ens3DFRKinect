#!/bin/bash
FORESTS=/usr/local/src/pcl-trunk-svn/gpu/people/data/results/

./ppl_app -numTrees 3 \
-tree0 $FORESTS/forest1/tree_20.txt \
-tree1 $FORESTS/forest2/tree_20.txt \
-tree2 $FORESTS/forest3/tree_20.txt \
-tree3 $FORESTS/forest3/tree_20.txt -mask 0 -FG 0 $1

