# RoadmapPlanner


This repository contains code for our project RoadmapPlanner.


# environment 

To setup a conda environment, run the following command

    conda env create -f environment.yml


To build cython extensions, run

    python setup.py build_ext --inplace


# Visualizing results


To visualize the results of our algorithm for constructing a path between two arbitary points, 
run 

    python robot_planning.py


To modify the starting and ending locations of the search, change lines 110 and 112 in the 
file. 


