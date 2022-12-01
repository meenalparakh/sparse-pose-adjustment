# Sparse Pose Adjustment
This is an implementation of the algorithm given in [1] for Pose Graph Optimization. The code is in Julia. <br/>
Three initialization techniques have been implmented: (a) odometry, (b) spanning tree, and (c) chordal relaxation.

## To test out the code in Atom
- Follow the instructions here to install Julia, Atom, and Juno: https://docs.junolab.org/stable/man/installation/
- Open up the `examples.jl` file under `SparsePoseAdjustment` directory in Atom. Execute the instructions from the beginning (similar to Jupyter Notebook).
- The code uses datasets in g2o format. To test the code on a new dataset, specify the location of the file similar to examples shown in `examples.jl`. 

## Example 
Following are the optimized pose graphs, using chordal relaxation initialization technique.
<p float="left">
  <img src="https://github.com/meenalparakh/sparse-pose-adjustment/blob/master/results/city.png" width="300" title="This is a Title" />
  <img src="https://github.com/meenalparakh/sparse-pose-adjustment/blob/master/results/CSAIL.png" width="300" /> 
</p>

<p align = "left">
(a) City10000 dataset &emsp; &emsp; &emsp; &emsp; &emsp;  &emsp;  &emsp;  &emsp; (b) CSAIL dataset.
</p>

<p float="left">
  <img src="https://github.com/meenalparakh/sparse-pose-adjustment/blob/master/results/manhattan.png" width="300" /> 
  <img src="https://github.com/meenalparakh/sparse-pose-adjustment/blob/master/results/kitti.png" width="300" /> 
</p>
<p align = "left">
(c) Manhattan dataset &emsp; &emsp; &emsp; &emsp; &emsp;  &emsp;  &emsp;  &emsp;  (d) Kitti dataset.
</p>


## References
[1] Konolige, K., Grisetti, G., Kümmerle, R., Burgard, W., Limketkai, B., & Vincent, R. (2010). Efficient Sparse Pose Adjustment for 2D mapping. 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems, 22-29.
[2] Juno: https://docs.junolab.org/stable/man/installation/
