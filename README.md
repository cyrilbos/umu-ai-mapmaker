# University of Umeå - Advanced AI assignment
Repository for the most difficult AI assignment at the University of Umeå, Sweden. 


## Purpose of the assignment
The goal is to implement multiple algorithms to make a roomba-like robot explore an environment and map it using range-limited laser sensors. 
The robot computes frontiers to go to to discover the most unknown regions. I used an optimized version of A* on the configuration space to find paths to the frontier points. The laser sensor model uses Bayes theory to update the probabilities on a given cell of the map and other parameters such as previous readings, the range, etc. The path tracking algorithm (pure pursuit) used while travelling from a point to another originates from a previous assignment (in another of my repository: Umu-AI-Fundamentals).

The simulator used is MRDS, originally created by Microsoft but sadly not developed anymore. The assignment should transition to another environment in the future. 

## Video example run
A video of an example run can be found here: 
[https://www.youtube.com/watch?v=6mEpk-EFsFs](https://www.youtube.com/watch?v=6mEpk-EFsFs)