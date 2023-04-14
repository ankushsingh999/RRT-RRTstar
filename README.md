# RRT-RRTstar
RRT and RRT star are two standard search motion planning algorithms. RRT is Rapidly exploring Random Tree. A tree is started at the starting configuration and randomly search space.
RRT.py is the file where a RRT class for RRT and RRT* is implemented.
main.py is the script that provides helper functions that load the map from an image and call the classes and functions from
WPI_map.jpg is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.


The coordinate system used here is [row, col], which is different from [x, y] in Cartesian coordinates. InREADME and the code comment, when the word 'point' is used, it refers to a simple list [row, col]. When the word 'node' or 'vertex'is used, it refers to the Node class in RRT.

# RRT: 
The algorithm starts with generating a random node and expanding the tree. The validity of the nodes is checked and the collision check is verified. A heap is used to pop out the node with lowest distance.If it satisfies the conditions the parent and the cost of the new node are updated. When the distance of the node is very close to the goal, the goal is appended to the goal.

# Results : 

![RRT](https://user-images.githubusercontent.com/64325043/231944928-2fcce3df-09ad-4591-9718-bd6b9571b8ac.png)

![image](https://user-images.githubusercontent.com/64325043/231945049-654ce2d2-e537-43bd-90da-bf19573d618e.png)


# RRT* :
RRT*:
RRT* is an extension of RRT. Its asymptomatically stable,i.e., as sampling approaches infinity the algorithm will deliver the optimal path. The heuristic of this algorithm is the cost which Iis the distance from the nodes. A rewiring function is additionally called which re constructs roadmap iteratively re-evaluating and updating the connections between the nodes.

# Results :

![RRT star](https://user-images.githubusercontent.com/64325043/231945131-9972b573-84fa-40d6-bf5c-aefb7c5fd109.png)

![image](https://user-images.githubusercontent.com/64325043/231945099-dd99117b-454b-441a-baca-7cc6a1b5d942.png)
