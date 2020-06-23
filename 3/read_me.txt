The zip contains vrep.py, vrepConst.py, remoteApi.dll(32-bit), Project3_V_Rep.py ,this read me file and some simulation videos.
The program was done in pycharm(32-bit).
The Project3_V_Rep.py is the main code file.
The libraries imported are math, mathplotlib, operator, time and vrep.
The dt has been given as 2/100 seconds.
The clearance is given to be 0.5m and minimum and maximum rpms are given to be 5 and 10 rad/sec.
The dimensions of the robot is same as mentioned in the turtebot handout.
The 0,0 is at the center of the v-rep platform and the map provided.
When the code is run, you have to enter the initial X, Y in meters and Initial theta in degrees, Final X, Y all in different line(i.e., press enter after each entry)
After the code is executed(i.e., when all the possible nodes are explored and path is generated), it will ask you if you want to see the path and node plotting,
if yes press y else press n. If pressed y the ypu have to click the cross button to start the simulation.

The initial coordinates have to be given manually in the vrep.
This program also prints a 1000 counter for every 1000 nodes explored.
The videos given are for (-4,-4) to (1,4) , (-4,1), (1,1), and (0,4). The shortes are (4,4) to (-4,1), and (4,4) to (0,4) and others are long interms of finding nodes.

The sequence of the program is such that:
First after the entry from the user it start computing the nodes and path.
After that it asks whether you want to see the plot the nodes generated and the path.
If y is entered, after the plotting is shown you have to cross it to start the simulation.
If n is entered, the simulation will start immediately.
