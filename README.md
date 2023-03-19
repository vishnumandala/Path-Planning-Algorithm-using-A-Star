# PROJECT 3

## ENPM661 - Planning for Autonomous Robots
### Hamza Shah Khan: 119483152 | hamzask@umd.edu
### Vishnu Mandala: 119452608 | vishnum@umd.edu

## Path Planning Algorithm
This program implements a path planning algorithm using A* algorithm to find the optimal path from a start node to a goal node in a 2D environment with obstacles. The obstacles are defined using equations, and the map parameters include width, height, radius and clearance.

The program first creates a 2D map of the environment with the obstacles marked as black and the free space marked as white. The boundaries and clearances are marked as gray. The start and goal nodes are also defined on this map.

The program then uses A* algorithm to find the optimal path from the start node to the goal node while avoiding the obstacles. The algorithm is visualized using animation, with each visited node marked in blue and the optimal path marked in red. The final cost of the path is also displayed.

## Contents
1. proj3_hamza_vishnu.pdf
2. a_star_hamza_vishnu.py
3. project3_output.png
4. animation.mp4
5. README.md

## Dependencies

1. python 3.11 (any version above 3 should work)
2. Python running IDE (I used VS Code)

## Libraries

1. NumPy
2. Queue
3. Time
4. OpenCV
5. Math

## Instructions
1. Download the zip file and extract it
2. Install python and the required dependencies: pip install numpy opencv-python
3. Run the code: a_star_hamza_vishnu.py
4. Type in the Clearance Value, Radius of the Robot, Start and Goal Nodes in the format (x y o)
5. The optimal path will be displayed on the screen and saved to a video file named animation.mp4 in the same directory as the program. The final cost of the path will be displayed in the console.

## Example Output

Enter the clearance: 3
Enter the radius: 2
Enter the start node (in the format 'x y o'). O should be given in degrees and as a multiple of 30 i.e. {.., -60, -30, 0, 30, 60, ..}: 35 67 30
Enter the goal node (in the format 'x y o'). O should be given in degrees and as a multiple of 30 i.e. {.., -60, -30, 0, 30, 60, ..}: 530 226 60
Enter the step size of the robot in the range 1-10: 3
Final Cost:  166.0

Goal Node Reached!
Shortest Path:  [(35, 67, 30), (38.0, 69.0, 30), (41.0, 71.0, 30), (44.0, 73.0, 30), (47.0, 75.0, 30), (50.0, 77.0, 30), (53.0, 79.0, 30), (56.0, 81.0, 30), (59.0, 83.0, 30), (62.0, 85.0, 30), (65.0, 87.0, 30), (68.0, 89.0, 30), (71.0, 91.0, 30), (74.0, 93.0, 30), (77.0, 95.0, 30), (80.0, 97.0, 30), (83.0, 99.0, 30), (86.0, 101.0, 30), (89.0, 103.0, 30), (92.0, 105.0, 30), (95.0, 107.0, 30), (98.0, 109.0, 30), (101.0, 111.0, 30), (104.0, 113.0, 30), (107.0, 115.0, 30), (110.0, 117.0, 30), (113.0, 119.0, 30), (116.0, 121.0, 30), (119.0, 123.0, 30), (122.0, 125.0, 30), (125.0, 127.0, 30), (128.0, 129.0, 30), (131.0, 131.0, 30), (134.0, 133.0, 30), (137.0, 135.0, 30), (140.0, 137.0, 30), (143.0, 139.0, 30), (146.0, 141.0, 30), (149.0, 143.0, 30), (152.0, 143.0, 60), (155.0, 145.0, 90), (158.0, 147.0, 150), (161.0, 149.0, 90), (164.0, 151.0, 150), (167.0, 153.0, 90), (170.0, 155.0, 150), (173.0, 157.0, 90), (176.0, 159.0, 150), (179.0, 161.0, 90), (182.0, 163.0, 150), (185.0, 165.0, 90), (188.0, 167.0, 150), (191.0, 169.0, 90), (194.0, 171.0, 150), (197.0, 173.0, 90), (200.0, 175.0, 150), (203.0, 177.0, 90), (206.0, 179.0, 150), (209.0, 181.0, 90), (212.0, 183.0, 150), (215.0, 185.0, 90), (218.0, 187.0, 150), (221.0, 189.0, 90), (224.0, 191.0, 150), (227.0, 193.0, 90), (230.0, 195.0, 150), (233.0, 197.0, 90), (236.0, 199.0, 150), (239.0, 201.0, 90), (242.0, 203.0, 150), (245.0, 205.0, 90), (248.0, 207.0, 150), (251.0, 209.0, 90), (254.0, 211.0, 150), (257.0, 213.0, 90), (260.0, 215.0, 150), (263.0, 217.0, 90), (266.0, 219.0, 150), (269.0, 221.0, 90), (272.0, 223.0, 150), (275.0, 225.0, 90), (278.0, 227.0, 150), (281.0, 227.0, 120), (284.0, 226.0, 90), (287.0, 225.0, 30), (290.0, 225.0, 60), (293.0, 225.0, 120), (296.0, 225.0, 60), (299.0, 225.0, 120), (302.0, 225.0, 60), (305.0, 225.0, 120), (308.0, 225.0, 60), (311.0, 225.0, 120), (314.0, 225.0, 60), (317.0, 225.0, 120), (320.0, 225.0, 60), (323.0, 225.0, 120), (326.0, 225.0, 60), (329.0, 225.0, 120), (332.0, 225.0, 60), (335.0, 225.0, 120), (338.0, 225.0, 60), (341.0, 225.0, 120), (344.0, 225.0, 60), (347.0, 225.0, 120), (350.0, 225.0, 60), (353.0, 225.0, 120), (356.0, 225.0, 60), (359.0, 225.0, 120), (362.0, 225.0, 60), (365.0, 225.0, 120), (368.0, 225.0, 60), (371.0, 225.0, 120), (374.0, 225.0, 60), (377.0, 225.0, 120), (380.0, 225.0, 60), (383.0, 225.0, 120), (386.0, 225.0, 60), (389.0, 225.0, 120), (392.0, 225.0, 60), (395.0, 225.0, 120), (398.0, 225.0, 60), (401.0, 225.0, 120), (404.0, 225.0, 60), (407.0, 225.0, 120), (410.0, 225.0, 60), (413.0, 225.0, 120), (416.0, 225.0, 60), (419.0, 225.0, 120), (422.0, 225.0, 60), (425.0, 225.0, 120), (428.0, 225.0, 60), (431.0, 225.0, 120), (434.0, 225.0, 60), (437.0, 225.0, 120), (440.0, 225.0, 60), (443.0, 225.0, 120), (446.0, 225.0, 60), (449.0, 225.0, 120), (452.0, 225.0, 60), (455.0, 227.0, 90), (458.0, 229.0, 150), (461.0, 229.0, 120), (464.0, 228.0, 90), (467.0, 227.0, 30), (470.0, 226.0, 90), (473.0, 225.0, 30), (476.0, 225.0, 60), (479.0, 225.0, 120), (482.0, 225.0, 60), (485.0, 225.0, 120), (488.0, 225.0, 60), (491.0, 225.0, 120), (494.0, 225.0, 60), (497.0, 225.0, 120), (500.0, 225.0, 60), (503.0, 225.0, 120), (506.0, 225.0, 60), (509.0, 225.0, 120), (512.0, 225.0, 60), (515.0, 225.0, 120), (518.0, 225.0, 60), (521.0, 225.0, 120), (524.0, 225.0, 60), (527.0, 225.0, 120), (530.0, 225.0, 60), (530.0, 225.0, 60)]

Runtime: 4.806797742843628 seconds

## Output Photo
![A Star](https://github.com/vishnumandala/Path-Planning-Algorithm-using-A-Star/blob/main/project3_output.png)

## Animation
https://drive.google.com/file/d/1gEpD78ewm2ewnozhcy5wWfuew5WhLpQ2/view?usp=share_link

