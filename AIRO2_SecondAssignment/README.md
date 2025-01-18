Artificial Intelligence for Robotics II
===================================================
Task Motion Planning
=============================================================

Project
-------------------

The aim of this project is to create a planning domain for a robot operating in an environment consisting of four regions where students are working on their assignment and a submission desk. The robot has to collect two reports from two randomly selected waypoints in the environment and deliver them to the submission desk. The primary objective is to minimise the motion cost, which refers to the length of the path travelled by the robot. To achieve this we used the popf-tif planner.

The detailed description of the project can be found [here](Assignment2.pdf).

How to Run the Simulation
-------------------

To begin, please refer to the following GitHub repository for the installation of the [popf-tif planner](https://github.com/popftif/popf-tif). Once you have accessed the repository, proceed with installing the planner.

After installing the popf-tif planner, proceed by cloning the repository:

```bash
git clone https://github.com/claudioDema99/AIRO2_SecondAssignment.git
```

You can already find the planner inside the `visit_domain` folder. 

**Note:** If needed, change the permissions of **popf3-clp** by using the following command in the terminal: 

```bash
chmod +x popf3-clp
```

To run the entire project, simply navigate to the `visit_domain` folder and enter the following command in the terminal:

```bash
./popf3-clp -x dom1.pddl prob1.pddl ../visits_module/build/libVisits.so region_poses
```





