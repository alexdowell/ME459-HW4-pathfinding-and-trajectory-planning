# ME459 HW4: Pathfinding and Trajectory Planning  

## Description  
This repository contains implementations of pathfinding and trajectory planning algorithms for **ME459 Robotics and Unmanned Systems**. The focus is on solving navigation and motion planning problems using **Dijkstra's algorithm, A* search, and Rapidly-exploring Random Tree (RRT)**. The repository includes Python scripts for implementing these algorithms and analyzing their performance in different environments.  

## Files Included  

### **Dijkstra's Algorithm Implementation**  
- **File:** ME_459_Dijkstra.py  
- **Topics Covered:**  
  - Grid-based path planning  
  - Graph search techniques  
  - Cost estimation and shortest path computation  
  - Obstacle avoidance  

### **A* Algorithm Implementation**  
- **File:** ME 459 Astar lab test.py  
- **Topics Covered:**  
  - Heuristic-based path planning  
  - Comparison with Dijkstra's algorithm  
  - Implementation of movement constraints  
  - TurtleBot simulation integration  

### **Rapidly-exploring Random Tree (RRT) Implementation**  
- **File:** ME 459 HW 4_6_test.py  
- **Topics Covered:**  
  - Sampling-based path planning  
  - Tree expansion and goal biasing  
  - Obstacle avoidance using spatial constraints  

### **Homework Problems and Documentation**  
- **File:** ME 459 HW 4.pdf  
  - Description of path planning problems and requirements  
  - Problem statements for Dijkstra's, A*, and RRT implementations  
  - Expected output and analysis  

- **File:** ME 459 HW4 Code #4 & #5.pdf  
  - Code documentation and explanation for Problems 4 and 5  
  - Sample output and performance comparison  

- **File:** ME 459 HW  4 ^N6 .pdf  
  - Problem 6 details on implementing RRT  
  - Instructions for modifying Dijkstra/A* to include RRT  

## Installation  
Ensure Python and the required libraries are installed before running the scripts.  

### **Required Python Packages**  
- numpy  
- matplotlib  
- math  
- rospy (for TurtleBot simulation)  

To install the necessary packages, run:  

```pip install numpy matplotlib```  

## Usage  
1. Open a terminal or Python environment.  
2. Run the desired path planning script using:  

```python ME_459_Dijkstra.py```  
```python ME 459 Astar lab test.py```  
```python ME 459 HW 4_6_test.py```  

3. View generated path plots and analyze results.  

## Example Output  

### **Dijkstra's Algorithm**  
- Computes shortest path through grid-based obstacles  
- Generates x vs. y trajectory plots  

### **A* Algorithm**  
- Implements heuristic cost evaluation  
- Compares search efficiency against Dijkstra  

### **RRT Algorithm**  
- Expands a tree to explore free space  
- Generates a path using sampled waypoints  

## Contributions  
This repository is designed for educational purposes. Feel free to modify and expand upon the existing implementations.  

## License  
This project is open for educational and research use.  

---  

**Author:** Alexander Dowell  

