    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <random>
#include <iomanip>

#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){}

VisitSolver::~VisitSolver(){}

void VisitSolver::loadSolver(string *parameters, int n){
  // Initialize the starting position of the robot as r0 (0,0)
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  // Let the user enter a valid number of links k
  do {
        std::cout << "Please insert the number of links between nodes (the number should be between 5 and 30): " << std::endl;
        std::cin >> k;
        std::cout << std::endl;
        
        if (k < 5 || k > 30) {
            std::cout << "Error! Please insert a value between 5 and 30" << std::endl;
        }
        
    } while (k < 5 || k > 30);
    
    std::cout << "Valid value entered: " << k << std::endl;

    string waypoint_file = "./waypoint.txt";
    // Generate random waypoints 
    randWaypointGenerator(waypoint_file);
    // Parse the waypoint file                            
    parseWaypoint(waypoint_file);          
    // Build the graph
    buildGraph();                                        
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;

  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);
      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

          string from = tmp.substr(0,2); 
          string to = tmp.substr(3,2);

          // The cost is equal to the total path covered by the robot
          act_cost = compute_path(from, to);
          if (act_cost >= 1000.0){
            act_cost = -1000.0;
          }
        }
      }
    }else{

      if(function=="dummy"){
        dummy = act_cost;
      }
      else if(function=="act-cost"){
        act_cost = value;
      } 
    }
  }
  
  double results = calculateExtern(dummy, act_cost);
  if (ExternalSolver::verbose){
    cout << "(dummy) " << results << endl;
  }

  toReturn["(dummy)"] = results;
  return toReturn;
}


list<string> VisitSolver::getParameters(){

  return affected;
}

list<string> VisitSolver::getDependencies(){
  
  return dependencies;
}


void VisitSolver::parseParameters(string parameters){

  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open()){

    while (getline(parametersFile,line)){
      curr=line.find(" ");
      string region_name = line.substr(0,curr).c_str();
      curr=curr+1;

      while(true ){
        next=line.find(" ",curr);
        region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());

        if (next ==-1)
          break;
        curr=next+1;
      }                
    }
  }
}


double VisitSolver::calculateExtern(double external, double total_cost){
  return total_cost;
}

void VisitSolver::parseWaypoint(string waypoint_file){

  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(waypoint_file);
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
      curr=line.find("[");
      string waypoint_name = line.substr(0,curr).c_str();

      curr=curr+1;
      next=line.find(",",curr);

      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find(",",curr);

      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find("]",curr);

      pose3 = (double)atof(line.substr(curr,next-curr).c_str());

      waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
    }
  }
}


// Function that, additionally to the already existing 6 waypoints, generates 24 new random waypoints
void VisitSolver::randWaypointGenerator(string waypoint_file) {
  float waypoints[numWaypoints][numCoordinates];

  ofstream outfile(waypoint_file);

  if (!outfile) {
    cerr << "Error opening file: " << waypoint_file << endl;
    return;
  }

    outfile.clear();     // Clear the contents of the file

    // Write on the file the six waypoints that are already known
    outfile << "wp0[0,0,0]" << std::endl;
    outfile << "wp1[-2.5,2.5,0]" << std::endl;
    outfile << "wp2[2.5,2.5,0]" << std::endl;
    outfile << "wp3[-2.5,-2.5,0]" << std::endl;
    outfile << "wp4[2.5,-2.5,0]" << std::endl;
    outfile << "wp5[3,0,0]" << std::endl;

    std::random_device random;         // Create a random number generator
    std::mt19937 gen(random());        // Seed the generator with a random device 
    std::uniform_real_distribution<> dis(-3.0, 3.0);      // Create a uniform distribution between -3.0 and 3.0

    // Generate random x and y values for the waypoints
    for (int i = 6; i < numWaypoints; i++) {
        for (int j = 0; j < numCoordinates; j++) {
            waypoints[i][j] = std::round(dis(gen) * 100.0) / 100.0;

            if (j == 0) {
              outfile << "wp" << i << "[" << std::fixed << std::setprecision(2) << waypoints[i][j] << ",";
            }
            else if (j == 1) {
                outfile << std::fixed << std::setprecision(2) << waypoints[i][j] << ",";
            } else {
                outfile << 0 << "]" << std::endl;
            }
        }
    }
  outfile.close();
}

// This function builds a graph connecting each waypoint to a maximum of k other waypoints
void VisitSolver::buildGraph() {
  int numConnections[numWaypoints] = {};   // Array to store the number of connections for each waypoint
  double nodeDistances[numWaypoints];      // Array to store the distances from the current waypoint to other waypoints

  // Calculate distances between waypoints and populate the distance matrix
  for (int i = 0; i < numWaypoints; i++) {
    for (int j = 0; j < numWaypoints; j++) {
      // Convert waypoint indices to strings and add prefix 'wp'
      string from = "wp" + to_string(i);
      string to = "wp" + to_string(j);
      if (i != j)
  	    dist_matrix[i][j] = distance_euc(from, to);    // Calculate Euclidean distance between waypoints
      else
        dist_matrix[i][j] = 1000.0;  // Set a high value for elements on the diagonal to avoid interference in finding the minimum distance
    }
  }

  for (int i = 0; i < numWaypoints; i++) {
    std::copy(dist_matrix[i], dist_matrix[i] + numWaypoints, nodeDistances);    // Copy distances for the current waypoint

    for (int j = 0; j < k; j++) {
      int min_dist_idx = findMinimumIndex(nodeDistances);    // Find the index of the waypoint with the minimum distance

      // Connect the current waypoint and the waypoint with the minimum distance if the maximum number of connections is not exceeded
      if (numConnections[i] < k && numConnections[min_dist_idx] < k) {
        adj_matrix[i][min_dist_idx] = dist_matrix[i][min_dist_idx];    // Connect the waypoints in the adjacency matrix
        adj_matrix[min_dist_idx][i] = dist_matrix[min_dist_idx][i];    // Connect the waypoints in the adjacency matrix (bidirectional)
        numConnections[i]++;    // Increment the number of connections for the current waypoint
        numConnections[min_dist_idx]++;    // Increment the number of connections for the waypoint with the minimum distance
      }

      nodeDistances[min_dist_idx] = std::numeric_limits<double>::max();    // Set the distance to the maximum value to exclude it from future selections
    }
  }
}

// Function used to compute the euclidean distance between two waypoints
double VisitSolver::distance_euc(string from, string to)
{
   // Retrieve the x and y coordinates of the 'from' waypoint
   float from_x = waypoint[from][0];
   float from_y = waypoint[from][1];

   // Retrieve the x and y coordinates of the 'to' waypoint
   float to_x = waypoint[to][0];
   float to_y = waypoint[to][1];

   // Compute the squared distance between the waypoints using the Euclidean distance formula
   float distance = pow((from_x - to_x), 2) + pow((from_y - to_y), 2);
   
   // Return the square root of the computed distance
   return sqrt(distance);
}

// Function that implements Dijkstra's shortest path algorithm for a graph represented using adjacency matrix representation
double VisitSolver::compute_path(string from, string to)
{
  // Extract the start and end indices
  int from_idx = extract_num(from);
  int to_idx = extract_num(to);
  //std::cout << "from: " << from_idx << "  to: " << to_idx << std::endl;

  // The output array.  dist[i] will hold the shortest distance from "from_idx" to "i"
  double dist[numWaypoints]; 
  // visited[i] will be true if waypoint "i" is included in shortest path tree or shortest distance from "from_idx" to "i" is finalized
  bool visited[numWaypoints] = {false};
  
  // Distance of "from_idx" waypoint from itself is always 0  
  dist[from_idx] = 0;
 
  for (int i = 0; i < numWaypoints; i++) {
    if (i != from_idx)
      dist[i] = 1000.0;
  }

  // Find shortest path for all waypoints
  for (int i = 0; i < numWaypoints - 1; i++)
  {
    // Pick the minimum distance waypoint from the set of waypoints not yet processed 
    // "u" is always equal to "from_idx" in the first iteration
    int u = minDistance(dist, visited);

    // Mark the picked waypoints as processed
    visited[u] = true;

    // Update dist value of the adjacent waypoints of the picked vertex
    for (int i = 0; i < numWaypoints; i++)                  
    {
      // Update dist[i] only if is not in visited, there is an edge from "u" to "i", and total
      // weight of path from "from_idx" to "i" through "u" is smaller than current value of dist[i]
      if (!visited[i] && adj_matrix[u][i] && dist[u] != 1000.0 && dist[u] + adj_matrix[u][i] < dist[i])
        dist[i] = dist[u] + adj_matrix[u][i];
      }
    }

  return dist[to_idx];
}

// This function finds the index of the minimum element inside the dist_array 
int VisitSolver::findMinimumIndex(double dist_array[]) {
    int minIndex = 0;

    for (int i = 1; i < numWaypoints; i++) {
        if (dist_array[i] < dist_array[minIndex]) {
            minIndex = i;    // Update the minIndex if a smaller element is found
        }
    }

    // Set the minimum element to a large value to mark it as visited
    dist_array[minIndex] = 1000.0;     

    // Return the index of the minimum element
    return minIndex;
}

// This function finds the index of the vertex with the minimum distance value from the set of vertices not yet included in the shortest path tree
int VisitSolver::minDistance(double dist[], bool visited[])
{
    // Initialize min value
    double min = 1000.0;
    int min_index = -1;
    
    // Iterate over all waypoints to find the vertex with the minimum distance value
    for (int i = 0; i < numWaypoints; i++) {
        // Check if the waypoint is not yet visited and its distance is less than or equal to the current minimum distance
        if (visited[i] == false && dist[i] <= min)
          min = dist[i], min_index = i; // Update the minimum distance and the index of the waypoint with the minimum distance
    }
    
    // Return the index of the vertex with the minimum distance value
    return min_index;
}

// This function extracts the numerical digits from the input string and returns the resulting integer value
int VisitSolver::extract_num(string str) {
  string extractedDigits;

  // Iterate over each character in the input string
  for (char c : str) {
    // Check if the character is a digit
    if (isdigit(c)) {
      extractedDigits += c; // If it is a digit, append it to the 'extractedDigits' string
    }
  }

  if (extractedDigits.empty()) {
    return -1; // If 'extractedDigits' is empty, no digits were found, so return -1
  }

  int extractedNum;
  try {
    extractedNum = stoi(extractedDigits); // Convert the extracted digits in 'extractedDigits' to an integer
  } catch (const std::exception& e) {
    // Handle exception if conversion fails
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return -1;
  }

  return extractedNum;
}
