// application.cpp <Starter Code>
// <BRANDON KIM>
//
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>

#include <iomanip>  /*setprecision*/

#include <string>

#include <vector>

#include <map>

#include <cstdlib>

#include <cstring>

#include <cassert>

#include <queue>

#include <stack>

#include "tinyxml2.h"

#include "dist.h"

#include "graph.h"

#include "osm.h"


using namespace std;
using namespace tinyxml2;
const double INF = numeric_limits < double > ::max();

class prioritize {
  public: bool operator()(const pair < long long, double > & p1,
    const pair < long long, double > & p2) const {
    return p1.second > p2.second;
  }

};
//Function that prints the fullname and coordinates of a given building
void printDetails(BuildingInfo b) {
  cout << " " << b.Fullname << endl;
  cout << "(" << b.Coords.Lat << ", " << b.Coords.Lon << ")" << endl;
}
//Function that prints the path given a vector of nodes
void printPath(vector < long long > v) {
  cout << "Path: ";
  for (int i = 0; i < v.size(); i++) {
    if (i != v.size() - 1) {
      cout << v[i] << "->";
    } else {
      cout << v[i];
    }

  }
  cout << endl;
}
//Creates a vector of nodes(long long) using the predecessors array and ending vertex from Dijkstra's algorithm
vector < long long > getPath(map < long long, long long > predecessors, long long endVertex) {
  long long currV = endVertex;
  vector < long long > path;
  stack < long long > stack1;
  while (currV != NULL && currV != -1) {
    stack1.push(currV);
    currV = predecessors[currV];
  }
  while (!stack1.empty()) {
    currV = stack1.top();
    stack1.pop();
    path.push_back(currV);
  }
  return path;
}
//Dijkstra's algorithm which calculates the shortest path given a starting node, a graph of nodes
//. This creates a distances map, predecessors map, and visited vector
vector < long long > DijkstraShortestPath(long long startV, graph < long long, double > G, map < long long, double > & distances, map < long long, long long > & predecessors) {
  vector < long long > visited;
  priority_queue < pair < long long, double > , vector < pair < long long, double >> , prioritize > unvisitedQueue;
  long long currV;
  double weight;

  for (auto & v: G.getVertices()) {
    distances[v] = INF;
    predecessors[v] = -1;
    unvisitedQueue.push(make_pair(v, distances.at(v)));

  }

  distances[startV] = 0;
  unvisitedQueue.push(make_pair(startV, 0));

  while (!unvisitedQueue.empty()) {
    currV = unvisitedQueue.top().first;
    weight = unvisitedQueue.top().second;
    unvisitedQueue.pop();

    if (distances[currV] == INF) {
      break;
    } else if (std::find(visited.begin(), visited.end(), currV) != visited.end()) {
      continue;
    } else {
      visited.push_back(currV);
    }

    for (auto & adjv: G.neighbors(currV)) { //each vertex adjacent to currV
      double edgeWeight;
      double alternativePathDistance;
      if (G.getWeight(currV, adjv, edgeWeight)) {
        alternativePathDistance = distances[currV] + edgeWeight;
      }
      if (alternativePathDistance < distances[adjv]) {
        distances[adjv] = alternativePathDistance;
        predecessors[adjv] = currV;
        unvisitedQueue.push(make_pair(adjv, alternativePathDistance));
      }
    }

  }

  return visited;
}
//Given the paths and nodes, this finds the nearest node(which exists in a pathway) to the given Building b
long long findNearestNode(BuildingInfo b, vector < FootwayInfo > Footways, map < long long, Coordinates > Nodes) {
  long long return1;
  double min = INF;
  for (auto & f: Footways) {
    for (auto & n: f.Nodes) {

      double distance = distBetween2Points(Nodes[n].Lat, Nodes[n].Lon, b.Coords.Lat, b.Coords.Lon);
      if (distance < min) {
        return1 = n;
        min = distance;
      }
    }
  }

  return return1;
}
//Function that finds the closest building given the coordinates and list of buildings. 
// Used to calculate the building of the center
BuildingInfo findCenter(Coordinates mid, vector < BuildingInfo > & Buildings) {
  BuildingInfo return1;
  double min = INF;
  for (auto & B: Buildings) {
    double distance = distBetween2Points(mid.Lat, mid.Lon, B.Coords.Lat, B.Coords.Lon);
    if (distance < min) {
      return1 = B;
      min = distance;
    }
  }
  return return1;
}
//Function that searches through the buildings to make sure a building exists for a given query given by the user
BuildingInfo searchBuilding(string query, vector < BuildingInfo > & Buildings, bool & found) {
  BuildingInfo b5;
  for (auto & B: Buildings) {
    if (query == B.Abbrev) {
      found = true;
      return B;
    }
  }

  for (auto & B: Buildings) {

    if (B.Fullname.find(query) != string::npos) {
      found = true;
      return B;
    }
  }
  found = false;
  return b5;
}

void application(
  map < long long, Coordinates > & Nodes, vector < FootwayInfo > & Footways,
  vector < BuildingInfo > & Buildings, graph < long long, double > G) {
  string person1Building, person2Building;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    //
    // TO DO: lookup buildings, find nearest start and dest nodes, find center
    // run Dijkstra's alg from each start, output distances and paths to destination:
    //

    //MILESTONE 7 : SEARCH BUILDINGS 1 and 2, using Buildings vector
    int foundCount = 0;
    bool found = false;
    bool found1 = false;
    BuildingInfo b1;
    BuildingInfo b2;
    b1 = searchBuilding(person1Building, Buildings, found);
    b2 = searchBuilding(person2Building, Buildings, found1);
    if (!found) {
      cout << "Person 1's building not found" << endl;
    } else {
      foundCount++;
    }
    if (!found1) {
      cout << "Person 2's building not found" << endl;
    } else {
      foundCount++;
    }
    int pathNotFound = 2;
    Coordinates cd1;
    Coordinates cd2;
    Coordinates midpoint;
    BuildingInfo center;
    while (pathNotFound > 0 && foundCount == 2) { //while path is not found
      cout << "Person 1's point:" << endl;
      printDetails(b1);
      cout << "Person 2's point:" << endl;
      printDetails(b2);
      cd1 = b1.Coords;
      cd2 = b2.Coords;
      midpoint = centerBetween2Points(cd1.Lat, cd1.Lon, cd2.Lat, cd2.Lon);
      center = findCenter(midpoint, Buildings); //MILESTONE 8 : LOCATE CENTER BUILDING
      long long node1ID = findNearestNode(b1, Footways, Nodes); //find nearest node is off
      long long node2ID = findNearestNode(b2, Footways, Nodes);

      cout << "Destination Building:" << endl;
      printDetails(center);
      cout << endl;

      cout << "Nearest P1 node:" << endl;
      cout << node1ID << endl;
      cout << "(" << Nodes[node1ID].Lat << ", " << Nodes[node1ID].Lon << ")" << endl;
      cout << "Nearest P2 node:" << endl;
      cout << node2ID << endl;
      cout << "(" << Nodes[node2ID].Lat << ", " << Nodes[node2ID].Lon << ")" << endl;
      cout << "Nearest destination node:" << endl;

      long long nodeCenterID = findNearestNode(center, Footways, Nodes);
      //MILESTONE 9 : Find nearest nodes from buildings 1,2,center
      map < long long, double > distances1;
      map < long long, long long > predecessors1;
      vector < long long > pathVector1;
      map < long long, double > distances2;
      map < long long, long long > predecessors2;
      vector < long long > pathVector2;

      vector < long long > pathOne;
      vector < long long > pathTwo;
      pathVector1 = DijkstraShortestPath(node1ID, G, distances1, predecessors1);
      //TODO MILESTONE 10: run dijkstra's algorithm
      pathVector2 = DijkstraShortestPath(node2ID, G, distances2, predecessors2);
      cout << nodeCenterID << endl;
      cout << "(" << Nodes[nodeCenterID].Lat << ", " << Nodes[nodeCenterID].Lon << ")" << endl;
      cout << endl;

      if (distances1[node2ID] >= INF) {
        cout << "Sorry, destination unreachable." << endl;
        break;
      }
      if (distances1[nodeCenterID] >= INF || distances2[nodeCenterID] >= INF) {
        //path not found
        cout << "At least one person was unable to reach the destination building. Finding next closest building" << endl;
        pathNotFound--;
        continue;
      } else {
        pathOne = getPath(predecessors1, nodeCenterID);
        pathTwo = getPath(predecessors2, nodeCenterID);
        cout << "Person 1's distance to dest: " << distances1[nodeCenterID] << " miles" << endl;
        printPath(pathOne);
        cout << endl;
        cout << "Person 2's distance to dest: " << distances2[nodeCenterID] << " miles" << endl;
        printPath(pathTwo);
        break;
      }

    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}

int main() {
  graph < long long, double > G; //graph of footways

  // maps a Node ID to it's coordinates (lat, lon)
  map < long long, Coordinates > Nodes;
  // info about each footway, in no particular order
  vector < FootwayInfo > Footways;
  // info about each building, in no particular order
  vector < BuildingInfo > Buildings;
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int) Nodes.size());
  assert(footwayCount == (int) Footways.size());
  assert(buildingCount == (int) Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  //
  // TO DO: build the graph, output stats:
  //

  // TODO MILESTONE 5 : add vertices

  for (auto & node: Nodes) {
    G.addVertex(node.first);
  }

  // TODO MILESTONE 6 : add edges
  double distance;
  Coordinates c1;
  Coordinates c2;
  for (auto & path: Footways) {
    for (unsigned int i = 0; i < path.Nodes.size() - 1; i++) {
      c1 = Nodes.at(path.Nodes[i]);
      c2 = Nodes.at(path.Nodes[i + 1]);
      distance = distBetween2Points(c1.Lat, c1.Lon, c2.Lat, c2.Lon);
      G.addEdge(c1.ID, c2.ID, distance);
      G.addEdge(c2.ID, c1.ID, distance);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}