#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "graph.h"
#include <queue>

#include <iomanip>  /*setprecision*/
#include <string>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;
const double INF = numeric_limits<double>::max();
//
// Implement your standard application here
//

class prioritize {
  public:
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {
      return p1.second > p2.second;
    }
    
};

void printArray(map<long long, double> distances) {
  cout << "Printing Distances Array: " << endl;
  for(auto& n : distances) {
    cout << n.first << ":" << n.second << " ";
  }
  cout << endl;
}

void printQ(priority_queue<pair<long long, double>,vector<pair<long long, double>>,prioritize> q) {
  cout << "Printing Queue: " << endl;
  while (! q.empty() ) {
    cout << q.top().first << ":" << q.top().second << endl;
    q.pop();
  } 
}

vector<long long> DijkstraShortestPath(long long startV,graph<long long, double> G,map<long long, double>& distances,map<long long, long long>& predecessors ) {
    cout << "NumVertices: " << G.NumVertices() << endl;
    cout << "NumEdges: " << G.NumEdges() << endl;
    
    vector<long long> visited;
  priority_queue<pair<long long, double>,vector<pair<long long, double>>,prioritize> unvisitedQueue;
  // map<long long, double> distances;
  // map<long long, long long> predecessors;
  long long currV;
  double weight;
  
  for(auto& v : G.getVertices()) {
    distances[v] = INF;
    predecessors[v] = -1;
    unvisitedQueue.push(make_pair(v,distances[v]));

  }
  printQ(unvisitedQueue);
  printArray(distances);
  distances[startV] = 0;
  unvisitedQueue.push(make_pair(startV,0));
  printQ(unvisitedQueue);
  while(!unvisitedQueue.empty()) {
    currV = unvisitedQueue.top().first;
    weight = unvisitedQueue.top().second;
    unvisitedQueue.pop();
    
    if(distances[currV] == INF) {
      break;
    } else if(std::find(visited.begin(), visited.end(),currV)!=visited.end()) {
      continue;
    } else {
      visited.push_back(currV);
    }
    // cout << "currV: " << currV << endl;
    for(auto& adjv : G.neighbors(currV)) {//each vertex adjacent to currV
      double edgeWeight;
      double alternativePathDistance;
      if (G.getWeight(currV,adjv,edgeWeight)) {
        alternativePathDistance = distances[currV] + edgeWeight;
      }
      if(alternativePathDistance < distances[adjv]) {
        distances[adjv] = alternativePathDistance;
        predecessors[adjv] = currV;
        unvisitedQueue.push(make_pair(adjv,alternativePathDistance));
      }
    }

  }

  return visited;
}


int main() {

    graph<int,int> g1;
    // ostream output1;
    g1.addVertex(5);
    g1.addVertex(6);
    g1.addVertex(4);
    g1.addEdge(5,6,100);
    g1.addEdge(5,5,0);
    graph<long long, double> G;
    G.addVertex(0.0);//G
    G.addVertex(1.0);//E
    G.addVertex(2.0);//C
    G.addVertex(3.0);//A
    G.addVertex(4.0);//B
    G.addVertex(5.0);//D
    G.addVertex(6.0);//F
    G.addVertex(7.0);//H
    G.addEdge(0.0,1.0,100);
    G.addEdge(0.0,3.0,12);
    G.addEdge(0.0,2.0,125);
    G.addEdge(1.0,0.0,80);
    G.addEdge(1.0,2.0,20);
    G.addEdge(1.0,5.0,30);
    G.addEdge(1.0,7.0,60);
    G.addEdge(2.0,5.0,7);
    G.addEdge(3.0,4.0,115);
    G.addEdge(4.0,5.0,1);
    G.addEdge(5.0,7.0,37);
    G.addEdge(5.0,6.0,27);
    G.addEdge(6.0,5.0,27);
    G.addEdge(6.0,7.0,20);

    long long startV = 0.0;
    map<long long, double> distances;
    map<long long, long long> predecessors;
    vector<long long> vis = DijkstraShortestPath(startV,G,distances,predecessors);
    printArray(distances);
    return 0;
}