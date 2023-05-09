// graph.h <Starter Code>
// < BRANDON KIM >
//
// Basic graph class using adjacency matrix representation.  Currently
// limited to a graph with at most 100 vertices.
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

//Reminders:
//0). implementing adjacency list
//1). no linear search
//2). If you decide to dynamically-allocate memory, add a destructor. 
// You’ll also need to add a copy constructor and operator= to properly make deep copies.
//3). reimplement all public functions

#pragma once

#include <iostream>

#include <stdexcept>

#include <vector>

#include <set>

#include <map>

#include <unordered_map>

#include <unordered_set>

#include <algorithm>

using namespace std;
//graph<string,int> g is the declaration
template < typename VertexT, typename WeightT >
  class graph {
    private: // - can add functions and variables to this

      int numberOfVertices;
    int numberOfEdges;

    // struct edgeNODE {
    //   VertexT value;
    //   WeightT weight;
    //   edgeNODE* edgeNext;
    // }
    unordered_map < VertexT,
    map < VertexT,
    WeightT >> umap;
    //Format: <VERTEX1,{VERTEX2,WEIGHT}>. If vertex1 has an edge to vertex2, directed graph
    //Map includes vertices that are adjacent to node (key)

    //VertexT values for the key, and a map of the vertices with their weights between them

    // struct EdgeData {
    //   bool     EdgeExists;
    //   WeightT  Weight;

    //   EdgeData() {
    //     EdgeExists = false;  // initially no edge, and no weight
    //   }
    // }; Delete this

    //
    // We are using adjacency matrix implementation, where rows
    // are the starting vertex and cols are the ending vertex.
    // We keep track of the vertices in the Vertices vector,
    // where the vertex's position in the vector --- 0, 1, 2,
    // 3, 4, 5, ... --- denotes the row in the adjacency matrix
    // where their edges are found.  Example: if vertex "ORD" is
    // in position 1 of the Vertices vector, then row 1 of
    // AdjMatrix are the edges that start at "ORD" and lead to
    // other vertices.
    //
    // static constexpr int MatrixSize = 100; delete this

    // EdgeData         AdjMatrix[MatrixSize][MatrixSize]; DELETE THIS
    vector < VertexT > Vertices; //can delete this if necessary

    //
    // _LookupVertex
    //
    // Finds the vertex in the Vertices vector and returns it's
    // index position if found, otherwise returns -1.
    //
    bool foundVertex(VertexT v) const {
      if (umap.find(v) == umap.end())
        return false;
      else
        return true;
    }

    public:
      //
      // constructor:
      //
      // Constructs an empty graph where n is the max # of vertices
      // you expect the graph to contain.
      //
      // NOTE: the graph is implemented using an adjacency matrix.
      // If n exceeds the dimensions of this matrix, an exception
      // will be thrown to let you know that this implementation
      // will not suffice.
      //

      //graph default constructor
      graph() {
        numberOfEdges = 0;
        numberOfVertices = 0;

      }

    //
    // NumVertices
    //
    // Returns the # of vertices currently in the graph.
    //
    int NumVertices() const {
      return numberOfVertices;
    }

    //
    // NumEdges
    //
    // Returns the # of edges currently in the graph.
    //
    int NumEdges() const {
      return numberOfEdges;
    }

    //
    // addVertex
    //
    // Adds the vertex v to the graph if there's room, and if so
    // returns true.  If the vertex already
    // exists in the graph, then false is returned.
    //
    bool addVertex(VertexT v) {

      //
      // is the vertex already in the graph?  If so, we do not
      // insert again otherwise Vertices may fill with duplicates:
      //
      if (foundVertex(v)) {
        return false;
      }

      //
      // if we get here, vertex does not exist so insert.  Where
      // we insert becomes the rows and col position for this
      // vertex in the adjacency matrix.
      //
      umap[v] = {}; //empty map initially for a new node
      numberOfVertices++;

      return true;
    }

    //
    // addEdge
    //
    // Adds the edge (from, to, weight) to the graph, and returns
    // true.  If the vertices do not exist or for some reason the
    // graph is full, false is returned.
    //
    // NOTE: if the edge already exists, the existing edge weight
    // is overwritten with the new edge weight.
    //
    bool addEdge(VertexT from, VertexT to, WeightT weight) {

      //it is directed, only add/update in the right direction
      if (!foundVertex(from) || !foundVertex(to)) { //if the vertices don't exist, return false
        return false;
      }
      if (umap.at(from).count(to) == 0) {
        numberOfEdges++;
      }

      umap[from][to] = weight;

      return true;

    }

    //
    // getWeight
    //
    // Returns the weight associated with a given edge.  If
    // the edge exists, the weight is returned via the reference
    // parameter and true is returned.  If the edge does not
    // exist, the weight parameter is unchanged and false is
    // returned.
    //
    bool getWeight(VertexT from, VertexT to, WeightT & weight) const {

      //
      // we need to search the Vertices and find the position
      // of each vertex; this will denote the row and col to
      // access in the adjacency matrix:
      //

      if (!foundVertex(from) || !foundVertex(to)) { //if the vertices don't exist, return false
        return false;
      }

      if (umap.at(from).count(to) == 0) {
        return false;
      }
      weight = umap.at(from).at(to);
      return true;

    }

    //
    // neighbors
    //
    // Returns a set containing the neighbors of v, i.e. all
    // vertices that can be reached from v along one edge.
    // Since a set is returned, the neighbors are returned in
    // sorted order; use foreach to iterate through the set.
    //
    set < VertexT > neighbors(VertexT v) const {
      set < VertexT > S;

      if (!foundVertex(v)) { //if the vertex doesn't exist, return the empty set
        return S;
      }
      //iterate through the map and insert the vertices into set
      for (auto & e: umap.at(v)) {

        S.insert(e.first);
      }
      return S;

    }

    //
    // getVertices
    //
    // Returns a vector containing all the vertices currently in
    // the graph.
    //
    vector < VertexT > getVertices() const {
      vector < VertexT > v1;
      for (auto & v: umap) {
        v1.push_back(v.first);
      }
      return v1;
    }

    //
    // dump
    //
    // Dumps the internal state of the graph for debugging purposes.
    //
    // Example:
    //    graph<string,int>  G(26);
    //    ...
    //    G.dump(cout);  // dump to console
    //
    void dump(ostream & output) const {
      output << "***************************************************" << endl;
      output << "********************* GRAPH ***********************" << endl;

      output << "**Num vertices: " << this -> NumVertices() << endl;
      output << "**Num edges: " << this -> NumEdges() << endl;

      output << endl;
      //A: (A,B,80) (A,C,100), …
      for (auto & v: umap) {
        output << v.first << ": ";
        for (auto & x: v.second) {
          output << "(" << v.first << "," << x.first << "," << x.second << ") ";
        }
        output << endl;
      }
      output << "**************************************************" << endl;
    }
  };