//
// Created by Kian Behzad on 3/19/21.
//

#ifndef PACK_AI_GRAPH_H
#define PACK_AI_GRAPH_H

#include <vector>
#include <utility>
#include <algorithm>

#include "pack_util/geom/vector_2d.h"


using Edge = std::pair<unsigned int, unsigned int>;

// implement a graph class
class Graph
{
public:
    // _vertices : each graph has some vertices and each vertex correspond to a position on the field
    // (if the positions are not important invalidate vector2d could be used)
    // _edges : defines the edges of the graph - each edge need 2 vertices specify by the id number of them
    // (for example (1, 3) defines an edge between the vertex nmber one and the vertex number three)
    Graph(const std::vector<rcsc::Vector2D>& _vertices, const std::vector<Edge>& _edges);

    // _vertices : each graph has some vertices and each vertex correspond to a position on the field
    // (if the positions are not important invalidate vector2d could be used)
    // this constructor doesnt define any edges
    Graph(const std::vector<rcsc::Vector2D>& _vertices);

    // an empty graph with no vertices and no edges
    Graph();

    // check the validity of a given graph with its edges and its number of vertices
    static bool check_graph_validity(const unsigned int& _vertices_num, const std::vector<Edge>& _edges);

    // check the validity of this graph
    bool is_valid() const;

    // returns the number of vertices of this graph
    unsigned int vertices_num() const;

    // returns all the edges of this graph
    std::vector<Edge> get_edges();

    // returns all the vertices' positions of this graph
    std::vector<rcsc::Vector2D> get_vertices();

    // add a single vertex to the graph
    // (if the position is not important invalidate vector2d could be used)
    void add_vertex(const rcsc::Vector2D& vertex);

    // add a vector of vertices to the graph
    // (if the position are not important invalidate vector2d could be used)
    void add_vertices(const std::vector<rcsc::Vector2D>& _vertices);

    // add a new edge between 2 vertices
    // (for example (1, 3) defines an edge between the vertex nmber one and the vertex number three)
    bool add_edge(const Edge& edge);

    // add a new edge between 2 vertices
    // (for example (1, 3) defines an edge between the vertex nmber one and the vertex number three)
    bool add_edge(const unsigned int& first_vertex, const unsigned int& second_vertex);




private:
    std::vector<rcsc::Vector2D> vertices;
    std::vector<Edge> edges;

    // removes duplicated edges (if there were any)
    void remove_repeated_edges();
};

#endif //PACK_AI_GRAPH_H
