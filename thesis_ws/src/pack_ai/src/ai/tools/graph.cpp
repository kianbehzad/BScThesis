//
// Created by Kian Behzad on 3/19/21.
//

#include "pack_ai/ai/tools/graph.h"

Graph::Graph(const std::vector<rcsc::Vector2D>& _vertices, const std::vector<Edge>& _edges)
{
    vertices = _vertices;
    edges = _edges;
    remove_repeated_edges();
}

Graph::Graph(const std::vector<rcsc::Vector2D>& _vertices) : Graph(_vertices, std::vector<Edge>{})
{

}

Graph::Graph() : Graph(std::vector<rcsc::Vector2D>{}, std::vector<Edge>{})
{

}

bool Graph::check_graph_validity(const unsigned int& _vertices_num, const std::vector<Edge>& _edges)
{
    if (_vertices_num <= 0)
        return false;
    for (const auto& edge : _edges)
    {
        if (!(edge.first >= 0 && edge.first < _vertices_num))
            return false;
        if (!(edge.second >= 0 && edge.second < _vertices_num))
            return false;
        if (edge.first == edge.second)
            return false;
    }
    return true;
}

bool Graph::is_valid() const
{
    return check_graph_validity(vertices_num(), edges);;
}

unsigned int Graph::vertices_num() const
{
    return vertices.size();
}

std::vector<Edge> Graph::get_edges()
{
    return edges;
}

std::vector<rcsc::Vector2D> Graph::get_vertices()
{
    return vertices;
}

void Graph::add_vertex(const rcsc::Vector2D& vertex)
{
    vertices.push_back(vertex);
}

void Graph::add_vertices(const std::vector<rcsc::Vector2D>& _vertices)
{
    for (const auto& v : _vertices)
        add_vertex(v);
}

bool Graph::add_edge(const Edge& edge)
{
    std::vector<Edge> tmp = edges;
    tmp.push_back(edge);

    bool check = check_graph_validity(vertices_num(), tmp);
    if (!check)
        return false;

    edges.push_back(edge);
    remove_repeated_edges();
    return true;
}

bool Graph::add_edge(const unsigned int& first_vertex, const unsigned int& second_vertex)
{
    return add_edge(Edge{first_vertex, second_vertex});
}

void Graph::remove_repeated_edges()
{
    edges.erase( unique( edges.begin(), edges.end() ), edges.end() );
}

bool Graph::are_neighbors(const unsigned int& vertex1, const unsigned int& vertex2)
{
    for (const auto& edge : edges)
        if (edge.first == vertex1 && edge.second == vertex2)
            return true;
        else if (edge.second == vertex1 && edge.first == vertex2)
            return true;

    return false;
}

std::vector<unsigned int> Graph::get_neighbors(const unsigned int& vertex_number)
{
    std::vector<unsigned int> neighbors;

    for (int i{}; i < vertices.size(); i++)
        if (are_neighbors(vertex_number, i))
            neighbors.push_back(i);

    return neighbors;
}

double Graph::get_dist(const unsigned int& vertex1, const unsigned int& vertex2)
{
    if (vertex1 > vertices_num() || vertex2 > vertices_num())
        return 0;
    return vertices[vertex1].dist(vertices[vertex2]);
}

bool Graph::get_dist(const unsigned int& vertex1, const unsigned int& vertex2, double& dist)
{
    dist = get_dist(vertex1, vertex2);
    return are_neighbors(vertex1, vertex2);
}
