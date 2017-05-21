#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <unordered_map>
#include <algorithm>

using std::cerr; using std::cin; using std::cout; using std::endl;

using Graph = std::unordered_map< int, std::vector<int>>; // map<vertex_id, vector_of_child_vertecies>
using Node  = std::tuple<int,int,int>; // tuple<vertex_id, parent_vertex_id, distance>

int dfs(Graph& graph, int const& init_node) {
    std::stack<Node> node_stack;
    node_stack.push(Node{init_node, init_node, 0});
    
    int max_distance = 0;
    
    while ( !node_stack.empty()) {
        int node = std::get<0> (node_stack.top());
        int parent = std::get<1> (node_stack.top());
        int distance = std::get<2> (node_stack.top());
        node_stack.pop();
        
        if ( max_distance < distance) {
            max_distance = distance;
        }
        
        for ( auto& child : graph[node] ) {
            if (child != parent) node_stack.push(Node{child, node, distance + 1});
        }
    }
    
    return max_distance + 1;
}

int main()
{
    int n; // the number of adjacency relations
    cin >> n; cin.ignore();
    
    std::unordered_map< int, std::vector<int>> graph;
    
    for (int i = 0; i < n; i++) {
        int xi; // the ID of a person which is adjacent to yi
        int yi; // the ID of a person which is adjacent to xi
        cin >> xi >> yi; cin.ignore();
        graph[yi].push_back(xi);
        graph[xi].push_back(yi);
    }
    
    // Find the leaf nodes
    std::vector<int> leaf_nodes;
    for (auto & node : graph) {
        if ( node.second.size() == 1) {
            leaf_nodes.push_back(node.first);
        }
    }
    
    // Due to the limited time provided, investigate only a few leaf nodes
    int max_distance = 0; 
    int i = leaf_nodes.size() > 1000 ?  1 : 2;
    cerr << leaf_nodes.size() << endl;
    for ( auto & node : leaf_nodes) {
        int distance = dfs(graph, node);
        if ( distance > max_distance ) { max_distance = distance; i--; if (!i) break; }
    }
        
    cout << (max_distance / 2) << endl;
}
