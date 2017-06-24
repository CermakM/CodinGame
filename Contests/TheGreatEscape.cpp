#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <deque>
#include <initializer_list>
#include <limits>
#include <algorithm>
#include <bitset>
#include <valarray>

using std::cin; using std::cerr; using std::cout; using std::endl;

template<class T>
bool inRange(const T& r_min, const T& r_max, std::initializer_list<T> args) {
    bool allOf = true;
    for (auto const& value : args)
        allOf &= (value >= r_min && value <= r_max);
    return allOf;
}

template<class T>
bool contains(const std::vector<T> & vec, const T& value) {
    for (auto & e : vec)
        if (e == value)
            return true;
    return false;
}
template<class T>
bool contains(const std::deque<T> & vec, const T& value) {
    for (auto & e : vec)
        if (e == value)
            return true;
    return false;
}

template<class T>
bool match(const T& x, const T& y, const T& xx, const T& yy) {
    return x == xx && y == yy;
}

template<class T>
double distance(T& x1, T& y1, T& x2, T& y2) {
    return sqrt( pow(x2 - x1, 2) + pow(y2 - y1, 2) );
}

struct Node;

struct Dragon {
    Dragon(const int & id_);

    int id;
    int x = -2, y = -2;
    int wall_count;
    // TODO: int utility = 0; // utility of the dragon is the size of area that leads to a goal
    double g_dist = std::numeric_limits<double>::infinity();
    std::deque<Node*> g_nodes;
    std::array<int, 2> target; // [x,y] coordinates of the target - then target[direction] gives the right one
    int direction; // 0 - horizontal, 1 - vertical
    int increment = 1; // 1 or -1 for right/left and down/up direction
};

struct Node {
    Node( const int& x_, const int& y_);

    int x, y;
    std::bitset<4> mask; // 1111 indicates free movements - left up down right
    std::unordered_map<int, Dragon*> dragons;
    std::vector<double> steps; // Steps to reach this node
    std::vector<double> g_dist;
    Node* previous = nullptr;
};

using Desk = std::vector< std::vector<Node> >;
struct Field {
    Field (const int& w, const int& h);
    Node* operator() (const int& x, const int& y);

	bool WallFinder(Dragon* const my_dragon, Dragon* const op_dragon);
    std::tuple<double, Node*, char> trackWall(unsigned index, Dragon * const my_dragon, Dragon * const op_dragon);
    bool placeWall(Desk& s_matrix, const int& wallX, const int& wallY, char const & orientation, bool testOn = true);
    void move(Dragon* const dragon, const int & t_x, const int& t_y);
    /* Using backtracking to track subgoals */
    void backtrackPath(Dragon *const dragon, Node* const parent);
    bool trackGoal(Dragon *const dragon, Node *const parent);
    void trackPath(Dragon* const dragon, std::deque<Node*>::iterator start, std::deque<Node*>::iterator goal);
    /* Return adjecent nodes considering walls and directions */
    std::vector<Node*> getNeighbours(Node* const node);

    int width, height;
    Desk matrix;
};

/**
 * Main
**/
int main() {
    int w; // w of the board
    int h; // h of the board
    int playerCount; // number of players (2 or 3)
    int myId; // id of my player (0 = 1st player, 1 = 2nd player, ...)
    cin >> w >> h >> playerCount >> myId;
    cin.ignore();

    Field field(w, h);
    std::vector < Dragon * > p_dragons, d_dragons;
    for (int i = 0; i < playerCount; i++)
        p_dragons.push_back(new Dragon(i));

    Dragon *my_dragon = p_dragons.at(myId);

    /**
     * Game Loop
    **/
    while (1) {
        for (int i = 0; i < playerCount; i++) {
            int x, y, wallsLeft;
            cin >> x >> y >> wallsLeft; cin.ignore();
            if (!p_dragons.at(i)) continue;
            field.move(p_dragons.at(i), x, y);
            if (p_dragons.at(i)->x == -1) {
                d_dragons.push_back(p_dragons.at(i));
                p_dragons.at(i) = nullptr;
                continue;
            }
            p_dragons.at(i)->wall_count = wallsLeft;
            p_dragons.at(i)->g_dist = std::numeric_limits<double>::infinity();
        }
        int wallCount; // number of walls on the board
        cin >> wallCount;
        cin.ignore();
        for (int i = 0; i < wallCount; i++) {
            int wallX, wallY;
            char wallOrientation; // wall orientation ('H' or 'V')
            cin >> wallX >> wallY >> wallOrientation;
            cin.ignore();
            field.placeWall(field.matrix, wallX, wallY, wallOrientation);
        }

        // Compute the goal distance for each dragon; update node utilities
        // Based on the dragon goal distance (TODO: based on dangerous area) , estimate the most dangerous dragon and area
        Dragon *danger_dragon = my_dragon;
        for (Dragon *const dragon : p_dragons) {
            if (!dragon) continue;
            Node *dragon_node = field(dragon->x, dragon->y);
            dragon->g_nodes.clear(); // TODO: maybe check if a wall has changed the walkthrough of any of the nodes instead?
            dragon->g_nodes = std::deque<Node*> ();
            if (my_dragon->wall_count || dragon==my_dragon) {
                field.backtrackPath(dragon, field(dragon->x, dragon->y));
                cerr << "Dragon " << dragon->id << " goal nodes: " << "\t";
                for (Node* const node : dragon->g_nodes)
                    cerr << node->x << node->y << " " ;
                cerr << endl;
                
                field.trackPath(dragon, dragon->g_nodes.begin(), dragon->g_nodes.begin() + 1);
                cerr << "Dragon " << dragon->id << " final goal nodes:  ";
                for (Node* const node : dragon->g_nodes)
                    cerr << node->x << node->y << " " ;
                cerr << endl;
            }
            cerr << " Dragons g_dists: " << dragon->g_dist << " < " << my_dragon->g_dist << endl;
            if ( (dragon->g_dist < danger_dragon->g_dist) || (dragon->g_dist == danger_dragon->g_dist && dragon->id < danger_dragon->id))
                danger_dragon = dragon;

        }

        // Estimate danger node where the danger dragon moves next
        cerr << "Danger dragon: " << danger_dragon->id << " : " << danger_dragon->x << danger_dragon->y << endl;

        // Place the wall or move if my utility is higher
        if (danger_dragon != my_dragon) {
            // TODO: Compute the best possible place for the wall
            if (field.WallFinder(my_dragon, danger_dragon))
                continue;
            else danger_dragon = my_dragon; // Else switch back to my dragon and move anyway;
        }
        
        Node *danger_node = danger_dragon->g_nodes.at(1);
        cerr << "Danger node: " << danger_node->x << danger_node->y << endl;
        
        // Output direction
        std::string dir_x[2] = {"LEFT", "RIGHT"};
        std::string dir_y[2] = {"UP",   "DOWN" };
        bool h_move = abs(danger_node->x - danger_dragon->x);
        h_move ? cout << dir_x[danger_node->x > danger_dragon->x] << endl
               : cout << dir_y[danger_node->y > danger_dragon->y] << endl;
    }
}

/**
 * Definitions
**/
Dragon::Dragon(const int& id_) {
    id = id_;
    switch (id) {
        case 0:
            direction = 0; // Horizontal
            increment = 1;
            target = {8, -1};
            break;
        case 1:
            direction = 0; // Horizontal
            increment = -1;
            target = {0, -1};
            break;
        case 2:
            direction = 1; // Vertical
            increment = 1;
            target = {-1, 8};
            break;
        case 3:
            direction = 1; // Vertical
            increment = -1;
            target = {-1, 0};
            break;
    }
}


//

Field::Field(const int& w, const int& h) {
    width = w; height = h;
    for (int row = 0; row < h; ++row) {
        matrix.push_back(std::vector<Node>());
        for (int line = 0; line < w; line++) {
            matrix[row].push_back(Node(line, row));
        }
    }
}

Node* Field::operator() (const int& x, const int& y) {
    if (x >= 0 && x < width && y >= 0 && y < height)
        return &matrix[y][x];

    return nullptr;
}

bool Field::WallFinder(Dragon* const my_dragon, Dragon* const op_dragon) {

	std::tuple<double, Node*, char> wall {std::numeric_limits<double>::infinity(), nullptr, 0};
	for (int i = 0; i < op_dragon->g_nodes.size() -2; i++) {
		std::tuple<double, Node*, char> temp_wall = trackWall(i, my_dragon, op_dragon);
		Node* wall_node = std::get<1>(temp_wall);
		if (!wall_node) continue;
		cerr << " current utility for wall: " << wall_node->x << wall_node->y << std::get<2>(temp_wall) << " is: " << std::get<0>(temp_wall) << " max util: " << std::get<0>(wall) << endl;
		if ( std::get<0> (temp_wall) < std::get<0>(wall) ) {
			wall = temp_wall;
		}
	}

	Node* target_node = std::get<1>(wall);
	if (!target_node) return false;

	placeWall(matrix, target_node->x, target_node->y, std::get<2>(wall), false);
	return true;
}

std::tuple<double, Node*, char> Field::trackWall(unsigned index, Dragon * const my_dragon, Dragon* const op_dragon) {
    cerr << "Tracking wall position for dragon : " << op_dragon->id << endl;

    // Base condition
    if (op_dragon->g_nodes.size() - 2 <= index) { 
    	return std::make_tuple(std::numeric_limits<double>::infinity(), nullptr, 0); 
    }

    // Create test matrix
    Desk t_matrix(matrix);

    // Create test dragons 
    Dragon m_dragon = *my_dragon;
    Dragon o_dragon = *op_dragon;

    m_dragon.g_nodes.clear(); o_dragon.g_nodes.clear();
    // Make copies of the goal vectors and link them with the test matrix
    for (int i = 0; i < op_dragon->g_nodes.size(); i++) {
        Node *node = op_dragon->g_nodes[i];
        o_dragon.g_nodes.push_back(&t_matrix[node->y][node->x]);
    }
    for (int i = 0; i < my_dragon->g_nodes.size(); i++ ) {
        Node *node = my_dragon->g_nodes[i];
        m_dragon.g_nodes.push_back(&t_matrix[node->y][node->x]);
    }

    Node* node = o_dragon.g_nodes.at(index);
    Node* next_node = o_dragon.g_nodes.at(index+1);
        
   // Find out dragon direction to the next node
    std::valarray<char> orientation_arr  {'V', 'H', 'H', 'V'};
    std::valarray<bool> mask {node->x > next_node->x, node->y > next_node->y, node->y < next_node->y, node->x < next_node->x};
    orientation_arr = orientation_arr[mask];
    char orientation;
    for (int i = 0; i < orientation_arr.size(); i++)
        if (orientation = orientation_arr[i]) break;

    // test wall and check how the wall affects op_dragon
	// set up diffs for multiple checks
	int y_offset = abs(next_node->x - node->x);
	int x_offset = abs(next_node->y - node->y);
	Node* wall_node = nullptr;
	if (placeWall(t_matrix, node->x, node->y, orientation)){
		wall_node = node;
	}	
	else if (placeWall(t_matrix, node->x - x_offset, node->y - y_offset, orientation)){
		wall_node = &t_matrix[node->y - y_offset][node->x - x_offset];
	}	

	if (!wall_node)	return std::make_tuple(std::numeric_limits<double>::infinity(), nullptr, 0);

	trackPath(&o_dragon, o_dragon.g_nodes.begin() + index, o_dragon.g_nodes.begin() + index + 1);

    // Check if the wall intercepts with my_dragon
    if (contains<Node*> (m_dragon.g_nodes, node) || contains<Node*> (m_dragon.g_nodes, wall_node)) {
    	trackPath(&m_dragon, m_dragon.g_nodes.begin() + index, m_dragon.g_nodes.begin() + index + 1);
    }
    
    // DEBUG
    cerr << " Dragon " << o_dragon.id << " new goal nodes:  ";
    for (Node* const node : o_dragon.g_nodes)
        cerr << node->x << node->y << " " ;
    cerr << endl;
    
    // Minimize the effect on my dragon ~ maximize utility of the wall
	return std::make_tuple(o_dragon.g_dist - m_dragon.g_dist, wall_node, orientation);
}

// TODO: Optimize algorithm for placing walls
bool Field::placeWall(Desk& s_matrix, const int& wallX, const int& wallY, char const & orientation, bool testOn) {
    if (orientation == 'V') {
        if (!inRange<int>(0, s_matrix.size()-1, {wallX-1, wallY+1})) return false;
        if (!s_matrix[wallY][wallX].mask[3] || !s_matrix[wallY+1][wallX].mask[3]) return false;
        s_matrix[wallY][wallX].mask.flip(3);
        s_matrix[wallY+1][wallX].mask.flip(3);
        s_matrix[wallY][wallX-1].mask.flip(0);
        s_matrix[wallY+1][wallX-1].mask.flip(0);
    }
    else if (orientation == 'H') {
        if (!inRange<int>(0, s_matrix.size()-1, {wallX+1, wallY-1})) return false;
        if (!s_matrix[wallY][wallX].mask[2] || !s_matrix[wallY][wallX+1].mask[2]) return false;
        s_matrix[wallY][wallX].mask.flip(2);
        s_matrix[wallY][wallX+1].mask.flip(2);
        s_matrix[wallY-1][wallX].mask.flip(1);
        s_matrix[wallY-1][wallX+1].mask.flip(1);
    }

    if (!testOn) cout << wallX << " " << wallY << " " << orientation << endl;

    return true;
}

void Field::backtrackPath(Dragon *const dragon, Node* const parent) {

    // Get the whole line
    std::vector<Node*> line;
    if (dragon->id < 2) {
        for (int y = 0; y < height; y++) {
            line.push_back(&matrix[y][parent->x]);
        }
    }
    else
        for (auto & node : matrix[parent->y])
            line.push_back(&node);

    // Sort it by distance from the parent && availability
    std::sort(line.begin(), line.end(), [parent, this] (Node* const n1, Node* const n2) {
        int diff = distance<int>(n1->x, n1->y, parent->x, parent->y) - distance<int>(n2->x, n2->y, parent->x, parent->y);
        if (diff) return diff < 0;
        else return n1->mask.count() > n2->mask.count(); // else sort by potential of the node
    });

    for ( Node* const node : line) {
        Node* next_node = dragon->id < 2 ? &matrix[node->y][node->x + dragon->increment]
                                         : &matrix[node->y + dragon->increment][node->x];
        if ( contains<Node*>(getNeighbours(node), next_node) ) {
            if (trackGoal(dragon, next_node)) {
                node->g_dist.at(dragon->id) = next_node->g_dist.at(dragon->id) + distance(node->x, node->y, next_node->x, next_node->y);
                dragon->g_nodes.push_front(node);
                if ( node != parent) {
                    parent->g_dist.at(dragon->id) = node->g_dist.at(dragon->id) + distance(parent->x, parent->y, node->x, node->y);
                    dragon->g_dist = parent->g_dist.at(dragon->id);
                    // Finally push the dragon node to the front
                    dragon->g_nodes.push_front(parent);
                }
                else
                    dragon->g_dist = node->g_dist.at(dragon->id);

                return;
            }
        }
    }
}

bool Field::trackGoal(Dragon *const dragon, Node *const parent) {
    std::array<int,2>* g_c = &dragon->target;
    int p_c[2] = { parent->x, parent->y };
    // cerr << "Dragon direction " << dragon->direction << endl;
    if (p_c[dragon->direction] == g_c->at(dragon->direction)) {
        dragon->g_dist = 0;
        parent->g_dist.at(dragon->id) = 0;
        dragon->g_nodes.push_front(parent);
        return true; // Goal found
    }
    // Get the whole line
    std::vector<Node*> line;
    line.push_back(parent);
    if (dragon->id < 2) {
        // Nodes in one direction if not blocked
        for (int y = parent->y+1; y < height; y++) {
            Node* node_up = this->operator()(parent->x, y);
            if (!node_up->mask[2]) break;
            if (node_up) line.push_back(node_up);
        }
        // Nodes in second direction if not blocked
        for (int y = parent->y-1; y >= 0; y--) {
            Node* node_down = this->operator()(parent->x, y);
            if (!node_down->mask[1]) break;
            if (node_down) line.push_back(node_down);
        }
    }
    else
    {
        // Nodes in one direction if not blocked
        for (int x = parent->x+1; x < width; x++) {
            Node* node_right = this->operator()(x, parent->y);
            if (!node_right->mask[3]) break;
            if (node_right) line.push_back(node_right);
        }
        // Nodes in second direction if not blocked
        for (int x = parent->x-1; x >= 0; x--) {
            Node* node_left = this->operator()(x, parent->y);
            if (!node_left->mask[0]) break;
            if (node_left) line.push_back(node_left);
        }
    }

    // Sort it by distance from the parent && availability
    std::sort(line.begin(), line.end(), [parent, this] (Node* const n1, Node* const n2) {
        int diff = distance<int>(n1->x, n1->y, parent->x, parent->y) - distance<int>(n2->x, n2->y, parent->x, parent->y);
        if (diff) return diff < 0;
        else return n1->mask.count() > n2->mask.count(); // else sort by potential of the node
    });
    // for (auto & node : line)
    //     cerr << node->x << node->y << " " ;
    // cerr << endl;

    for ( Node* const node : line) {
        Node* next_node = dragon->id < 2 ? &matrix[node->y][node->x + dragon->increment]
                                         : &matrix[node->y + dragon->increment][node->x];
        if ( contains<Node*>(getNeighbours(node), next_node) ) {
            if (trackGoal(dragon, next_node)) {
                node->g_dist.at(dragon->id) = next_node->g_dist.at(dragon->id) + distance(node->x, node->y, next_node->x, next_node->y);
                dragon->g_nodes.push_front(node);
                if ( node != parent) {
                    parent->g_dist.at(dragon->id) = node->g_dist.at(dragon->id) + distance(parent->x, parent->y, node->x, node->y);
                    dragon->g_dist = parent->g_dist.at(dragon->id);
                }
                else
                    dragon->g_dist = node->g_dist.at(dragon->id);

                return true;
            }
        }
    }
    return false;
}

/* Tracks nodes until the goal node is found */
// TODO: Pathfinding algorithm - DFS
void Field::trackPath(Dragon* const dragon, std::deque<Node*>::iterator start, std::deque<Node*>::iterator goal) {
    std::deque<Node*> deque;
    std::vector<Node*> closed, visited;

    Node* parent = *start;
    parent->previous = nullptr;
    if ( parent->steps.at(dragon->id) == std::numeric_limits<double>::infinity()) parent->steps.at(dragon->id) = 0;
    visited.push_back(parent);
    deque.push_back(parent);
    while (!deque.empty()) {
        parent = deque.front(); deque.pop_front();
        // cerr << "Opening node : " << parent->x << parent->y << endl;
        // Moving goal node if not goal node specified
//        if (!goal_node) goal_node = dragon->id > 2 ? this->operator()(parent->x, (int)g_c->at(dragon->direction))
//                                                   : this->operator()((int)g_c->at(dragon->direction), parent->y);
        if (parent == *goal) {
            cerr << "GOAL reached" << endl;
            break; // Goal node found;
        }
        std::deque<Node*>::iterator head = deque.begin();
        for (Node* const node : getNeighbours(parent)) {
            // cerr << " Checking node : " << node->x << node->y  << endl;
            node->g_dist.at(dragon->id) = distance<int>((*goal)->x, (*goal)->y, node->x, node->y);
            // cerr << node->g_dist.at(dragon->id) << " <= " << parent->g_dist.at(dragon->id) << endl;
            // cerr << parent->steps.at(dragon->id) +1 << " < " << node->steps.at(dragon->id) << endl;
            if ( !contains<Node*> (closed, node) && (parent->steps.at(dragon->id) + 1 < node->steps.at(dragon->id))) {
                // cerr << "enqueing ... " << endl;
                node->steps.at(dragon->id) = parent->steps.at(dragon->id) + 1;
                node->previous = parent;
                deque.push_front(node);
                visited.push_back(node);
            }
            // else cerr << "not suitable" << endl;
        }
        // Sort newly inserted nodes by goal distance
        std::sort(deque.begin(), head, [dragon](Node* n1, Node* n2) {
            return n1->g_dist.at(dragon->id) < n2->g_dist.at(dragon->id);
        });

        closed.push_back(parent);
    }

    // Backtrack nodes between start and goal
    std::deque<Node*> subgoals;
    while ( parent = parent->previous) {
        parent->g_dist.at(dragon->id) += (*goal)->g_dist.at(dragon->id);
        subgoals.push_front(parent);
    }

    // Discard the initial node to avoid duplicity
    subgoals.pop_front();
    dragon->g_nodes.insert(goal, subgoals.begin(), subgoals.end());

    // Clean up all nodes
    for ( Node * const node : visited)
        node->steps.at(dragon->id) = std::numeric_limits<double>::infinity();
}

std::vector<Node*> Field::getNeighbours(Node* const node) {
    std::vector<Node*> vec;
    const int L_LIM = 0, H_LIM = matrix.size() - 1;
    for (int i = 0; i < node->mask.size(); i++) {
        if (node->mask[i]) {
            if (i == 0 && inRange<int>(L_LIM, H_LIM, {node->x+1}))
                vec.push_back(&matrix[node->y][node->x + 1]);
            else if (i == 2 && inRange<int>(L_LIM, H_LIM, {node->y-1}))
                vec.push_back(&matrix[node->y - 1][node->x]);
            else if (i == 1 && inRange<int>(L_LIM, H_LIM, {node->y+1}))
                vec.push_back(&matrix[node->y + 1][node->x]);
            else if (i == 3 && inRange<int>(L_LIM, H_LIM, {node->x-1}))
                vec.push_back(&matrix[node->y][node->x - 1]);
        }
    }
    return vec;
}

void Field::move(Dragon* const dragon, const int & t_x, const int& t_y) {
// 	if (matrix[dragon->y][dragon->x].dragons.count(dragon->id))
    if(dragon->x > 0) matrix[dragon->y][dragon->x].dragons.erase(dragon->id);
    dragon->x = t_x;
    dragon->y = t_y;
    if(dragon->x == -1) return;
    matrix[t_y][t_x].dragons[dragon->id] = dragon;
}

//

Node::Node( const int& x_, const int& y_) {
    x = x_;
    y = y_;
    mask.flip(); // Set all bits to 1;
    steps.resize(4, std::numeric_limits<double>::infinity());
    g_dist.resize(4, std::numeric_limits<double>::infinity());
}

//
