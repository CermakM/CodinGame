#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

using std::cerr; using std::cin; using std::cout; using std::endl;


/* DECLARATIONS */

class Point {
public: 
    enum PDir { HERE, LEFT, RIGHT, UP, DOWN };
    Point() { x = -1; y = -1; }
    Point(const int& x_, const int& y_) { x = x_; y = y_; }
    Point pointOn(Point::PDir direction); 
    bool isDownFrom(const Point& other) const    { return x == other.x && y > other.y; }
    bool isFarDownFrom(const Point& other) const { return y > other.y; }
    bool isUpFrom(const Point& other) const      { return x == other.x && y < other.y; }
    bool isFarUpFrom(const Point& other) const   { return y < other.y; } 
    bool isRightFrom (const Point& other) const  { return y == other.y && x > other.x; }
    bool isFarRightFrom(const Point& other) const { return x > other.x; }
    bool isLeftFrom(const Point& other) const    { return y == other.y && x < other.x; } 
    bool isFarLeftFrom(const Point& other) const { return x < other.x; }
    bool isHorizontaly(const Point& other ) const { return y == other.y; }
    bool isVerticaly(const Point& other) const   { return x == other.x; }
    PDir whereIs(const Point& other) { if (other.isFarLeftFrom(*this)) return LEFT; if (other.isFarRightFrom(*this)) return RIGHT; if (other.isFarUpFrom(*this)) return UP; if(other.isFarDownFrom(*this)) return DOWN; return HERE;}
    Point operator - (const Point& other) { return Point( x - other.x, y - other.y);}
    Point operator + (const Point& other) { return Point( x + other.x, y + other.y);}
    double distanceTo (const Point& other) const { return sqrt(pow(x - other.x,2 ) + pow(y - other.y, 2)); }
    bool operator < (const Point& other) { return x < other.x || y < other.y; }
    bool operator > (const Point& other) { return x < other.x || y < other.y; }
    bool operator == (const Point& other) const { return (x == other.x) && (y == other.y); 
        
    }
    operator bool() const { return this->inRange(); }
    friend std::ostream& operator << (std::ostream& os, const Point& p) { return os << p.x << " " << p.y; }

    bool inRange(const uint& lLimx = 0, const uint& hLimx = 8, const uint& lLimy = 0, const uint& hLimy = 8) const { return x >= lLimx && x <= hLimx && y >= lLimy && y <= hLimy; }
    int x,y;
};

struct Wall
{
    Wall(const Point& p, const char& orientation_);
    Wall(const uint& x_, const uint& y_, const char& orientation_);
    bool hasIntersection(const Wall& other) const;

    uint x, y;
    char orientation;
    Point first, second;
};


class Node : public Point {
public:
    Node() { }
    Node(const int& x_, const int& y_) : Point(x_,y_) { };
    bool isClosed() const { return closed; } 
    bool isOpened() const { return opened; }
    bool hasWall(const char& orientation) { return wall[orientation] != nullptr; }
    bool hasWall() { return wall['H'] != nullptr || wall['V'] != nullptr;}

    bool closed = false;
    bool opened = false;
    std::unordered_map<char, std::shared_ptr<Wall>> wall { {'H', nullptr}, {'V', nullptr} };
    double gScore = std::numeric_limits<double>::infinity();
    double fScore;

    Node* previous = nullptr;
};


struct Link {
    enum LState { MAL, FUNCT, BRIDGED };
    Link() { state = MAL; hash = 0; }
    Link(uint hash_ ) { hash = hash_; }

    static uint getHash(Node* const inNode, Node* const outNode);
    // static std::pair<Point, Point> getPointsFromHash();

    uint hash;
    LState state = FUNCT;
};

struct Board {
    Board();
    Node* nodeAt(const Point& p) { if (p.inRange()) return &nodes[p.y].at(p.x); else return nullptr; }
    Node* nodeAt(const int& x, const int& y) { if (Point(x,y).inRange()) return &nodes[y][x]; else return nullptr; }
   
    std::shared_ptr<Link> getLink (Node* const inNode, Node* const outNode) { uint hash = Link::getHash(inNode, outNode); if(links.count(hash)) return links.at(hash); else return nullptr; }
    void removeLink(Node* const inNode, Node* const outNode);
    void disableLink(Node* const inNode, Node* const outNode);
    void enableLink(Node* const inNode, Node* const outNode);
    // Node* getLinkedNode(Node* const this_node);  
    std::vector<std::shared_ptr<Link>> getLinksByNode(Node* const node);
    Link::LState linkState(Node* const inNode, Node* const outNode) { return links.at(Link::getHash(inNode, outNode))->state;}
    
    std::vector<Node*> getNeighbours(Node* const node) ;
    std::vector<Node*> getNeighbourNodes(const Wall& wall);
    double heuristicCostEstimate(Point const& start, Point const& target, uint const& players_walls);

    void placeWall(const Point& p, const char& orientation);
    void removeWall(const Point& p, const char& orientation);
    bool isValidWall(const Wall& wall) ;

    std::unordered_map<uint, std::shared_ptr<Link>> links;
    std::vector<std::shared_ptr<Wall>> walls;
    std::vector<Node> nodes[9];
    const uint width = 9, height = 9;
    uint wall_count = 0;
    uint player_count = 2;
};


class Player {
public:
    Player(const int& id_);

    bool isPlaying() { if(this->pos == Point(-1,-1) ) playing = false; return playing; }
    /* Find a position of the most useful wall */
    bool findWall(Board& board, const Player& target) {}
    /* Tracks utility of the wall */
    bool trackWall(Board& board, const Point& p) {}
    /* Finds a path to the target */
    bool findPath(Board board); 

    Node getClosestTarget(Node const& node);

    void getDirection();

    bool hasEscapePath(Board board, Wall const& closing_wall);

    /* Get the possible wall position using test board and test player */
    bool getWallPosition(Board board, const Player& target_player);

    short id;
    bool playing = true;
    uint wall_count;

    Point pos;
    Node target;
    Point::PDir direction;

    std::vector<Node> g_nodes;
};


/* DEFINITIONS */

Point Point::pointOn(PDir direction) {
    switch (direction) {
        case LEFT:
            return Point(x-1, y);
        case RIGHT:
            return Point(x+1, y);
        case UP: 
            return Point(x, y-1);
        case DOWN:
            return Point(x, y+1);
    }
}

Wall::Wall(const Point& p, const char& orientation_) {
    x = p.x; y = p.y;
    orientation = orientation_;
    first = Point(p.x, p.y);
    second = orientation_ == 'V' ? Point(p.x, p.y + 1) : Point(p.x + 1, p.y);
}

Wall::Wall(const uint& x_, const uint& y_, const char& orientation_) {
    x = x_; y = y_;
    orientation = orientation_;
    first = Point(x, y);
    second = orientation_ == 'V' ? Point(x, y + 1) : Point(x + 1, y);
}

bool Wall::hasIntersection(const Wall& other) const {
    cerr << " Checking intersection of walls: " << first << " " << orientation << " and " << other.first << " " << other.orientation << endl;
    // Same orientations
    if (this->orientation == other.orientation ) {
        if (first == other.first)  return true;
        if (second == other.first) return true;
        if (first == other.second) return true;
    }
    else {
        if (second == other.second) return true;
    }
    cerr << " -- OK -- " << endl;
    return false;
}

Board::Board() {
    for (int row = 0; row < 9; row++) {
        for (int col = 0; col < 9; col++) {
            nodes[row].push_back(Node(col, row));
        }
    }

    for (int row = 0; row < 9; row++) {
        for (int col = 0; col < 9; col++) {
            Node* c_node  = nodeAt(col, row);
            Node* c_left  = nodeAt(col-1, row);
            Node* c_up    = nodeAt(col, row-1);
            Node* c_down  = nodeAt(col, row+1);
            Node* c_right = nodeAt(col+1, row);

            if (c_left) {
                uint hash = Link::getHash(c_node, c_left);
                auto link = std::make_shared<Link>( Link(hash) );
                links[hash] = link;
                hash = Link::getHash(c_left, c_node);
                links[hash] = link; 
            }
            if (c_right) {
                uint hash = Link::getHash(c_node, c_right);
                auto link = std::make_shared<Link>( Link(hash) );
                links[hash] = link;
                hash = Link::getHash(c_right, c_node);
                links[hash] = link; 
            }
            if (c_up) {
                uint hash = Link::getHash(c_node, c_up);
                auto link = std::make_shared<Link>( Link(hash) );
                links[hash] = link;
                hash = Link::getHash(c_up, c_node);
                links[hash] = link; 
            }
            if (c_down) {
                uint hash = Link::getHash(c_node, c_down);
                auto link = std::make_shared<Link>( Link(hash) );
                links[hash] = link;
                hash = Link::getHash(c_down, c_node);
                links[hash] = link; 
            }
        }
    }
}

std::vector<Node*> Board::getNeighbours(Node* const node) {
    std::vector<Node*> n;
    for (Node* const child : std::vector<Node*> { nodeAt(node->pointOn(Point::LEFT)), nodeAt(node->pointOn(Point::RIGHT)), 
                                                  nodeAt(node->pointOn(Point::UP)), nodeAt(node->pointOn(Point::DOWN))}  )
        if (child && getLink(node, child)->state == Link::FUNCT) n.push_back(child);

    return n;
}

std::vector<Node*> Board::getNeighbourNodes(const Wall& wall) {
    std::unordered_map<char, Point> dir_offset { {'H', Point(1,0)},{'V', Point(0,1)} };
    std::unordered_map<char, Point> side_offset { {'H', Point(0,-1)},{'V', Point(-1,0)} };
    Point pf = wall.first, ps = wall.second;
    std::vector<Node*> surrounding_nodes = {
        nodeAt(pf), nodeAt(ps), nodeAt(ps + dir_offset[wall.orientation]),
        nodeAt(pf + side_offset[wall.orientation]), nodeAt(ps + side_offset[wall.orientation]),
        nodeAt(ps + dir_offset[wall.orientation] + side_offset[wall.orientation])
    };
    std::vector<Node*> output;
    for ( auto const node : surrounding_nodes) 
        if (node) output.push_back(node);
    return output;
}

std::vector<std::shared_ptr<Link>> Board::getLinksByNode(Node* const node) {
    std::vector<std::shared_ptr<Link>> link_vec;
    for (Node* const n : getNeighbours(node)) {
        link_vec.push_back(getLink(node, n));;
    }
}

void Board::removeLink(Node* const inNode, Node* const outNode) {
    links.erase(Link::getHash(inNode, outNode));
    links.erase(Link::getHash(outNode, inNode));
}

void Board::disableLink(Node* const inNode, Node* const outNode) {
    links.at(Link::getHash(inNode, outNode))->state = Link::MAL;
}

void Board::enableLink(Node* const inNode, Node* const outNode) {
    links.at(Link::getHash(inNode, outNode))->state = Link::FUNCT;
}

double Board::heuristicCostEstimate(Point const& start, Point const& target, uint const& players_walls) {

    double gdist = start.distanceTo(target);
    double heuristics = nodeAt(start)->hasWall() + nodeAt(target)->hasWall();

    return gdist + heuristics;
}


bool Board::isValidWall(const Wall& wall) {
    cerr << "Checking validity of wall: " << wall.first << " " << wall.orientation << endl;
    if (wall.orientation == 'V') {
        if ( !wall.first.inRange(1,8, 0,7)) return false;
    }
    else {
        if ( !wall.first.inRange(0,7, 1,8)) return false;
    }
    
    std::vector<Node*> danger_nodes = { nodeAt(wall.first), nodeAt(wall.second) };
    for (auto const node : danger_nodes) {
        auto v_wall = node->wall['V'];
        if ( v_wall && v_wall->hasIntersection(wall) ) return false;
        auto h_wall = node->wall['H'];
        if ( h_wall && h_wall->hasIntersection(wall) ) return false;
    }
        
    return true;
}

void Board::placeWall(const Point& p, const char& orientation){
    Node* c_node = nodeAt(p);
    // If the wall has already been accounted for, return
    if ( c_node->wall[orientation] ) return;
    
    auto wall = std::make_shared<Wall> ( Wall(p, orientation) );
    c_node->wall[orientation] = wall;
    walls.push_back(wall);
    if (orientation == 'V') {
        // TODO: Instead of disabling / removing links, try to bridge them to shorten path search
        disableLink(c_node, nodeAt(c_node->pointOn(Point::LEFT)));
        Node* c_node_bottom = nodeAt(c_node->pointOn(Point::DOWN));
        disableLink(c_node_bottom, nodeAt(c_node_bottom->pointOn(Point::LEFT)));
        c_node_bottom->wall[orientation] = wall;
    }
    else {
        disableLink(c_node, nodeAt(c_node->pointOn(Point::UP)));
        Node* c_node_right = nodeAt(c_node->pointOn(Point::RIGHT));
        disableLink(c_node_right, nodeAt(c_node_right->pointOn(Point::UP)));
        c_node_right->wall[orientation] = wall;
    }
}

void Board::removeWall(const Point& p, const char& orientation){
    Node* c_node = nodeAt(p);
    c_node->wall[orientation] = nullptr;

    for (auto it = walls.begin(); it != walls.end(); it++) {
        if ((*it)->first == p && (*it)->orientation == orientation) {
            walls.erase(it);
            break;
        }
    }

    if (orientation == 'V') {
        // TODO: Instead of disabling / removing links, try to bridge them to shorten path search
        enableLink(c_node, nodeAt(c_node->pointOn(Point::LEFT)));
        Node* c_node_bottom = nodeAt(c_node->pointOn(Point::DOWN));
        enableLink(c_node_bottom, nodeAt(c_node_bottom->pointOn(Point::LEFT)));
        c_node_bottom->wall[orientation] = nullptr;
    }
    else {
        enableLink(c_node, nodeAt(c_node->pointOn(Point::UP)));
        Node* c_node_right = nodeAt(c_node->pointOn(Point::RIGHT));
        enableLink(c_node_right, nodeAt(c_node_right->pointOn(Point::UP)));
        c_node_right->wall[orientation] = nullptr;
    }
}


uint Link::getHash(Node* const inNode, Node* const outNode) { 
    std::string s; 
    for (auto e : std::vector<int>{inNode->x, inNode->y, outNode->x, outNode->y}) 
        s += std::to_string(e); 

    return std::stoi(s); 
}


Player::Player(const int& id_) {
    id = id_;
    switch (id) {
        case 0:
            direction = Point::RIGHT;
            break;
        case 1:
            direction = Point::LEFT;
            break;
        case 2:
            direction = Point::DOWN;
            break;
        case 3:
            direction = Point::UP;
            break;
    }
}


bool Player::findPath(Board board) {

    std::vector<Node*> openSet;
    Node* parent = board.nodeAt(pos);
    
    // Complete heuristic for the first node
    this->target = this->getClosestTarget(*parent);
    parent->fScore = board.heuristicCostEstimate(*parent, this->target, this->wall_count);

    openSet.push_back(parent);

    bool goal_found = false;
    while (!openSet.empty()) {
        parent = openSet.back();
        // cerr << "Current parent node: " << *parent << endl;

        if ( *parent == this->target ) {
            goal_found = true;
            break; // Target reached    
        }
        
        parent->closed = true;
        openSet.pop_back();
        parent->opened = false;
        
        // Set up potentially closest target
        this->target = this->getClosestTarget(*parent);

        for ( Node* const child : board.getNeighbours(parent)) {
            if (child->isClosed()) continue;
            uint tentative_gScore = parent->gScore + 1;
            if (!child->isOpened()) {
                openSet.push_back(child);
                child->opened = true;
            }
            if (tentative_gScore >= child->gScore) continue;
            child->gScore = tentative_gScore;
            child->previous = parent;
            child->fScore = tentative_gScore + board.heuristicCostEstimate(*child, this->target, this->wall_count);
            // cerr << "  Pushing back child: " << *child << endl;
        }
        // Minimize the fScore decreasingly
        std::sort(openSet.begin(), openSet.end(), [] (Node* const n1, Node* const n2) { return n1->fScore > n2->fScore;});
    }

    if (!goal_found) return false;
    
    this->g_nodes.clear();
    this->g_nodes.push_back(*parent);
    while (parent = parent->previous) {
        this->g_nodes.push_back(*parent);
    }
    
    this->target = this->g_nodes.at(g_nodes.size() - 2);
    cerr << "Final target for player " << this->id << ": " << this->target << endl;

    return true;
}

Node Player::getClosestTarget(Node const& node) {
    if (direction == Point::UP) return Node(node.x, 0);
    if ( Point::DOWN == direction) return Node(node.x, 8);
    if ( Point::LEFT == direction) return Node(0, node.y);
    if ( Point::RIGHT == direction) return Node(8, node.y);
}

void Player::getDirection() {
    if (this->pos.isRightFrom(this->target)) cout << "LEFT" << endl;
    else if (this->pos.isLeftFrom(this->target)) cout << "RIGHT" << endl;
    else if (this->pos.isUpFrom(this->target)) cout << "DOWN" << endl;
    else if (this->pos.isDownFrom(this->target)) cout << "UP" << endl;
    
    else cout << "ERROR GETTING DIRECTION" << endl;
}


bool Player::hasEscapePath(Board board, Wall const& closing_wall) {

    board.placeWall(closing_wall.first, closing_wall.orientation);

    if (this->findPath(board)) {
        board.removeWall(closing_wall.first, closing_wall.orientation);
        return true;
    }

    board.removeWall(closing_wall.first, closing_wall.orientation);
    return false;
}


bool Player::getWallPosition(Board board, const Player& target_player) {

    // TODO: minimax algorithm
    char wall_or[2] { 'H', 'V' };
    std::unordered_map<char, Point> wall_offset { {'H', Point(-1,0) }, {'V', Point(0,-1)}};

    for ( auto it = target_player.g_nodes.rbegin(); it != target_player.g_nodes.rend() - 1; it++) {
        Node c_node = *it; Node n_node = *(it + 1);
        Player test_player = target_player;
        
        char orientation = wall_or[c_node.isHorizontaly(n_node)];

        if ( Point::LEFT == c_node.whereIs(n_node) || Point::UP == c_node.whereIs(n_node) ) {
            Wall test_wall(c_node, orientation);
            Wall test_wall_moved(c_node + wall_offset[orientation], orientation);
            if ( board.isValidWall(test_wall) ) {
                if (test_player.hasEscapePath(board, test_wall)) { 
                    cout << c_node << " " << orientation << endl;
                    return true;
                }
            }
            // if ( board.isValidWall(test_wall_moved) ) {
            //      if (test_player.hasEscapePath(board, test_wall_moved)) { 
            //         cout << c_node + wall_offset[orientation] << " " << orientation << endl;
            //         return true;
            //     }
            // }
        }
        else {
            Wall test_wall(n_node, orientation);
            Wall test_wall_moved(n_node + wall_offset[orientation], orientation);
            if ( board.isValidWall(test_wall) ) {
                if (test_player.hasEscapePath(board, test_wall)) {
                cout << n_node << " " << orientation << endl;
                return true;  
                } 
            }
            // if ( board.isValidWall(test_wall_moved) ) {
            //      if (test_player.hasEscapePath(board, test_wall_moved)) { 
            //         cout << n_node + wall_offset[orientation] << " " << orientation << endl;
            //         return true;
            //     }
            // }
        }
    }

    return false;
}

/* MAIN GAME */ 

int main()
{
    int w, h, playerCount, my_Id;
    cin >> w >> h >> playerCount >> my_Id; cin.ignore();

    Board board; board.player_count = 4;
    std::vector<Player> players { Player(0), Player(1), Player(2), Player(3) };
    Player& my_player = players.at(my_Id);
    
    // game loop
    while (1) {
        for (int i = 0; i < playerCount; i++) {
            int x, y, walls_left; // x-coordinate of the player
            cin >> x >> y >> walls_left; cin.ignore();
            players[i].pos = Point(x,y);
            players[i].wall_count = walls_left;
        }
        int wallCount; // number of walls on the board
        cin >> wallCount; cin.ignore();
        board.wall_count = wallCount;   
        for (int i = 0; i < wallCount; i++) {
            int x,y; char orientation; // x,y-coordinate of the wall
            cin >> x >> y >> orientation; cin.ignore();
            board.placeWall(Point(x,y), orientation);
        }
    
        // Evaluate danger - count up places
        int danger_Id;
        double min_heuristic_distance = std::numeric_limits<double>::infinity();
        for ( int id = 0; id < playerCount; id++ ) {
            Player& player = players[id];
            if (!player.isPlaying()) continue;
            Node& player_node = *board.nodeAt(player.pos);
            double heuristics = board.heuristicCostEstimate( player_node, player.getClosestTarget(player_node), player.wall_count);
            cerr << id << " heuristics: " << heuristics << endl;
            if (heuristics < min_heuristic_distance) {
                danger_Id = id;
                min_heuristic_distance = heuristics;
            }
        }

        my_player.findPath(board);
        if (danger_Id != my_Id && my_player.wall_count) {
            Player& danger_player = players.at(danger_Id);
            danger_player.findPath (board);
            // std::for_each(danger_player.g_nodes.begin(), danger_player.g_nodes.end(), [](Node& node) {cerr << node << "  " ;});
            // cerr << endl;
            if (my_player.getWallPosition(board, danger_player)) continue;
        }

        my_player.getDirection();
    }
}