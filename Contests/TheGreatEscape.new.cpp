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
    bool isDownFrom(const Point& other) const { return x == other.x && y > other.y; }
    bool isFarDownFrom(const Point& other) const { return y > other.y; }
    bool isUpFrom(const Point& other) const { return x == other.x && y < other.y; }
    bool isFarUpFrom(const Point& other) const { return y < other.y; } 
    bool isRightFrom (const Point& other) const  { return y == other.y && x > other.x; }
    bool isFarRightFrom(const Point& other) const { return x > other.x; }
    bool isLeftFrom(const Point& other) const { return y == other.y && x < other.x; } 
    bool isFarLeftFrom(const Point& other) const { return x < other.x; }
    PDir whereIs(const Point& other) { if (other.isFarLeftFrom(*this)) return LEFT; if (other.isFarRightFrom(*this)) return RIGHT; if (other.isFarUpFrom(*this)) return UP; if(other.isFarDownFrom(*this)) return DOWN; return HERE;}
    Point operator - (const Point& other) { return Point( x - other.x, y - other.y);}
    Point operator + (const Point& other) { return Point( x + other.x, y + other.y);}
    double distanceTo (const Point& other) { return sqrt(pow(x - other.x,2 ) + pow(y - other.y, 2)); }
    bool operator == (const Point& other) { return x == other.x && y == other.y; }
    operator bool() const { return x >= 0 && y >=0; }
    friend std::ostream& operator << (std::ostream& os, const Point& p) { return os << "x: " << p.x << " y: " << p.y; }

    bool inRange() const { return x >= 0 && x < 9 && y >= 0 && y < 9; }
    int x,y;
};

class Node : public Point {
public:
    Node() { }
    Node(const int& x_, const int& y_) : Point(x_,y_) { };
    bool isClosed() const { return closed; } 
    bool isOpened() const { return opened; }

    bool closed = false;
    bool opened = false;
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
    enum AccessOption { ALL, ACCESSIBLE, INACCESSIBLE };
    Node* nodeAt(const Point& p) { if (p.inRange()) return &nodes[p.y].at(p.x); else return nullptr; }
    Node* nodeAt(const int& x, const int& y) { if (Point(x,y).inRange()) return &nodes[y][x]; else return nullptr; }
    std::shared_ptr<Link> getLink (Node* const inNode, Node* const outNode) { uint hash = Link::getHash(inNode, outNode); if(links.count(hash)) return links.at(hash); else return nullptr; }
    // void removeLink(std::shared_ptr<Link>>);
    void removeLink(Node* const inNode, Node* const outNode);
    void disableLink(Node* const inNode, Node* const outNode);
    // Node* getLinkedNode(Node* const this_node);  
    std::vector<std::shared_ptr<Link>> getLinksByNode(Node* const node);
    std::vector<Node*> getNeighbours(Node* const node) ;
    // std::vector<Node*> followLink(const Link& link);

    void placeWall(const Point& p, const char& orientation);
    Link::LState linkState(Node* const inNode, Node* const outNode) { return links.at(Link::getHash(inNode, outNode))->state;}

    double heuristicCostEstimate(Node* const start, Node* const target) { return start->distanceTo(*target) + double( wall_count ) * (this->player_count - 1);}

    std::unordered_map<uint, std::shared_ptr<Link>> links;
    std::vector<Node> nodes[9];
    const uint width = 9, height = 9;
    uint wall_count = 0;
    uint player_count = 2;
};


class Player {
public:
    Player(const int& id_);
    /* Find a position of the most useful wall */
    bool findWall(Board& board, const Player& target) {}
    /* Tracks utility of the wall */
    bool trackWall(Board& board, const Point& p) {}
    /* Finds a path to the target */
    void findPath(Board board);
    void setClosestTarget(Node* const node);
    void getDirection();

    short id;
    Point pos;
    Node target;
    Point::PDir direction;
    uint wall_count;
    std::vector<Node*> g_nodes;
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

// std::vector<Node*> Board::followLink(const std::shared_ptr<Link>& link) {
//     std::vector<Node*> node_vec;
//     if (link.state != Link::BRIDGED) return node_vec;

//     auto bridged_link = link;
//     node_vec.push_back(link->second);
//     while (bridged_link = link->bridge)
//         node_vec.push_back(bridged_link->getLinkedNode(node_vec.back()));

//     return node_vec;
// }


void Board::placeWall(const Point& p, const char& orientation){
    Node* c_node = nodeAt(p);
    if (orientation == 'V') {
        // TODO: Instead of disabling / removing links, try to bridge them to shorten path search
        disableLink(c_node, nodeAt(c_node->pointOn(Point::LEFT)));
        Node* c_node_bottom = nodeAt(c_node->pointOn(Point::DOWN));
        disableLink(c_node_bottom, nodeAt(c_node_bottom->pointOn(Point::LEFT)));
    }
    else {
        disableLink(c_node, nodeAt(c_node->pointOn(Point::UP)));
        Node* c_node_right = nodeAt(c_node->pointOn(Point::RIGHT));
        disableLink(c_node_right, nodeAt(c_node_right->pointOn(Point::UP)));
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


void Player::findPath(Board board) {

    std::vector<Node*> openSet;
    Node* parent = board.nodeAt(pos);
    
    // Complete heuristic for the first node
    this->setClosestTarget(parent);
    parent->fScore = board.heuristicCostEstimate(parent, &this->target);

    openSet.push_back(parent);

    Node* previously_opened = nullptr;
    while (!openSet.empty()) {
        parent = openSet.back();
        cerr << "Current parent node: " << *parent << endl;

        if ( *parent == this->target ) 
            break; // Target reached    
        
        parent->closed = true;
        openSet.pop_back();
        parent->opened = false;
        
        // Set up potentially closest target
        this->setClosestTarget(parent);

        // if (direction == Point::LEFT && parent->isFarLeftFrom(this->target)) { parent->closed = true; continue; }
        // if (direction == Point::RIGHT && parent->isFarRightFrom(this->target)) { parent->closed = true; continue; }
        // if (direction == Point::UP && parent->isFarUpFrom(this->target)) { parent->closed = true; continue; }
        // if (direction == Point::DOWN && parent->isFarDownFrom(this->target)) { parent->closed = true; continue; }



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
            child->fScore = tentative_gScore + board.heuristicCostEstimate(child, &this->target);
            // cerr << "  Pushing back child: " << *child << endl;
        }
        // Minimize the fScore decreasingly
        std::sort(openSet.begin(), openSet.end(), [] (Node* const n1, Node* const n2) { return n1->fScore > n2->fScore;});
    }
    
    this->g_nodes.push_back(parent);
    while (parent = parent->previous) {
        this->g_nodes.push_back(parent);
    }
    
    this->target = *this->g_nodes.at(g_nodes.size() - 2);
    cerr << "Final target: " << this->target << endl;
}

void Player::setClosestTarget(Node* const node) {
    if (direction == Point::UP) this->target = Node(node->x, 0);
    else if ( Point::DOWN == direction) this->target = Node(node->x, 8);
    else if ( Point::LEFT == direction) this->target = Node(0, node->y);
    else if ( Point::RIGHT == direction) this->target = Node(8, node->y);
}

void Player::getDirection() {
    if (this->pos.isRightFrom(this->target)) cout << "LEFT" << endl;
    else if (this->pos.isLeftFrom(this->target)) cout << "RIGHT" << endl;
    else if (this->pos.isUpFrom(this->target)) cout << "DOWN" << endl;
    else if (this->pos.isDownFrom(this->target)) cout << "UP" << endl;
    
    else cout << "ERROR GETTING DIRECTION" << endl;
}

/* MAIN GAME */ 

int main()
{
    int w, h, playerCount, myId;
    cin >> w >> h >> playerCount >> myId; cin.ignore();

    Board board; board.player_count = 4;
    std::vector<Player> players { Player(0), Player(1), Player(2), Player(3) };
    Player& my_player = players.at(myId);
    
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
    
        my_player.findPath (board);
        my_player.getDirection();
    }
}