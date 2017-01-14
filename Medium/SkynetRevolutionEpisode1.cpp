#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <utility>
#include <algorithm>

typedef std::pair<int, int> Link;

class SkyDog
{

public:

	SkyDog();
	~SkyDog();

	void AddLink( int node1, int node2 );

	/**
	*  Get only those links, which have connection to a gate
	*/
	void FindGateLinks( const int& index_of_gate );

	/*
	*	Get the link to severe
	*/
	std::string GetCutoffIndex( const int& agent_index );

	/**
	*	Clears the excessive links
	*/
	void RemoveExcessive();

private:

	std::vector<Link>* _links = nullptr;
	std::deque<Link>  _gate_links;

};

using std::cin; using std::cout; using std::endl;

int main() {
	SkyDog Dog;
	int numOfNodes; // the total number of nodes in the level, including the gateways
	int numOfLinks; // the number of links
	int numOfGates; // the number of exit gateways
	cin >> numOfNodes >> numOfLinks >> numOfGates; cin.ignore();
	for ( int i = 0; i < numOfLinks; i++ ) {
		int node1, node2; // N1 and N2 defines a link between these nodes
		cin >> node1 >> node2; cin.ignore();
		node1 > node2 ? Dog.AddLink( node1, node2 ) : Dog.AddLink( node2, node1 );
	}

	for ( int i = 0; i < numOfGates; i++ ) {
		int indexOfGate; // the index of a gateway node
		cin >> indexOfGate; cin.ignore();
		Dog.FindGateLinks( indexOfGate );
	}

	Dog.RemoveExcessive();

	// game loop
	while ( 1 ) {
		int agent; // The index of the node on which the Skynet agent is positioned this turn
		cin >> agent; cin.ignore();

		cout << Dog.GetCutoffIndex( agent ) << endl;
	}
}

// ------------

SkyDog::SkyDog() {
	_links = new std::vector<Link>;
}

SkyDog::~SkyDog() {
	delete _links;
}

void SkyDog::AddLink( int node1, int node2 ) {
	Link new_link = std::make_pair( node1, node2 );
	_links->push_back( new_link );
}

void SkyDog::FindGateLinks( const int& index_of_gate ) {

	for ( Link& link : *_links ) {
		if ( link.first == index_of_gate ) {
			// Swap contents
			int temp_val = link.second;
			link.second = link.first;
			link.first = temp_val;
			_gate_links.push_back( link );
		}
		else if ( link.second == index_of_gate )
			_gate_links.push_back( link );
	}
}

std::string SkyDog::GetCutoffIndex( const int& agent_index ) {

	std::string cutoff_link;

	// Sort out by the lowest distance to the agent
	std::sort( _gate_links.begin(), _gate_links.end(), [agent_index]( Link& link1, Link& link2 )
	{ return abs( link1.first - agent_index ) < abs( link2.first - agent_index ); }
	);

	Link link = _gate_links.front();
	cutoff_link = std::to_string( link.first ) + " " + std::to_string( link.second );

	_gate_links.pop_front();

	return cutoff_link;
}

void SkyDog::RemoveExcessive() {

	delete _links;
	_links = nullptr;
}
