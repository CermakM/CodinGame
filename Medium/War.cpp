#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>

using std::cin; using std::cerr; using std::cout; using std::endl;

class Card {
    public:
        Card() { /* empty */ };
        Card(std::string& card_string) {
            _color = card_string.back();
            card_string.pop_back();
            try {
                _value = std::stoi(card_string);
            }
            catch (...) {
                _value = _value_mapper[card_string];
            }
        }
        Card(const Card& other) {
            this->_color = other._color;
            this->_value = other._value;
        }
        
        bool operator == (Card const& other) {
            return this->_value == other._value;
        }
        bool operator > (Card const& other) {
            return this->_value > other._value;
        }
        
        unsigned short int value() const { return _value; }
        std::string color() const { return _color; }
        
    private:
        std::string _color = "";
        std::unordered_map<std::string, int> _value_mapper { {"J", 11}, {"Q", 12}, {"K", 13}, {"A", 14} };
        unsigned short int _value = -1;
};

// Returns the winning player - 1 or 2 || PAT - 0
int Battle(std::vector<Card>& deck_p1, std::vector<Card>& deck_p2, std::vector<Card>& discarded_deck, int num_of_cards = 1) {

    if (deck_p1.size() < num_of_cards || deck_p2.size() < num_of_cards)
        return 0; // PAT
    Card card1, card2;
    for ( int i = 0; i < num_of_cards; i++) {
        card1 = std::move(deck_p1.front()); card2 = std::move(deck_p2.front());
        deck_p1.erase(deck_p1.begin()); deck_p2.erase(deck_p2.begin());
        discarded_deck.push_back(card1); discarded_deck.push_back(card2);
    }
    if (card1 > card2) {
        for (int i = 0; i < discarded_deck.size(); i+=2) deck_p1.push_back(discarded_deck[i]);
        for (int i = 1; i < discarded_deck.size(); i+=2) deck_p1.push_back(discarded_deck[i]);
        return 1;
    }
    else if (card2 > card1) {
        for (int i = 0; i < discarded_deck.size(); i+=2) deck_p2.push_back(discarded_deck[i]);
        for (int i = 1; i < discarded_deck.size(); i+=2) deck_p2.push_back(discarded_deck[i]);
        return 2;
    }
    else 
        return Battle(deck_p1, deck_p2, discarded_deck, 4);
}


int main()
{
    int n; // the number of cards for player 1
    cin >> n; cin.ignore();
    std::vector<Card> deck_p1, deck_p2;
    for (int i = 0; i < n; i++) {
        std::string input;
        cin >> input; cin.ignore();
        deck_p1.push_back(Card(input));
    }
    int m; // the number of cards for player 2
    cin >> m; cin.ignore();
    for (int i = 0; i < m; i++) {
        std::string input;
        cin >> input; cin.ignore();
        deck_p2.push_back(Card(input));
    }

    int game_rounds = 0, winner = 1;
    while ( !(deck_p1.empty() || deck_p2.empty()) && winner) {
      
        std::vector<Card> discarded_deck;
        winner = Battle(deck_p1, deck_p2, discarded_deck);
        ++ game_rounds;
    }    
    
    !winner ? cout << "PAT" : cout << winner << " " << game_rounds;
    return 0;
}
