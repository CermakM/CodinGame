#include <iostream>
#include <string>
#include <cmath>

using std::cin; using std::cout; using std::endl;

int Difference( const int& bound, const int& current ) {
    
	return round( fabs( (float)bound - (float)current) / 2 );
}

int main() {
	int W, H; // width and height of the building.
	cin >> W >> H; cin.ignore();
	int N; // maximum number of turns before game over.
	cin >> N; cin.ignore();
	int X0, Y0;
	cin >> X0 >> Y0; cin.ignore();
    
    // Set up search limits
	int lower_hbound, lower_vbound; lower_hbound = lower_vbound = 0;
	int higher_hbound = W - 1, higher_vbound = H - 1;

	while ( 1 ) {
		std::string bombDir; // the direction of the bombs from batman's current location (U, UR, R, DR, D, DL, L or UL)
		cin >> bombDir; cin.ignore();
        
        // Each step set the new lower or higher bound based on current position and compute diff
        // Note: We add or subtract one in order not to visit the place agian
		for ( char& letter : bombDir ) {
			switch ( letter ) {
				case 'U':
					higher_vbound = Y0 - 1;
					Y0 -= Difference( lower_hbound, Y0 );
					break;
				case 'D':
					lower_hbound = Y0 + 1;
					Y0 += Difference( higher_vbound, Y0 );
					break;
				case 'R':
					lower_vbound = X0 + 1;
					X0 += Difference( higher_hbound, X0 );
					break;
				case 'L':
					higher_hbound = X0 - 1;
					X0 -= Difference( lower_vbound, X0 );
					break;
			}
		}
		std::string output = std::to_string( X0 ) + " " + std::to_string( Y0 );
		cout << output << endl;
	}
}
