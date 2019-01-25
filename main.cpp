#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

// This will be way more simple with a Point structure
class Point
{
public:

	Point() : x(0), y(0) {}

	Point(int _x, int _y) : x(_x), y(_y) {}

	bool operator==(const Point& other)const { return (x == other.x && y == other.y); }

	bool operator!=(const Point& other)const { return (x != other.x || y != other.y); }

	bool operator*=(float multiplier) { x *= multiplier; y *= multiplier; }

	bool operator+=(const Point& other) { x += other.x; y += other.y; }

	// a vector operation but same thing
	int Norme() { return sqrtf(x*x + y*y); }

	int x;
	int y;
};


class Terrain
{
public:

	// just the basic reading
	Terrain()
	{
		cin >> laps; cin.ignore();
		cin >> nbCheckpoints; cin.ignore();
		for (int i = 0; i < nbCheckpoints; i++) {
			Point p;
			cin >> p.x >> p.y; cin.ignore();
			checkpoints.push_back(p);
		}
	}


	int laps;
	int nbCheckpoints;
	std::vector<Point> checkpoints;
};




class Pod
{
public:

	// apparently allies and opponents receive the same infos so it is better here
	Pod()
	{
		cin >> position.x >> position.y >> speed.x >> speed.y >> angle >> nextCheckpoint; cin.ignore();
	}

	// GetCheckpoint mean current destination 
	Point GetCheckpoint(const Terrain& terrain) const {
		return terrain.checkpoints[nextCheckpoint];
	}


	Point speed;

	Point position;

	int angle;

	int nextCheckpoint;
};


class Ally : public Pod
{
public:

	void GoTo(const Point& dest, const string& power) const {
		cout << dest.x << " " << dest.y << " " << power << endl;
	}

	// GetNextCheckpoint mean the point after my current destination 
	Point GetNextCheckpoint(const Terrain& terrain) const {
		return terrain.checkpoints[nextCheckpoint];
	}

};


class Opponent : public Pod
{
public:

};



int main()
{
	Terrain terrain = Terrain(); // this read the input data

								 // game loop
	while (1) {

		// those read the input, so the order is importants
		Ally pod1 = Ally();
		Ally pod2 = Ally();

		Opponent opponent1 = Opponent();
		Opponent opponent2 = Opponent();


		pod1.GoTo(pod1.GetCheckpoint(terrain), "100");
		pod2.GoTo(pod2.GetCheckpoint(terrain), "100");

	}
}