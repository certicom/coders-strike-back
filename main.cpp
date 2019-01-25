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


// No API to do that apparently, like the given angle this return an angle between -180 and 180
int GetAngle(Point& vector)
{
	float alpha = atanf((float)vector.y / vector.x);

	if (vector.x < 0)
		alpha += M_PI; // to be on a 2pi range instead of 1pi

	if (alpha > M_PI)
		alpha -= 2 * M_PI;

	return (alpha * 180.0f) / M_PI; // degres because the game gives degres
}


// return the angle between two vectors
// this is limited to -180 - 180
int GetAngleBetweenTwoVectors(Point& vector1, Point& vector2)
{
	int alpha1 = GetAngle(vector1);

	int alpha2 = GetAngle(vector2);

	int diff = alpha1 - alpha2;


	if (diff > 180)
		diff -= 360;

	if (diff < -180)
		diff += 360;

	return diff;
}

// This is an obselete part, but it may be reused in the other leagues
bool KnowTheField(vector<Point>& list)
{
	if (list.size() == 0)
		return false;

	Point p = list[0];

	for (int i = 1; i<list.size(); i++)
	{
		if (list[i] == p)
			return true;
	}
}

// This is an obselete part, but it may be reused in the other leagues
void LearnTheField(int x, int y, vector<Point>& list)
{
	if (KnowTheField(list))
		return; // easier

	if (list.size() == 0 || Point(x, y) != list[list.size() - 1])
	{
		list.push_back(Point(x, y));
	}
}

// This is an obselete part, but it may be reused in the other leagues
Point GetNextCheckpoint(int x, int y, vector<Point>& list)
{
	Point p(x, y);

	for (int i = 0; i<list.size(); i++)
	{
		if (list[i] == p)
			return i == list.size() - 1 ? list[0] : list[i + 1];
	}

	return p;
}

/**
* Auto-generated code below aims at helping you parse
* the standard input according to the problem statement.
**/
int main()
{

	// game loop
	while (1) {
		int x;
		int y;
		int nextCheckpointX; // x position of the next check point
		int nextCheckpointY; // y position of the next check point
		int nextCheckpointDist; // distance to the next checkpoint
		int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
		cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
		int opponentX;
		int opponentY;
		cin >> opponentX >> opponentY; cin.ignore();

		//----------------------------------------------------------------------
		//----------------------------------------------------------------------

		// This part handle the learning of the checkpoints
		static std::vector<Point> checkpointList;

		static bool isKnowingTheField = false;

		LearnTheField(nextCheckpointX, nextCheckpointY, checkpointList);

		if (!isKnowingTheField && KnowTheField(checkpointList))
		{
			isKnowingTheField = true;
			checkpointList.resize(checkpointList.size() - 1);
			// now we have a clean representation of all the checkpoints in order
		}

		//----------------------------------------------------------------------
		//----------------------------------------------------------------------

		static Point lastPos(x, y);

		Point speed(x - lastPos.x, y - lastPos.y); // more a vector, but same structure

		Point toCheckpoint(nextCheckpointX - x, nextCheckpointY - y);

		Point opponent(opponentX, opponentY);

		Point target;

		string power;

		int alpha = GetAngle(toCheckpoint);

		int beta = GetAngle(speed);

		int gamma = GetAngleBetweenTwoVectors(toCheckpoint, speed);


		Point normal; // this is THE trajectory correction vector 

		if (gamma >= 0)
		{
			normal = Point(-toCheckpoint.y, toCheckpoint.x); // the 'right' normal
		}
		else
		{
			normal = Point(toCheckpoint.y, -toCheckpoint.x); // the 'left' normal
		}


		float multiplier = (float)speed.Norme() / toCheckpoint.Norme();

		normal *= multiplier;

		target = Point(nextCheckpointX, nextCheckpointY);

		if (speed.Norme() > 1)
			target += normal;



		cerr << "speed :  " << GetAngle(speed) << "  " << speed.Norme() << endl;
		cerr << "check :  " << GetAngle(toCheckpoint) << "  " << toCheckpoint.Norme() << endl;
		cerr << "Normal :  " << normal.Norme() << endl;
		cerr << "Diff angle :  " << gamma << endl; // - left / + right




		// these 5 lines are still the best
		int angle = abs(nextCheckpointAngle);

		if (angle > 90)
			power = "0";
		else
			power = "100";


		// And I do not even use the boost :)
		/*
		if(nextCheckpointDist > 4000 && angle < 5 )
		power = "BOOST";
		*/


		lastPos = Point(x, y);

		cout << target.x << " " << target.y << " " << power << endl;
	}
}
