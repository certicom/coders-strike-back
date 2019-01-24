#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

class Point
{
public:

	Point() : x(0), y(0) {}

	Point(int _x, int _y) : x(_x), y(_y) {}

	//Point(const Point& other) {x = other.x; y = other.y;}

	bool operator==(const Point& other)const { return (x == other.x && y == other.y); }

	bool operator!=(const Point& other)const { return (x != other.x || y != other.y); }

	// a vector operation but same thing
	int Norme() { return sqrtf(x*x + y*y); }

	int x;
	int y;
};


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

void LearnTheField(int x, int y, vector<Point>& list)
{
	if (KnowTheField(list))
		return; // easier

	if (list.size() == 0 || Point(x, y) != list[list.size() - 1])
	{
		list.push_back(Point(x, y));
	}
}

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

		string power;

		int angle = abs(nextCheckpointAngle);

		cerr << speed.Norme() << endl;

		Point target;

		if (isKnowingTheField)
		{
			// all the point of learning the checkpoint is for this adrifting effect
			if (speed.Norme()*1.5 > nextCheckpointDist)
			{
				target = GetNextCheckpoint(x, y, checkpointList); // well, the one after the normal logic target
				power = "0";
			}
			else
			{
				target = Point(nextCheckpointX, nextCheckpointY);

				if (angle > 90)
					power = "0";
				else
					power = "100";
			}
		}
		else
		{
			target = Point(nextCheckpointX, nextCheckpointY);

			if (angle > 90)
				power = "0";
			else
				power = "100";
		}



		/*
		if(nextCheckpointDist > 4000 && angle < 5 )
		power = "BOOST";
		*/
		lastPos = Point(x, y);

		cout << target.x << " " << target.y << " " << power << endl;
	}
}
