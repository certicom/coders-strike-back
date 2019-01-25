#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

// define the intensity of speed compensation for trajectory correction 
#define TRAJECTORY_CORRECTION_INTENSITY 0.01f // proportionel

// When we get close to a checkpoint, we must start turning toward the next 
// one to benefit off the inertia. This create a 'drift effect' but sometimes
// we can be to short to trigger the checkpoint, so this define set the level of safety we add
// too low we miss, too high we waste time
#define DRIFTING_SAFEFTY 0.4f // probably proportionel

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
	int Norme() const { return sqrtf(x*x + y*y); }

	int x;
	int y;
};


// No API to do that apparently, like the given angle this return an angle between -180 and 180
int GetAngle(const Point& vector) {

	float alpha = atanf((float)vector.y / vector.x);

	if (vector.x < 0)
		alpha += M_PI; // to be on a 2pi range instead of 1pi

	if (alpha > M_PI)
		alpha -= 2 * M_PI;

	return (alpha * 180.0f) / M_PI; // degres because the game gives degres
}


// return the angle between two vectors
// this is limited to -180 - 180
int GetAngleBetweenTwoVectors(const Point& vector1, const Point& vector2) {

	int alpha1 = GetAngle(vector1);

	int alpha2 = GetAngle(vector2);

	int diff = alpha1 - alpha2;


	if (diff > 180)
		diff -= 360;

	if (diff < -180)
		diff += 360;

	return diff;
}


// this return a vector with a norme of 10000 because internaly point only use integers
Point GetVectorFromAngle(int angle) {

	float radAngle = (angle * M_PI) / 180.0f;
	return Point(10000.0f*cosf(radAngle), 10000.0f*sinf(radAngle));
}

// very classic computation of the disatnce between two points
int Distance(const Point& vector1, const Point& vector2) {

	Point p((vector1.x - vector2.x), (vector1.y - vector2.y));
	return sqrtf(p.x*p.x + p.y*p.y);
}

class Terrain
{
public:

	// just the basic reading
	Terrain() {

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
	Pod(Terrain& _terrain) {

		cin >> position.x >> position.y >> speed.x >> speed.y >> angle >> nextCheckpoint; cin.ignore();
		terrain = &_terrain;
	}

	// Get the angle between the direction of the pod and a point
	int GetPodAngleWithSomething(const Point& something) const {
		return GetAngleBetweenTwoVectors(GetVectorFromAngle(angle), Point(something.x - position.x, something.y - position.y));
	}

	// GetCheckpoint mean current destination 
	Point GetCheckpoint() const {
		return terrain->checkpoints[nextCheckpoint];
	}

	// This will be more detailled in the report
	int GetStopDistance() {
		// 5.666666 is the result of a simple convergent serie since friction is proportional
		return speed.Norme() * (5.66666f - DRIFTING_SAFEFTY);
	}

	Point speed;
	Point position;
	Terrain* terrain; // just to internally use the terrain
	int angle;
	int nextCheckpoint;
};



class Opponent : public Pod
{
public:

	Opponent(Terrain& _terrain) : Pod(_terrain) {}
};



class Ally : public Pod
{
public:

	Ally(Terrain& _terrain) : Pod(_terrain) {}

	void GoTo(const Point& dest, const string& power) const {

		// the previous algorithm wich worked very well
		Point toTarget(dest.x - position.x, dest.y - position.y);

		int alpha = GetAngle(toTarget);

		int beta = GetAngle(speed);

		int gamma = GetAngleBetweenTwoVectors(toTarget, speed);


		Point normal; // this is THE trajectory correction vector 

		if (gamma >= 0)
		{
			normal = Point(-toTarget.y, toTarget.x); // the 'right' normal
		}
		else
		{
			normal = Point(toTarget.y, -toTarget.x); // the 'left' normal
		}


		//float multiplier = (float)speed.Norme() / (1.0f/toTarget.Norme());

		float multiplier = abs(gamma)*TRAJECTORY_CORRECTION_INTENSITY;

		normal *= multiplier;

		Point target;

		target = dest;


		if (speed.Norme() > 1)
			target += normal;


		cout << target.x << " " << target.y << " " << power << endl;
	}

	// GetNextCheckpoint mean the point after my current destination 
	Point GetNextCheckpoint() const {

		if (nextCheckpoint == terrain->checkpoints.size() - 1)
			return terrain->checkpoints[0];

		return terrain->checkpoints[nextCheckpoint + 1];
	}

	bool IsOpponentTooClose(const Opponent& op) {

		if (Distance(op.position, position) < 900) {
			return true;
		}
		return false;
	}

};


class Racer : public Ally
{
public:

	Racer(Terrain& _terrain) : Ally(_terrain) {}

	// since the begging those 5 simple line seems to be incredible efficient
	const string GetPowerForTarget(const Point& target) const {

		int angle = abs(GetPodAngleWithSomething(target));

		if (angle > 90)
			return "0";

		return "100";
	}

	// probably better as polymorphic function but here we do not need it
	// this function control the pod
	void Play(const Opponent& op1, const Opponent& op2) {


		string power;

		/*
		if(GetCheckpoint().Norme() > 4000 && angle < 5 )
		power = "BOOST";
		*/

		//cerr << "distance  " << GetStopDistance() << "     "  << Distance(GetCheckpoint(), position)<< endl;


		Point ToCheckpoint(GetCheckpoint().x - position.x, GetCheckpoint().y - position.y);

		int angle = abs(GetAngleBetweenTwoVectors(ToCheckpoint, speed));


		if (Distance(GetCheckpoint(), position) < GetStopDistance() && angle < 30)
		{
			power = GetPowerForTarget(GetNextCheckpoint());
			GoTo(GetNextCheckpoint(), power);
		}
		else
		{
			power = GetPowerForTarget(GetCheckpoint());

			if (IsOpponentTooClose(op1) && Point(speed.x - op1.speed.x, speed.y - op1.speed.y).Norme() > 300)
				power = "SHIELD";

			else if (IsOpponentTooClose(op2) && Point(speed.x - op2.speed.x, speed.y - op2.speed.y).Norme() > 300)
				power = "SHIELD";


			GoTo(GetCheckpoint(), power);
		}


	}
};

// The simplest strategy that I can think about with two pod is to use one for blocking the opponents
class Blocker : public Ally
{
public:

	Blocker(Terrain& _terrain) : Ally(_terrain) {}

	// this function control the pod
	void Play(const Opponent& op1, const Opponent& op2) {

		string power;

		power = "100";

		if (IsOpponentTooClose(op1) && Point(speed.x - op1.speed.x, speed.y - op1.speed.y).Norme() > 300)
			power = "SHIELD";

		else if (IsOpponentTooClose(op2) && Point(speed.x - op2.speed.x, speed.y - op2.speed.y).Norme() > 300)
			power = "SHIELD";

		GoTo(op1.position, power);
	}
};



int main()
{
	Terrain terrain = Terrain(); // this read the input data

	// game loop
	while (1) {

		// those read the input, so the order is importants
		Racer pod1 = Racer(terrain);
		Blocker pod2 = Blocker(terrain);


		Opponent opponent1 = Opponent(terrain);
		Opponent opponent2 = Opponent(terrain);

		// order is also important here
		pod1.Play(opponent1, opponent2);
		pod2.Play(opponent1, opponent2);

	}
}