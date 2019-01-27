#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <sstream>


//---------------------------------------------------------------------------------------
//-------------------------     Tweakable parameters     --------------------------------
//---------------------------------------------------------------------------------------

// define the intensity of speed compensation for trajectory correction 
#define TRAJECTORY_CORRECTION_INTENSITY 0.008f // proportionel

// the minimum angle between speed and checkpoint 
// for wich we consider that the pod is on the right direction
#define MINIMUM_GOOD_ANGLE 25 // proportionel

// When we get close to a checkpoint, we must start turning toward the next 
// one to benefit off the inertia. This create a 'drift effect' but sometimes
// we can be to short to trigger the checkpoint, so this define set the level of safety we add
// too low we miss, too high we waste time
#define DRIFTING_SAFEFTY 0.2f // probably not exactly proportionel

// The effective (using referential) speed threshold for detecting 'high energy' collisions
#define COLLISION_THRESHOLD 200 // Proportionel

// the minimal distance between the pod and the checkpoint to activate the boost
#define BOOST_MIN_DISTANCE 2500

// The maximal error angle between the pod and checkpoint to activate the boost
#define BOOST_MAX_ANGLE 5

//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------


using namespace std;

// This will be way more simple with a Point structure
class Point
{
public:

	Point() : x(0.0f), y(0.0f) {}

	Point(float _x, float _y) : x(_x), y(_y) {}

	bool operator==(const Point& other)const { return (x == other.x && y == other.y); }
	bool operator!=(const Point& other)const { return (x != other.x || y != other.y); }

	void operator+=(const Point& other) { x += other.x; y += other.y; }
	void operator-=(const Point& other) { x -= other.x; y -= other.y; }
	void operator*=(float multiplier) { x *= multiplier; y *= multiplier; }
	void operator/=(float multiplier) { x /= multiplier; y /= multiplier; }

	Point operator+(const Point& other)const { return Point(x + other.x, y + other.y); }
	Point operator-(const Point& other)const { return Point(x - other.x, y - other.y); }
	Point operator*(float multiplier)const { return Point(x*multiplier, y*multiplier); }
	Point operator/(float multiplier)const { return Point(x / multiplier, y / multiplier); }

	// a vector operation but same thing
	float Norme() const { return sqrtf(x*x + y*y); }

	float x;
	float y;
};


float RadToDeg(float rad) {
	return (rad * 180.0f) / M_PI;
}

float DegToRad(int deg) {
	return (deg * M_PI) / 180.0f;
}


// No API to do that apparently, like the given angle this return an angle between -180 and 180
float GetAngle(const Point& vector) {

	float alpha = atanf(vector.y / vector.x);

	if (vector.x < 0)
		alpha += M_PI; // to be on a 2pi range instead of 1pi (-90/270)

	if (alpha > M_PI) // to limit between -180/180 instead of -90 270
		alpha -= 2 * M_PI;

	return RadToDeg(alpha); // degres because the game gives degres
}


// return the angle between two vectors
// this is limited to -180 - 180
float GetAngleBetweenTwoVectors(const Point& reference, const Point& vector) {

	float alpha1 = GetAngle(vector);

	float alpha2 = GetAngle(reference);

	float diff = alpha1 - alpha2;


	if (diff > 180)
		diff -= 360;

	if (diff < -180)
		diff += 360;

	return diff;
}



Point GetVectorFromAngle(float angle, float norme = 1.0f) {

	float radAngle = DegToRad(angle);
	return Point(norme*cosf(radAngle), norme*sinf(radAngle));
}

// very classic computation of the distance between two points
float Distance(const Point& vector1, const Point& vector2) {

	Point p = vector1 - vector2;
	return sqrtf(p.x*p.x + p.y*p.y);
}



class Terrain
{
public:

	// just the basic reading
	static void InitTerrain() {

		cin >> laps; cin.ignore();
		cin >> nbCheckpoints; cin.ignore();
		for (int i = 0; i < nbCheckpoints; i++) {
			Point p;
			cin >> p.x >> p.y; cin.ignore();
			checkpoints.push_back(p);
		}
	}

	static int laps;
	static int nbCheckpoints;
	static std::vector<Point> checkpoints;
};

int Terrain::laps;
int Terrain::nbCheckpoints;
std::vector<Point> Terrain::checkpoints;





class Pod
{
public:


	Pod() { }

	// apparently allies and opponents receive the same infos so it is better here
	virtual void Read() {
		cin >> position.x >> position.y >> speed.x >> speed.y >> angle >> nextCheckpoint; cin.ignore();

		if (angle > 180) // Why did they change that ?? -180 < angle < 180 is far better
			angle -= 360;
	}

	// Get the angle between the direction of the pod and a point
	float GetPodAngleWithSomething(const Point& something) const {
		return GetAngleBetweenTwoVectors(GetVectorFromAngle(angle), something - position);
	}

	// GetCheckpoint mean current destination 
	Point GetCheckpoint() const {
		return Terrain::checkpoints[nextCheckpoint];
	}

	// This will be more detailled in the report
	float GetStopDistance() const {
		// 5.666666 is the result of a simple convergent serie since friction is proportional
		return speed.Norme() * (5.66666f - DRIFTING_SAFEFTY);
	}

	Point speed;
	Point position;
	float angle;
	int nextCheckpoint;
};



class Opponent : public Pod
{
public:

	Opponent() : Pod(), currentLap(1), endLap(false) {}

	// override read to update the current lap of the opponent
	void Read() {
		Pod::Read();
		if (nextCheckpoint == 0)
			endLap = true;
		else if (endLap) {
			currentLap++;
			endLap = false;
		}
	}

	// return the total quantity of checkpoints triggered by this opponent
	int GetCheckpointCount() const {
		return ((currentLap - 1 + endLap) * Terrain::nbCheckpoints) + (nextCheckpoint - 1);
	}

	// return a point from which we can easily block this opponent
	Point GetGoodPointToAttack() const {
		// for now very simple
		return Terrain::checkpoints[(nextCheckpoint + 1) % Terrain::checkpoints.size()];
	}

	int currentLap;
	bool endLap; // for comparaison with nextCheckpoint purpose only


	static Opponent* opponent1;
	static Opponent* opponent2;
};

Opponent* Opponent::opponent1;
Opponent* Opponent::opponent2;



class Ally : public Pod
{
public:

	Ally() : Pod() {}


	// the previous algorithm wich worked very well
	// It correct the direction of the speed vector by targeting a point next to the checkpoint
	Point TrajectoryCorrection_old(const Point& dest) const {

		Point toTarget = dest - position;

		float l_angle = GetAngleBetweenTwoVectors(toTarget, speed);

		Point normal; // this is THE trajectory correction vector 

		if (l_angle >= 0)
		{
			normal = Point(-toTarget.y, toTarget.x); // the 'right' normal
		}
		else
		{
			normal = Point(toTarget.y, -toTarget.x); // the 'left' normal
		}

		// This is very important, it define how far we offset the target from the checkpoint
		// The current operation is not convincing 
		float multiplier = abs(l_angle)*TRAJECTORY_CORRECTION_INTENSITY;

		normal *= multiplier;

		Point target = dest;

		if (speed.Norme() > 1)
			target += normal;

		return target;
	}

	Point TrajectoryCorrection(const Point& dest) const {

		if (speed.Norme() < 1)
			return dest;

		Point toTarget = dest - position;

		float alpha = GetAngleBetweenTwoVectors(toTarget, speed);

		if (abs(alpha) > 60)
			return dest;

		float beta = GetAngle(toTarget);

		float gamma = beta - alpha; //beta + (-alpha)

		Point mirrorSpeedError = GetVectorFromAngle(gamma, speed.Norme());

		return position + mirrorSpeedError;
	}



	void CopyForSimulation(const Pod* other) {

		angle = other->angle;
		speed = other->speed;
		position = other->position;
		nextCheckpoint = other->nextCheckpoint;
	}

	//return the number of degre that the simulated pod must be rotate
	float Turn_Simulation(const Point& target) const {
		float l_angle = GetPodAngleWithSomething(target);
		return (l_angle < 0.0f ? -1.0f : 1.0f) * (abs(l_angle) > 18 ? 18 : l_angle);
	}


	void Simulate(const Point& destination, const string& power) {

		int intPower;
		if (power == "BOOST")
			intPower = 100;
		else if (power == "SHIELD")
			intPower = 0;
		else {
			istringstream iss(power);
			iss >> intPower;
		}

		// Game engine simulation
		angle += Turn_Simulation(destination);
		Point acceleration = GetVectorFromAngle(angle, (float)intPower);
		speed += acceleration;
		position += speed;
		speed *= 0.85f;
		position = Point(int(position.x), (int)position.y);
		speed = Point(int(speed.x), (int)speed.y);

		if (Distance(position, Terrain::checkpoints[nextCheckpoint]) < 600) {
			nextCheckpoint = (nextCheckpoint + 1) % Terrain::checkpoints.size();
		}
	}

	void GoTo(const Point& dest, const string& power, bool isSimulation) {

		Point target = TrajectoryCorrection(dest);

		if (isSimulation)
			Simulate(target, power);
		else
			cout << (int)target.x << " " << (int)target.y << " " << power << endl;
	}

	// GetNextCheckpoint mean the point after my current destination 
	Point GetNextCheckpoint() const {

		if (nextCheckpoint == Terrain::checkpoints.size() - 1)
			return Terrain::checkpoints[0];

		return Terrain::checkpoints[nextCheckpoint + 1];
	}


	// the algorithm to find if we must turn on the shield, because a important collision will append
	bool WillHitHardOpponent(const Opponent* op) {

		Point NextPos = position + speed;
		Point NextOpponentPos = op->position + op->speed;

		// collision detection
		if (Distance(NextPos, NextOpponentPos) < 800) { // pod have raduis of 400 so collison dist: 400*2

														//this is not enought, we only accept 'high energy' collisions
														// because shut down the engine for an insignificant trajectory change is not smart
			Point collisionSpeed = speed - op->speed;

			if (collisionSpeed.Norme() > COLLISION_THRESHOLD)
				return true;
		}
		return false;
	}

	// since the begging those 5 simple line seems to be incredibly efficient
	const string GetPowerForTarget(const Point& target) const {

		float l_angle = abs(GetPodAngleWithSomething(target));

		if (l_angle > 90)
			return "0";

		return "100";
	}

};


class Racer : public Ally
{
public:

	Racer() : Ally() {}

	void Read() {
		Ally::Read();
		cerr << position.x << " " << position.y << "    " << speed.x << " " << speed.y << "   " << angle << endl;
	}


	bool ShouldBoost() const {

		float l_angle = abs(GetPodAngleWithSomething(GetCheckpoint()));

		// nextCheckpoint != 1 avoid wasting the boost at the beginning since that what the boss does
		// It often result by a collision which make the boost useless
		// Against other players that not a problem however
		if (GetCheckpoint().Norme() > BOOST_MIN_DISTANCE &&
			l_angle < BOOST_MAX_ANGLE &&
			nextCheckpoint != 1)
			return true;

		return false;
	}


	// get if the speed of the pod is correct enought to go in 'drift mod'
	bool ShouldDrift_SimulationComputation() const {

		cerr << "Simulation" << endl;

		Racer simulatedRacer = Racer();

		simulatedRacer.CopyForSimulation(this);

		for (int i = 0; i<7; i++)
		{
			//simulatedRacer.play();

			//cerr <<  simulatedRacer.position.x << " " << posSim.y<< "    " << speedSim.x << " " << speedSim.y << "   " << angleSim << endl;

			// simulate drift
			if (Distance(simulatedRacer.position, GetCheckpoint()) < 600) { //600 : checkpoints size
				return true;
			}
		}

		return false;
	}


	bool ShouldDrift_SpeedComputation() const {

		if (Distance(GetCheckpoint(), position) < GetStopDistance()) {// we admit that we can drift

			int simAngle = angle;
			Point simPos = position;
			Point simSpeed = speed;

			for (int i = 0; i<7; i++) {


				float angleWithNextCheckpoint = abs(GetAngleBetweenTwoVectors(GetVectorFromAngle(simAngle), GetNextCheckpoint() - simPos));

				Point acceleration = Point();
				if (angleWithNextCheckpoint < 90)
					acceleration = GetVectorFromAngle(simAngle, 100.0f);

				simAngle += Turn_Simulation(GetNextCheckpoint());
				simSpeed += acceleration;
				simPos += simSpeed;
				simSpeed *= 0.85f;
				simPos = Point(int(simPos.x), (int)simPos.y);
				simSpeed = Point(int(simSpeed.x), (int)simSpeed.y);

				//cerr <<  simPos.x << " " << simPos.y<< "    " << simSpeed.x << " " << simSpeed.y << "   " << simAngle << endl;


				if (Distance(simPos, GetCheckpoint()) < 600) //600 : checkpoints size
					return true;

			}

		}
		return false;
	}


	// probably better as polymorphic function but here we do not need it
	// this function control the pod
	void Play(bool isSimulation = false) {

		string power;

		if (ShouldDrift_SpeedComputation()) // ShouldDrift_SimulationComputation | ShouldDrift_SpeedComputation()
		{
			power = GetPowerForTarget(GetNextCheckpoint());
			GoTo(GetNextCheckpoint(), power, isSimulation);
		}
		else
		{
			power = GetPowerForTarget(GetCheckpoint()); // low priority behaviour

			if (ShouldBoost()) // higher priority behaviour
				power = "BOOST";

			if (WillHitHardOpponent(Opponent::opponent1) || WillHitHardOpponent(Opponent::opponent2)) // top priority behaviour
				power = "SHIELD";

			GoTo(GetCheckpoint(), power, isSimulation);
		}
	}

	static Racer* racer;
};

Racer* Racer::racer;

// The simplest strategy that I can think about with two pod is to use one for blocking the opponents
class Blocker : public Ally
{
public:

	Blocker() : Ally(), state_readyToHit(false) {}

	int GetTimeToGo(const vector<int>& distances, const Point& target)const {

		int toTarget = Distance(target, position);
		for (int i = 1; i<distances.size(); i++) {

			if (distances[i] > toTarget) {
				if (distances[i] - toTarget < toTarget - distances[i - 1])
					return i;
				return i - 1;
			}
		}
		return distances.size();
	}

	bool IsSpeedGloballyTowardTarget(const Point& target) const {
		return (abs(GetAngleBetweenTwoVectors(speed, target - position)) < 90);
	}

	// get the best point on a trajectory to intersect the opponent
	Point GetBestIntersection(const vector<Point>& trajectory, const Point& target)const {

		vector<int>distances;
		distances.push_back(0);
		int simSpeed = speed.Norme() * (IsSpeedGloballyTowardTarget(target) ? 1.0f : -1.0f);
		for (int i = 0; i<20; i++) { // I do not think this is a simplifiable suite
			simSpeed = (simSpeed + 100)*0.85f;
			distances.push_back(distances[i] + simSpeed);
			cerr << "inters : " << distances[distances.size() - 1] << endl;
		}
		// now we know the distance which we can travel in a certain time

		for (int i = 0; i<trajectory.size(); i++) { // This is O(n²) but optimizable

			if (GetTimeToGo(distances, trajectory[i]) == i) {
				return trajectory[i];
			}
		}
		return trajectory[trajectory.size() - 1];
	}

	Point InterpolateIntersection(const Opponent* op) const {

		vector<Point> trajectory;

		Racer simulatedOp = Racer();
		simulatedOp.CopyForSimulation(op);

		for (int i = 0; i<20; i++) {
			simulatedOp.Play(true); // play as simulation
			trajectory.push_back(simulatedOp.position);
		}

		return GetBestIntersection(trajectory, op->position);
	}

	// this function control the pod
	void Play(bool isSimulation = false) {

		// we find the best opponent to try to block it
		const Opponent* bestTarget;

		if (Opponent::opponent1->GetCheckpointCount() >= Opponent::opponent2->GetCheckpointCount()) // == does not matter
			bestTarget = Opponent::opponent1;
		else
			bestTarget = Opponent::opponent2;

		string power;

		if (WillHitHardOpponent(Opponent::opponent1) || WillHitHardOpponent(Opponent::opponent2)) {
			power = "SHIELD";
			state_readyToHit = false;
			GoTo(bestTarget->position, power, isSimulation); // target does not matter only SHIELD
		}
		else { // if we do not hit now we try some smart moves to be a good blocker

			if (!state_readyToHit) {

				Point target = position + ((position - bestTarget->position)* 100.0f);

				if (Distance(position, bestTarget->position) > 8000) {
					state_readyToHit = true;
				}
				else {
					power = GetPowerForTarget(target);
					GoTo(target, power, isSimulation);
				}
			}
			// not an else because of the possible state modifcation
			if (state_readyToHit) {
				Point target = InterpolateIntersection(bestTarget);
				cerr << "best : " << bestTarget->position.x << " " << bestTarget->position.y << endl;
				power = GetPowerForTarget(target);
				GoTo(target, power, isSimulation);
			}
		}
	}

	bool state_readyToHit;

	static Blocker* blocker;
};

Blocker* Blocker::blocker;



int main()
{
	Terrain::InitTerrain(); // this read the input data

							// now the pod are 'static' to allow using states
	Racer::racer = new Racer();
	Blocker::blocker = new Blocker();

	Opponent::opponent1 = new Opponent();
	Opponent::opponent2 = new Opponent();

	// game loop
	while (1) {

		// those read the input, so the order is importants
		Racer::racer->Read();
		Blocker::blocker->Read();

		Opponent::opponent1->Read();
		Opponent::opponent2->Read();


		// order is also important here
		Racer::racer->Play();
		Blocker::blocker->Play();

	}

	delete Racer::racer;
	delete Blocker::blocker;
	delete Opponent::opponent1;
	delete Opponent::opponent2;
}