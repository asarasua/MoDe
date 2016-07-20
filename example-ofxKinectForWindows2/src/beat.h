#ifndef mode_particle
#define mode_particle

#define N_PARTICLES 60

#include "ofMain.h"

class Particle {
public:
	// Constructor
	Particle();

	//Dynamics
	void moveTo(ofVec3f _position);
	void addForce(ofVec2f _force);
	void addNoise(float _angle, float _turbulence);   //adds a random subtle movement
	//Fade outs
	void addAlphaFade(bool _fadeOut);
	void addScaleFade(bool _melt);

	//State checkers
	bool crossedBorder();
	bool isAlive();

	//update
	void update(float _speedLimit);
	void draw();

	ofColor color, strokeColor;
	ofVec2f vel, acc;
	ofVec3f pos;

	float alphaF, scaleF, size;

	float damp;

	int life, initialLife;
};

class Beat {
public:
	Beat();
	Beat(long timeStamp, ofVec2f screenPos, ofVec2f velocity);
	void update();
	void draw(int x, int y, int widht, int height);
	long getTimeStamp();
private:
	long timeStamp_;
	vector<Particle> particles;
};

#endif
