#include "beat.h"
#include <random>

Particle::Particle() {
	size = ofRandom(20);

	color.set(255, 0, 50);
	strokeColor.set(0, 0, 0);

	pos.set(ofGetWindowWidth()*0.5, ofGetWindowHeight()*0.5);
	vel.set(ofRandom(-1, 1), ofRandom(-1, 1));
	acc.set(0, 0);

	alphaF = 1;
	scaleF = 1;

	damp = 0.05;

	life = initialLife = ofRandom(200, 1000);
}

/*
Update
*/
void Particle::update(float _speedLimit = 0.0) {
	vel += acc;     // Suma la aceleración a la velocidad
	vel *= damp;

	if (_speedLimit != 0)
		vel.limit(_speedLimit);

	pos += vel;     // Suma la velocidad a la posición
	acc *= 0;       // Vuelve a cero la aceleración

	life--;
}

/*
Dynamics
*/
void Particle::moveTo(ofVec3f _position) {
	pos += (_position - pos) * damp;
}


void Particle::addForce(ofVec2f _force) {
	acc += _force;
}

void Particle::addNoise(float _angle, float _turbulence) {
	float angle = ofSignedNoise(pos.x * 0.005f, pos.y * 0.005f) * _angle;
	ofVec2f noiseVector(cos(angle), sin(angle));
	acc += noiseVector * _turbulence * (1.0 - ofNormalize(life, 0, initialLife));
}

/*
Painting
*/
void Particle::addAlphaFade(bool _fadeOut = true) {
	if (_fadeOut)
		alphaF = 1.0f - ofNormalize(life, 0, initialLife);
	else
		alphaF = ofNormalize(life, 0, initialLife);
}

void Particle::addScaleFade(bool _melt = true) {
	if (_melt)
		scaleF = 1.0f - ofNormalize(life, 0, initialLife);
	else
		scaleF = ofNormalize(life, 0, initialLife);
}

void Particle::draw() {
	//Draw fill
	ofSetColor(color, color.a * alphaF);
	ofFill();
	ofCircle(pos.x, pos.y, size * scaleF);

	//Draw stroke
	ofSetColor(strokeColor, strokeColor.a * alphaF);
	ofNoFill();
	ofCircle(pos.x, pos.y, size * scaleF);
}


/*
Particle state
*/
bool Particle::crossedBorder() {
	bool crossed = false;
	if (pos.x < 0 || pos.y < 0 || pos.x > ofGetWindowWidth() || pos.y > ofGetWindowHeight())
	{
		crossed = true;
	}
	return crossed;
}

bool Particle::isAlive() {
	return life > 0;
}

Beat::Beat()
{
	for (size_t i = 0; i < N_PARTICLES; i++)
	{
		particles.push_back(Particle());
	}
}

Beat::Beat(long timeStamp, ofVec2f screenPos, ofVec2f velocity)
{
	for (size_t i = 0; i < N_PARTICLES; i++)
	{
		particles.push_back(Particle());
	}

	timeStamp_ = timeStamp;

	std::default_random_engine generator;
	std::normal_distribution<float> distribution(1.0, 0.1);

	for (auto &particle : particles) {
		particle.pos = screenPos;
		particle.alphaF = 1;
		particle.life = 200;
		particle.initialLife = 200;
		particle.size = 2;
		//beatParticles[i].color.set(250, 250, 250, 255);
		particle.damp = 0.98;
		particle.vel.x = 0.0;
		particle.vel.y = 0.0;
		//float angle = ofRandom(-45, 45);
		float angle = 360.0 * distribution(generator);
		velocity.rotate(angle);
		velocity *= distribution(generator);
		particle.addForce(velocity* 100.0);
	}
}

void Beat::update()
{
	for (auto &particle : particles) {
		particle.update(0.0);
	}
}

void Beat::draw(int x, int y, int width, int height)
{
	ofPushMatrix();
	ofPushStyle();
	ofTranslate(x, y);
	ofScale((float)width / 1920, (float)height / 1080);

	int foo = timeStamp_ - ofGetElapsedTimeMillis();
	float alpha = ofMap(foo, -3000, 0, 0, 255, true);
	for (auto particle : particles) {
		ofSetColor(alpha, alpha, 0, alpha);
		ofCircle(particle.pos.x, particle.pos.y, 2);
	}

	ofPopStyle();
	ofPopMatrix();
}

long Beat::getTimeStamp()
{
	return timeStamp_;
}
