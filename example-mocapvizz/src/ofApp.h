#pragma once

#include "ofMain.h"
#include "ofxKinectFeatures.h"
#include "ofxOpenNI.h"
#include "particle.h"
#include <tr1/random>

enum {
    VELOCITY_MEAN,
    ACCELERATION_Y,
    RELPOSTOTORSO_X
};

#define PADDING 20
#define NUM_PARTICLES_HAND 100
#define NUM_PARTICLES_BEAT_TOTAL 1000
#define NUM_PARTICLES_BEAT_INSTANCE 200
#define HANDS_MAX_SIZE 4
#define RES_X 1024
#define RES_Y 768

class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
        void drawNiceSkeleton(float x, float y, float w, float h);
        void drawFbo();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void userEvent(ofxOpenNIUserEvent & event);
        void mocapMax(MocapMaxEvent &e);
        void mocapMin(MocapMinEvent &e);
    
    
    ofxOpenNI kinect;
    bool hadUsers;
    ofxKinectFeatures featExtractor;
    ofTrueTypeFont font;
    int fontSize;
    bool cursor;
    
    ofFbo fbo; // with alpha
    int fadeAmnt;
    float alphaControl;
    
    //Hands trajectory
    Particle rightHand[NUM_PARTICLES_HAND];
    Particle leftHand[NUM_PARTICLES_HAND];
    int hue, sat;
    ofImage vuMeter;
    
    //Beat particles
    Particle beatParticles[NUM_PARTICLES_BEAT_TOTAL];
    int particleIndex, particleSize;
    float lastRightBeat, lastLeftBeat;
    
    //Normal distribution
    tr1::mt19937 rng;
    tr1::normal_distribution<float> *normdist;
    tr1::variate_generator<tr1::mt19937, tr1::normal_distribution<float> > *rndnorm;//(rng, *normdist);
    
};
