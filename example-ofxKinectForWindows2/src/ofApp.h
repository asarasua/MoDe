#pragma once

#include "ofMain.h"
#include "mode_extractor.h"
#include "ofxModeGraph.h"
#include "ofxKinectForWindows2.h"
#include "beat.h"
#define N_BEATS 40

class ofxKinectNuiDrawTexture;
class ofxKinectNuiDrawSkeleton;

enum {
	VELOCITY_MEAN,
	ACCELERATION_Y,
	RELPOSTOTORSO_X
};

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	//void userEvent(ofxOpenNIUserEvent & event);

	void drawProjectedWithColor(int x, int y, int width, int height);

	void mocapExtreme(MoDe::ofxMoDeEvent &e);

	vector<Beat> beats;
	int beatLife;
	vector<ofPtr<ofxMoDeGraph>> graphs;

	ofxKFW2::Device kinect;
	//bool hadUsers;
	MoDe::ofxMoDe featExtractor;
	int joint, feature;
	ofTrueTypeFont font;

	bool drawMode;

	ofSoundPlayer sound;
};