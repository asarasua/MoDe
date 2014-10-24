#pragma once

#include "ofMain.h"
#include "ofxKinectFeatures.h"
#include "ofxKinectNui.h"
#include "ofxKinectNuiPlayer.h"
#include "ofxKinectNuiRecorder.h"

class ofxKinectNuiDrawTexture;
class ofxKinectNuiDrawSkeleton;

enum {
    VELOCITY_MEAN,
    ACCELERATION_Y,
    RELPOSTOTORSO_X
};

class ofApp : public ofBaseApp{
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
		ofVec2f getProjectiveCoordinateFor(ofPoint worldCoordinate);
    
    ofxKinectNui kinect;
	ofxBase3DVideo* kinectSource;
    //bool hadUsers;
    ofxKinectFeatures featExtractor;
    int joint, feature;
    ofTrueTypeFont font;

	ofTexture calibratedTexture;
	bool bRecord;
	bool bPlayback;
	bool bDrawVideo;
	bool bDrawDepthLabel;
	bool bDrawSkeleton;
	bool bDrawCalibratedTexture;
	bool bPlugged;
	bool bUnplugged;
		
	unsigned short nearClipping;
	unsigned short farClipping;
	int angle;
		
	int mRotationX, mRotationY;

	// Please declare these texture pointer and initialize when you want to draw them
	ofxKinectNuiDrawTexture*	videoDraw_;
	ofxKinectNuiDrawTexture*	depthDraw_;
	ofxKinectNuiDrawTexture*	labelDraw_;
	ofxKinectNuiDrawSkeleton*	skeletonDraw_;
};