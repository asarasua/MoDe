#pragma once

#include "ofMain.h"
#include "KinectFeatures.h"
#include "ofxKinectFeaturesGraph.h"
#include "ofxOpenNI.h"

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
    void userEvent(ofxOpenNIUserEvent & event);
    void mocapExtreme(MocapEvent &e);
    
    
    ofxOpenNI kinect;
    bool hadUsers;
    KinectFeatures featExtractor;
    int j, f;
    ofTrueTypeFont font;
    
    vector<ofPtr<ofxKinectFeaturesGraph>> graphs;
};
