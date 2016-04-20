#pragma once

#include "ofMain.h"
#include "ofxKinectFeatures.h"
#include "SynapseStreamer.h"

enum {
    VELOCITY_MEAN,
    ACCELERATION_Y,
    FEAT_COUNT
};

enum {
    RIGHT_HAND,
    LEFT_HAND,
    JOINT_COUNT
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
    void mocapMax(MocapMaxEvent &e);
    string getJointAsString(int j);
    
    
    bool hadUsers;
    ofxKinectFeatures featExtractor;
    int j, f;
    ofTrueTypeFont font;
    
    vector< double > leftHand;
    vector< double > rightHand;
    
    unsigned int beatInRH, beatInLH;
    
    SynapseStreamer synapseStreamer;
};
