#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    hadUsers = false;
    
    //Open the connection with Synapse
    synapseStreamer.openSynapseConnection();
    
    //Set which joints we want to track
    synapseStreamer.trackAllJoints(false);
    synapseStreamer.trackLeftHand(true);
    synapseStreamer.trackRightHand(true);
    
    //featExtractor.setup(head, torso);
    
    ofAddListener(MocapMaxEvent::events, this, &ofApp::mocapMax);
    
    ofSetWindowShape(700, 200);
    ofSetBackgroundColor(0);
    
    font.load("verdana.ttf", 18);
    
    j = RIGHT_HAND;
    f = VELOCITY_MEAN;

    beatInLH = 0;
    beatInRH = 0;
    
}

//--------------------------------------------------------------
void ofApp::update(){
    //Parse any new messages from the synapse streamer
    synapseStreamer.parseIncomingMessages();
    
    if( synapseStreamer.getNewMessage() ){
        //Get the left hand and right hand joings
        ofPoint (synapseStreamer.getLeftHandJointBody()[0], synapseStreamer.getLeftHandJointBody()[1], synapseStreamer.getLeftHandJointBody()[2]);
        ofPoint (synapseStreamer.getRightHandJointBody()[0], synapseStreamer.getRightHandJointBody()[1], synapseStreamer.getRightHandJointBody()[2]);
        
        map<int, ofPoint> joints;
        joints[RIGHT_HAND] = ofPoint (synapseStreamer.getLeftHandJointBody()[0], synapseStreamer.getLeftHandJointBody()[1], synapseStreamer.getLeftHandJointBody()[2]);
        joints[LEFT_HAND] = ofPoint (synapseStreamer.getRightHandJointBody()[0], synapseStreamer.getRightHandJointBody()[1], synapseStreamer.getRightHandJointBody()[2]);
        featExtractor.update(joints);
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(ofColor::white);
    float w = ofGetWidth(); //w
    float h = ofGetHeight(); //h
    
    ofSetColor(255,255,255);
    
    ostringstream os;
    os << "ofxKinectFeatures example " << endl;
    os << "FPS: " << ofGetFrameRate() << endl;
    os << "Quantity of Motion: " << featExtractor.getQom() << endl;
    //os << "Symmetry: " << featExtractor.getSymmetry() << endl;
    os << "Contraction Index: " << featExtractor.getCI() << endl << endl;
    os << "Current joint (left-right to change): " << getJointAsString(j) << endl;
    os << "Current feature (up-down to change): ";
    switch (f) {
        case VELOCITY_MEAN:
            os << "Velocity magnitude mean = " << featExtractor.getVelocityMagnitudeMean(j);
            break;
        case ACCELERATION_Y:
            os << "Acceleration along y axis (up-down movement)" << featExtractor.getAcceleration(j).y;
            break;
        default:
            break;
    }
    
    //Draw circles for every beat in Y axis from the hands
    if (ofGetElapsedTimeMillis() - beatInLH < 1000)
        ofDrawCircle(500, 150, 20 * (float)(1000.0 - (ofGetElapsedTimeMillis() - beatInLH)) / 1000.0);
    if (ofGetElapsedTimeMillis() - beatInRH < 1000)
        ofDrawCircle(200, 150, 20 * (float)(1000.0 - (ofGetElapsedTimeMillis() - beatInRH)) / 1000.0);
    
    ofDrawBitmapString(os.str(), 20, 30);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case OF_KEY_RIGHT:
            if (j < JOINT_COUNT-1) {
                j++;
            }
            break;
        case OF_KEY_LEFT:
            if (j > 0){
                j--;
            }
            break;
        case OF_KEY_UP:
            if (f < FEAT_COUNT-1) {
                f++;
            }
            break;
        case OF_KEY_DOWN:
            if (f > 0){
                f--;
            }
        default:
            break;
    }
}

void ofApp::mocapMax(MocapMaxEvent &e){
    if (e.joint == RIGHT_HAND && e.feature == FEAT_ACCELERATION && e.axis == MOCAP_Y && e.value > 5.0) {
        beatInRH = ofGetElapsedTimeMillis();
    } else if (e.joint == LEFT_HAND && e.feature == FEAT_ACCELERATION && e.axis == MOCAP_Y && e.value > 5.0) {
        beatInLH = ofGetElapsedTimeMillis();
    }
}

string ofApp::getJointAsString(int j){
    string jointString = "";
    switch (j) {
        case RIGHT_HAND:
            jointString = "RIGHT HAND";
            break;
        case LEFT_HAND:
            jointString = "LEFT HAND";
        default:
            break;
    }
    return jointString;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}