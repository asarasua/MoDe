#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initColorSource();
	kinect.initBodySource();

	//kinect.addKinectListener(this, &ofApp::kinectPlugged, &ofApp::kinectUnplugged);

	featExtractor.setup(JointType_Head, JointType_SpineMid);

	ofSetWindowShape(1080, 720);

	font.loadFont("verdana.ttf", 18);

	joint = JointType_HandRight;
	feature = VELOCITY_MEAN;
}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	for (auto & body : kinect.getBodySource()->getBodies())
	{
		if (body.tracked)
		{
			map<int, ofPoint> joints;
			for (auto joint : body.joints)
			{
				joints[joint.first] = joint.second.getPosition();
			}
			featExtractor.update(joints);
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);
	ofSetColor(ofColor::white);
	float w = ofGetWidth(); //w
	float h = ofGetHeight(); //h

							 //image
	kinect.getColorSource()->draw(0, 0, w, h);
	kinect.getBodySource()->drawProjected(0, 0, w, h);

	ostringstream os;
	os << "ofxKinectFeatures example " << endl;
	os << "FPS: " << ofGetFrameRate() << endl;

	for (auto & body : kinect.getBodySource()->getBodies())
	{
		if (body.tracked) {
			os << "Quantity of Motion: " << featExtractor.getQom() << endl;
			//os << "Symmetry: " << featExtractor.getSymmetry() << endl;
			os << "Contraction Index: " << featExtractor.getCI() << endl << endl;
			os << "Current joint (left-right to change): ";
			switch (joint)
			{
			case JointType_SpineMid:
				os << "SPINE" << endl;
				break;
			case JointType_SpineShoulder:
				os << "SPINE SHOULDER" << endl;
				break;
			case JointType_Head:
				os << "HEAD" << endl;
				break;
			case JointType_ShoulderLeft:
				os << "LEFT SHOULDER" << endl;
				break;
			case JointType_ElbowLeft:
				os << "LEFT ELBOW" << endl;
				break;
			case JointType_WristLeft:
				os << "LEFT WRIST" << endl;
				break;
			case JointType_HandLeft:
				os << "LEFT HAND" << endl;
				break;
			case JointType_ShoulderRight:
				os << "RIGHT SHOULDER" << endl;
				break;
			case JointType_ElbowRight:
				os << "RIGHT ELBOW" << endl;
				break;
			case JointType_WristRight:
				os << "RIGHT WRIST" << endl;
				break;
			case JointType_HandRight:
				os << "RIGHT HAND" << endl;
				break;
			case JointType_HipLeft:
				os << "LEFT HIP" << endl;
				break;
			case JointType_KneeLeft:
				os << "LEFT KNEE" << endl;
				break;
			case JointType_AnkleLeft:
				os << "LEFT ANKLE" << endl;
				break;
			case JointType_FootLeft:
				os << "LEFT FOOT" << endl;
				break;
			case JointType_HipRight:
				os << "RIGHT HIP" << endl;
				break;
			case JointType_KneeRight:
				os << "RIGHT KNEE" << endl;
				break;
			case JointType_AnkleRight:
				os << "RIGHT ANKLE" << endl;
				break;
			case JointType_FootRight:
				os << "RIGHT FOOT" << endl;
				break;
			default:
				break;
			}


			ofxKinectForWindows2::Data::Joint j = body.joints.at((JointType)joint);
			ofVec2f jointProjectivePosition = j.getProjected(kinect.getBodySource()->getCoordinateMapper());
			jointProjectivePosition.x *= w / kinect.getColorSource()->getWidth();
			jointProjectivePosition.y *= h / kinect.getColorSource()->getHeight();
			os << "Current feature (up-down to change): ";
			ofSetColor(255);
			switch (feature) {
			case VELOCITY_MEAN:
				os << "Velocity magnitude mean" << endl;
				os << jointProjectivePosition << endl;
				//font.drawString(ofToString(featExtractor.getSkeleton(0)->getVelocityMean(joint)), jointProjectivePosition.x, jointProjectivePosition.y);
				font.drawString(ofToString(featExtractor.getVelocityMean(joint)), jointProjectivePosition.x, jointProjectivePosition.y);
				break;
			case ACCELERATION_Y:
				os << "Acceleration along y axis (up-down movement)" << endl;
				//font.drawString(ofToString(featExtractor.getSkeleton(0)->getAcceleration((Joint)j).y), jointProjectivePosition.x, jointProjectivePosition.y);
				font.drawString(ofToString(featExtractor.getAcceleration(joint).y), jointProjectivePosition.x, jointProjectivePosition.y);
				break;
			case RELPOSTOTORSO_X:
				os << "Relative position to torso in x axis" << endl;
				//font.drawString(ofToString(featExtractor.getSkeleton(0)->getRelativePositionToTorso((Joint)j).x), jointProjectivePosition.x, jointProjectivePosition.y);
				font.drawString(ofToString(featExtractor.getRelativePositionToTorso(joint).x), jointProjectivePosition.x, jointProjectivePosition.y);
				break;
			default:
				break;
			}
		}
	}

	ofSetColor(0, 0, 0, 100);
	ofRect(10, 10, 500, 150);
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(os.str(), 20, 30);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case OF_KEY_RIGHT:
		if (joint < JointType_Count - 1) {
			joint++;
		}
		break;
	case OF_KEY_LEFT:
		if (joint > 0) {
			joint--;
		}
		break;
	case OF_KEY_UP:
		if (feature < RELPOSTOTORSO_X) {
			feature++;
		}
		break;
	case OF_KEY_DOWN:
		if (feature > 0) {
			feature--;
		}
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}