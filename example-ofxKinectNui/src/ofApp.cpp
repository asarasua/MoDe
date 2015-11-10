#include "ofApp.h"
#include "ofxKinectNuiDraw.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofxKinectNui::InitSetting initSetting;
	initSetting.grabVideo = true;
	initSetting.grabDepth = true;
	initSetting.grabAudio = false;
	initSetting.grabLabel = true;
	initSetting.grabSkeleton = true;
	initSetting.grabCalibratedVideo = true;
	initSetting.grabLabelCv = false;
	initSetting.videoResolution = NUI_IMAGE_RESOLUTION_640x480;
	initSetting.depthResolution = NUI_IMAGE_RESOLUTION_640x480;
	kinect.init(initSetting);
//	kinect.setMirror(false); // if you want to get NOT mirror mode, uncomment here
//	kinect.setNearmode(true); // if you want to set nearmode, uncomment here
	kinect.open();
    
    //kinect.addKinectListener(this, &ofApp::kinectPlugged, &ofApp::kinectUnplugged);
	
#ifdef USE_TWO_KINECTS
	// watch out that only the first kinect can grab label and skeleton.
	kinect2.init(true, true, false, false, false, false, false, true);
	kinect2.open();
#endif
	ofSetVerticalSync(true);

	kinectSource = &kinect;
	angle = kinect.getCurrentAngle();
	bRecord = false;
	bPlayback = false;
	bPlugged = kinect.isConnected();
	nearClipping = kinect.getNearClippingDistance();
	farClipping = kinect.getFarClippingDistance();
	
	bDrawVideo = false;
	bDrawDepthLabel = false;
	bDrawSkeleton = false;
	bDrawCalibratedTexture = false;

	featExtractor.setup(NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SPINE);

	ofSetFrameRate(60);
	
	calibratedTexture.allocate(kinect.getDepthResolutionWidth(), kinect.getDepthResolutionHeight(), GL_RGB);

	videoDraw_ = ofxKinectNuiDrawTexture::createTextureForVideo(kinect.getVideoResolution());
	depthDraw_ = ofxKinectNuiDrawTexture::createTextureForDepth(kinect.getDepthResolution());
	labelDraw_ = ofxKinectNuiDrawTexture::createTextureForLabel(kinect.getDepthResolution());
	skeletonDraw_ = new ofxKinectNuiDrawSkeleton();
	kinect.setVideoDrawer(videoDraw_);
	kinect.setDepthDrawer(depthDraw_);
	kinect.setLabelDrawer(labelDraw_);
	kinect.setSkeletonDrawer(skeletonDraw_);
    
    ofSetWindowShape(640, 480);
    
    font.loadFont("verdana.ttf", 18);
    
	joint = NUI_SKELETON_POSITION_HAND_RIGHT;
    feature = VELOCITY_MEAN;
}

//--------------------------------------------------------------
void ofApp::update(){
    kinectSource->update();

	ofPoint* skeletonPoints[ofxKinectNui::SKELETON_COUNT];
	kinect.getRawSkeletonPoints(skeletonPoints);
	if (kinect.isFoundSkeleton())
	{
		for(int i = 0; i < ofxKinectNui::SKELETON_COUNT; i++){
			if(kinect.isTrackedSkeleton(i)){
				map<int, ofPoint> joints;
				for (int j = 0; j < ofxKinectNui::SKELETON_POSITION_COUNT; j++)
				{
					joints[j] = skeletonPoints[i][j];
				}
				featExtractor.update(joints);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0, 0, 0);
    ofSetColor(ofColor::white);
    float w = ofGetWidth(); //w
    float h = ofGetHeight(); //h
    
    //image
	kinect.drawVideo(0, 0, 640, 480);
    kinect.drawSkeleton(0, 0, 640, 480);
    
    ostringstream os;
    os << "ofxKinectFeatures example " << endl;
    os << "FPS: " << ofGetFrameRate() << endl;
	for(int i = 0; i < ofxKinectNui::SKELETON_COUNT; i++){
		if (kinect.isTrackedSkeleton(i))
		{
			ofVec2f jointProjectivePosition = getProjectiveCoordinateFor(featExtractor.getPosition(joint));
			os << "Quantity of Motion: " << featExtractor.getQom() << endl;
			//os << "Symmetry: " << featExtractor.getSymmetry() << endl;
			os << "Contraction Index: " << featExtractor.getCI() << endl << endl;
			os << "Current joint (left-right to change): ";
			switch (joint)
			{
				case NUI_SKELETON_POSITION_HIP_CENTER:
					os << "HIP CENTER" << endl;
					break;
				case NUI_SKELETON_POSITION_SPINE:
					os << "SPINE" << endl;
					break;
				case NUI_SKELETON_POSITION_SHOULDER_CENTER:
					os << "SHOULDER CENTER" << endl;
					break;
				case NUI_SKELETON_POSITION_HEAD:
					os << "HEAD" << endl;
					break;
				case NUI_SKELETON_POSITION_SHOULDER_LEFT:
					os << "LEFT SHOULDER" << endl;
					break;
				case NUI_SKELETON_POSITION_ELBOW_LEFT:
					os << "LEFT ELBOW" << endl;
					break;
				case NUI_SKELETON_POSITION_WRIST_LEFT:
					os << "LEFT WRIST" << endl;
					break;
				case NUI_SKELETON_POSITION_HAND_LEFT:
					os << "LEFT HAND" << endl;
					break;
				case NUI_SKELETON_POSITION_SHOULDER_RIGHT:
					os << "RIGHT SHOULDER" << endl;
					break;
				case NUI_SKELETON_POSITION_ELBOW_RIGHT:
					os << "RIGHT ELBOW" << endl;
					break;
				case NUI_SKELETON_POSITION_WRIST_RIGHT:
					os << "RIGHT WRIST" << endl;
					break;
				case NUI_SKELETON_POSITION_HAND_RIGHT:
					os << "RIGHT HAND" << endl;
					break;
				case NUI_SKELETON_POSITION_HIP_LEFT:
					os << "LEFT HIP" << endl;
					break;
				case NUI_SKELETON_POSITION_KNEE_LEFT:
					os << "LEFT KNEE" << endl;
					break;
				case NUI_SKELETON_POSITION_ANKLE_LEFT:
					os << "LEFT ANKLE" << endl;
					break;
				case NUI_SKELETON_POSITION_FOOT_LEFT:
					os << "LEFT FOOT" << endl;
					break;
				case NUI_SKELETON_POSITION_HIP_RIGHT:
					os << "RIGHT HIP" << endl;
					break;
				case NUI_SKELETON_POSITION_KNEE_RIGHT:
					os << "RIGHT KNEE" << endl;
					break;
				case NUI_SKELETON_POSITION_ANKLE_RIGHT:
					os << "RIGHT ANKLE" << endl;
					break;
				case NUI_SKELETON_POSITION_FOOT_RIGHT:
					os << "RIGHT FOOT" << endl;
					break;
			default:
				break;
			}
			os << "Current feature (up-down to change): ";
			switch (feature) {
				case VELOCITY_MEAN:
					os << "Velocity magnitude mean" << endl;
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
    
	ofSetColor(0,0,0,100);
	ofRect(10, 10, 500, 150);
	ofSetColor(255,255,255);
    ofDrawBitmapString(os.str(), 20, 30);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case OF_KEY_RIGHT:
            if (joint < NUI_SKELETON_POSITION_COUNT-1) {
                joint++;
            }
            break;
        case OF_KEY_LEFT:
            if (joint > 0){
                joint--;
            }
            break;
        case OF_KEY_UP:
            if (feature < RELPOSTOTORSO_X) {
                feature++;
            }
            break;
        case OF_KEY_DOWN:
            if (feature > 0){
                feature--;
            }
        default:
            break;
    }
}

//--------------------------------------------------------------
ofVec2f ofApp::getProjectiveCoordinateFor(ofPoint worldCoordinate){
	ofVec2f projectivePos;
	projectivePos.x = (worldCoordinate.x / ( (float)(320.0f/kinect.getDepthResolutionWidth()) * worldCoordinate.z * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240) ) + kinect.getDepthResolutionWidth()/2.0f;
	projectivePos.y = ofGetWindowHeight() - ((worldCoordinate.y / ( (float)(320.0f/kinect.getDepthResolutionHeight()) * worldCoordinate.z * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240) ) + kinect.getDepthResolutionHeight()/2.0f);
	return projectivePos;
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