#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initColorSource();
	kinect.initBodySource();
	kinect.initInfraredSource();
	featExtractor.setup(JointType_Head, JointType_SpineShoulder, 60);
	//featExtractor.setFilterLevel(MoDe::FILTER_HARD);

	ofSetFrameRate(30.0);

	drawMode = true;

	ofAddListener(MoDe::ofxMoDeEvent::events, this, &ofApp::mocapExtreme);

	sound.loadSound("met.wav");

	beatLife = 3000;
	ofPtr<ofxMoDeGraph> g1(new ofxMoDeGraph(30, 30, 400, 100, 0, "foo"));
	graphs.push_back(g1);
	ofPtr<ofxMoDeGraph> g2(new ofxMoDeGraph(30, 300, 400, 100, 100, "foo"));
	graphs.push_back(g2);
	ofPtr<ofxMoDeGraph> g3(new ofxMoDeGraph(30, 500, 400, 100, 200, "foo"));
	graphs.push_back(g3);
}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	for (auto &beat : beats)
		beat.update();
	//--
	//Getting joint positions (skeleton tracking)
	//--
	//

	if (kinect.isFrameNew()) {
		auto bodies = kinect.getBodySource()->getBodies();
		for (auto body : bodies) {
			if (body.tracked) {
				map<int, ofPoint> joints;
				for (auto joint : body.joints) {
					//now do something with the joints
					joints[joint.second.getType()] = joint.second.getPosition();
				}
				featExtractor.update(joints);
			}
		}
		featExtractor.getJoint(JointType_HandRight).getUniDescriptor(MoDe::DESC_ACCELERATION_TRAJECTORY).getCrest();
		graphs[0]->addValue(featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_ACCELERATION).getCurrent().y);
		graphs[0]->setThreshold(featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_ACCELERATION).getUpperThreshold().y);

		graphs[1]->addValue(featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_ACCELERATION).getCrest().y);

		graphs[2]->addValue(featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_ACCELERATION).getRms().y);
		//for (auto graph : graphs)
		//	addValueToGraph(graph);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	int h = ofGetHeight();
	int w = h / 9 * 16;
	int x = ofGetWidth() / 2 - w / 2;
	if (drawMode)
		kinect.getColorSource()->draw(x, 0, w, h);
	else
		kinect.getInfraredSource()->draw(x, 0, w, h);
	//kinect.getBodySource()->drawProjected(0, 0, w, h);
	int colorBrightness = ofMap(featExtractor.getQom(), 0, 0.05, 100, 255);
	drawProjectedWithColor(x, 0, w, h);


	for (auto beat : beats)
	{
		if (ofGetElapsedTimeMillis() - beat.getTimeStamp() <= beatLife)
		{
			beat.draw(x, 0, w, h);
		}
	}

	for (auto graph : graphs)
		graph->draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	if (key == 'f' | key == 'F')
	{
		ofToggleFullscreen();
	}
	else if (key == 'C' | key == 'c') {
		drawMode = !drawMode;
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

void ofApp::drawProjectedWithColor(int x, int y, int width, int height)
{
	ofPushMatrix();
	ofPushStyle();
	ofTranslate(x, y);
	ofScale((float)width / 1920, (float)height / 1080);

	//Draw graph colors on top of corresponding joints
	//for (auto body : kinect.getBodySource()->getBodies())
	//{
	//	if (body.tracked)
	//	{
	//		for (auto graph : graphs) {
	//			ofSetColor(ofColor::fromHsb(graph->getHue(), 255, 255, 100));
	//			if (graph->getDescriptor() != DESC_QOM && graph->getDescriptor() != DESC_CI)
	//				ofCircle(body.joints[(JointType)graph->getJoint()].getProjected(kinect.getBodySource()->getCoordinateMapper()), 30);
	//		}
	//	}
	//}


	auto bodies = kinect.getBodySource()->getBodies();
	auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

	for (auto body : bodies) {
		if (body.tracked)
		{
			for (auto bone : boneAtlas) {
				auto firstJointInBone = body.joints[bone.first];
				auto secondJointInBone = body.joints[bone.second];
				float brightness = ofMap(featExtractor.getJoint(bone.first).getDescriptor(MoDe::DESC_VELOCITY).getMagnitude(), 0, 0.04, 70, 255, true);
				ofSetColor(brightness, 150);
				//ofSetLineWidth(3 + 500 * featExtractor.getVelocityMagnitude(bone.first));
				ofSetLineWidth(3 + 500 * featExtractor.getJoint(bone.first).getDescriptor(MoDe::DESC_VELOCITY).getMagnitude());
				ofLine(firstJointInBone.getProjected(kinect.getBodySource()->getCoordinateMapper()), secondJointInBone.getProjected(kinect.getBodySource()->getCoordinateMapper()));
			}
		}
	}

	ofPopStyle();
	ofPopMatrix();
}

void ofApp::mocapExtreme(MoDe::ofxMoDeEvent & e) {
	if (e.feature == MoDe::DESC_ACCELERATION && e.axis == MoDe::AXIS_Y && e.joint == JointType_HandRight && e.extremeType == MoDe::EXTREME_TYPE_MAX) {
		ColorSpacePoint projected = { 0 };
		CameraSpacePoint position = { featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].x, featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].y, featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].z };
		kinect.getBodySource()->getCoordinateMapper()->MapCameraPointToColorSpace(position, &projected);

		Beat newBeat(ofGetElapsedTimeMillis(), ofVec2f(projected.X, projected.Y), featExtractor.getJoint(JointType_HandRight).getDescriptor(MoDe::DESC_VELOCITY).getData().end()[-2]);

		if (beats.size() <= N_BEATS) {
			beats.insert(beats.begin(), newBeat);
		}
		if (beats.size() > N_BEATS) {
			beats.pop_back();
		}

		//cout << "MAX at RH with value: " << e.value << endl;

		//for (auto &graph : graphs) {
		//	if (graph->getJoint() == JointType_HandRight)
		//		graph->newEvent();
		//}

		sound.play();
	}

	else if (e.feature == MoDe::DESC_ACCELERATION && e.axis == MoDe::AXIS_Y && e.joint == JointType_HandLeft) {
		ColorSpacePoint projected = { 0 };
		CameraSpacePoint position = { featExtractor.getJoint(JointType_HandLeft).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].x, featExtractor.getJoint(JointType_HandLeft).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].y, featExtractor.getJoint(JointType_HandLeft).getDescriptor(MoDe::DESC_POSITION).getData().end()[-2].z };
		kinect.getBodySource()->getCoordinateMapper()->MapCameraPointToColorSpace(position, &projected);

		Beat newBeat(ofGetElapsedTimeMillis(), ofVec2f(projected.X, projected.Y), featExtractor.getJoint(JointType_HandLeft).getDescriptor(MoDe::DESC_VELOCITY).getData().end()[-2]);

		if (beats.size() <= N_BEATS) {
			beats.insert(beats.begin(), newBeat);
		}
		if (beats.size() > N_BEATS) {
			beats.pop_back();
		}

		//cout << "MAX at LH with value: " << e.value << endl;

		//for (auto &graph : graphs) {
		//	if (graph->getJoint() == JointType_HandLeft)
		//		graph->newEvent();
		//}

		sound.play();
	}
}

//void ofApp::addValueToGraph(ofPtr<Graph> graph) {
//	switch (graph->getDescriptor())
//	{
//	case DESC_VELOCITY:
//		graph->addValue(featExtractor.getVelocity(graph->getJoint()));
//		break;
//	case DESC_VELOCITY_MAG:
//		graph->addValue(featExtractor.getVelocityMagnitude(graph->getJoint()));
//		break;
//	case DESC_VELOCITY_MEAN:
//		graph->addValue(featExtractor.getVelocityMagnitudeMean(graph->getJoint()));
//		break;
//	case DESC_ACCELERATION:
//		graph->addValue(featExtractor.getAcceleration(graph->getJoint()));
//		break;
//	case DESC_ACCELERATION_MAG:
//		graph->addValue(featExtractor.getAccelerationMagnitude(graph->getJoint()));
//		break;
//	case DESC_ACCELERATION_MEAN:
//		graph->addValue(featExtractor.getAccelerationMagnitudeMean(graph->getJoint()));
//		break;
//	case DESC_ACCELERATION_TRAJECTORY:
//		graph->addValue(featExtractor.getAccelerationTrajectory(graph->getJoint()));
//		break;
//	case DESC_ACCELERATION_TRAJECTORY_MEAN:
//		graph->addValue(featExtractor.getAccelerationTrajectoryMean(graph->getJoint()));
//		break;
//	case DESC_DISTANCETOTORSO:
//		graph->addValue(featExtractor.getDistanceToTorso(graph->getJoint()));
//		break;
//	case DESC_RELATIVEPOSTOTORSO:
//		graph->addValue(featExtractor.getRelativePositionToTorso(graph->getJoint()));
//		break;
//	case DESC_QOM:
//		graph->addValue(featExtractor.getQom());
//		break;
//	case DESC_CI:
//		graph->addValue(featExtractor.getCI());
//		break;
//	default:
//		break;
//	}
//}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
