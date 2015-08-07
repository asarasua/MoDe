#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetBackgroundColor(0, 0, 0);
    ofSetWindowShape(RES_X, RES_Y);
    fontSize = 15;
    font.loadFont("Courier New.ttf", fontSize);
    
    cursor = false;
    ofHideCursor();
    
    //KINECT
    kinect.setup();
    kinect.addDepthGenerator();
    kinect.addImageGenerator();
    kinect.setRegister(false);
    kinect.setMirror(true);
    kinect.addUserGenerator();
    kinect.setMaxNumUsers(3);
    kinect.setSkeletonProfile(XN_SKEL_PROFILE_ALL);
    kinect.start();
    hadUsers = false;
    
    //normal distribution
    normdist = new tr1::normal_distribution<float>();
    rndnorm  = new tr1::variate_generator<tr1::mt19937, tr1::normal_distribution<float> >(rng, *normdist);
    
    //FEAT EXTRACTOR
    featExtractor.setup(JOINT_HEAD, JOINT_TORSO);
    featExtractor.setFilterLevel(filter::HARD);
    ofAddListener(kinect.userEvent, this, &ofApp::userEvent);
    ofAddListener(MocapMaxEvent::events, this, &ofApp::mocapMax);
    ofAddListener(MocapMinEvent::events, this, &ofApp::mocapMin);
    
    //HAND TRAJ PARTICLES
    for( int i = 0; i < NUM_PARTICLES_HAND; i++ ){
        leftHand[i].size   = HANDS_MAX_SIZE;
        leftHand[i].damp = 1;
        rightHand[i].size   = HANDS_MAX_SIZE;
        rightHand[i].damp = 1;
    }
    hue = 245;
    sat = 178;
    
    //SKELETON SHADE (FBO)
    #ifdef TARGET_OPENGLES
    fbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA ); // with alpha, 32 bits red, 32 bits green, 32 bits blue, 32 bits alpha, from 0 to 1 in 'infinite' steps
        ofLogWarning("ofApp") << "GL_RGBA32F_ARB is not available for OPENGLES.  Using RGBA.";
    #else
        fbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA32F_ARB); // with alpha, 32 bits red, 32 bits green, 32 bits blue, 32 bits alpha, from 0 to 1 in 'infinite' steps
    #endif
    
    fbo.begin();
    ofClear(255,255,255, 0);
    fbo.end();
    fadeAmnt = 5;
    alphaControl = 0.5;
    particleIndex = 0;
    particleSize = 2;
    lastRightBeat = 0.0;
    lastLeftBeat = 0.0;
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    
    //This is a trick to reset the user generator if all users are lost
    //update hand trajectories
    if (kinect.getNumTrackedUsers()) {
        //mark the flag: there are users
        hadUsers = true;
    } else if (!kinect.getNumTrackedUsers() && hadUsers){
        fbo.begin();
        ofClear(255,255,255, 0);
        fbo.end();
        hadUsers = false;
        kinect.waitForThread(true);
        kinect.setPaused(true);
        kinect.removeUserGenerator();
        kinect.addUserGenerator();
        kinect.setPaused(false);
        kinect.startThread();
    }
    
    //featExtractor
    for (int i = 0; i < kinect.getNumTrackedUsers(); i++) {
        ofxOpenNIUser user = kinect.getTrackedUser(i);
        //The following "if" statement is a hard-coded alternative for if(kinect.getUserGenerator().IsNewDataAvailable()), which doesn't work properly in ofxOpenNI
        if (user.getJoint((Joint)0).getWorldPosition() != ofPoint(0,0,0) &&
            user.getJoint((Joint)0).getWorldPosition() != featExtractor.getPosition(0) ) {
            map<int, ofPoint> joints;
            for (int j = 0; j < user.getNumJoints(); j++) {
                joints[j] = user.getJoint((Joint)j).getWorldPosition();
            }
            featExtractor.update(joints);
        }
    }

    //Hand trajectories particles
    if (kinect.getNumTrackedUsers()) {
        //mark the flag: there are users
        ofPoint lh = kinect.getTrackedUser(0).getJoint(JOINT_LEFT_HAND).getProjectivePosition();
        leftHand[0].moveTo(ofVec3f(lh.x, lh.y, featExtractor.getRelativePositionToTorso(JOINT_LEFT_HAND).z));
        leftHand[0].color.set(255, 0, ofMap(leftHand[0].pos.z, -1, 1, 0, 50, true));
        ///leftHand[0].size =
        for (int i = NUM_PARTICLES_HAND-1; i > 0; i-- ){
            leftHand[i].moveTo(leftHand[i-1].pos);
            leftHand[i].color.setHsb(hue, sat, ofMap(leftHand[i].pos.z, -1, 1, 0, 200, true));
            leftHand[i].alphaF = ofMap(NUM_PARTICLES_HAND - i, 0, NUM_PARTICLES_HAND, 0.2, 1);
            //leftHand[i].color.setHsb(ofMap(rightHand[i].pos.z, -1, 1, 55, 255, true), 200, ofMap(NUM_PARTICLES_HAND - i, 0, NUM_PARTICLES_HAND, 50, 255));
        }
        
        ofPoint rh = kinect.getTrackedUser(0).getJoint(JOINT_RIGHT_HAND).getProjectivePosition();
        rightHand[0].moveTo(ofVec3f(rh.x, rh.y, featExtractor.getRelativePositionToTorso(JOINT_RIGHT_HAND).z));
        rightHand[0].color.set(255, 0, ofMap(rightHand[0].pos.z, -1, 1, 0, 40, true));
        for (int i = NUM_PARTICLES_HAND-1; i > 0; i-- ){
            rightHand[i].moveTo(rightHand[i-1].pos);
            rightHand[i].color.setHsb(hue, sat, ofMap(rightHand[i].pos.z, -1, 1, 0, 200, true));
            rightHand[i].alphaF = ofMap(NUM_PARTICLES_HAND - i, 0, NUM_PARTICLES_HAND, 0.2, 1);
            //rightHand[i].color.setHsb(ofMap(rightHand[i].pos.z, -1, 1, 0, 200, true), 200, ofMap(NUM_PARTICLES_HAND - i, 0, NUM_PARTICLES_HAND, 50, 255));
        }
    }
    
    //Update BEAT particles
    for (int i = 0; i < NUM_PARTICLES_BEAT_TOTAL; i++) {
        beatParticles[i].update(0.0);
        beatParticles[i].addNoise(10, 0.01);
        beatParticles[i].alphaF = ofMap(beatParticles[i].vel.length(), 0, 3, 0, 1);
        //beatParticles[i].addAlphaFade(false);
    }
    
    //FBO
    fbo.begin();
    drawFbo();
    fbo.end();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofBackground(0, 0, 0);
    ofSetColor(ofColor::white);
    float w = ofGetWidth(); //w
    float h = ofGetHeight(); //h
        
    //INFO
    ostringstream os;
    os << "conductor analysis" << endl;
    os << "FPS: " << ofGetFrameRate() << endl;
    if (kinect.getNumTrackedUsers()) {
        os << "Quantity of Motion: " << featExtractor.getQom() << endl;
        os << "Contraction Index: " << featExtractor.getCI() << endl << endl;
        os << "RIGHT HAND";
        if (ofGetElapsedTimef() - lastRightBeat < 0.45) {
            os << "   [DETECTED BEAT]";
        }
        
        
        os << endl;
        os << "velocity" << endl;
        os << "  x: " << featExtractor.getVelocity(JOINT_RIGHT_HAND).x << endl;
        os << "  y: " << featExtractor.getVelocity(JOINT_RIGHT_HAND).x << endl;
        os << "  z: " << featExtractor.getVelocity(JOINT_RIGHT_HAND).z << endl;
        os << "  magnitude: " << featExtractor.getVelocityMagnitude(JOINT_RIGHT_HAND) << endl;
        os << "acceleration" << endl;
        os << "  x: " << featExtractor.getAcceleration(JOINT_RIGHT_HAND).x << endl;
        os << "  y: " << featExtractor.getAcceleration(JOINT_RIGHT_HAND).x << endl;
        os << "  z: " << featExtractor.getAcceleration(JOINT_RIGHT_HAND).z << endl;
        os << "  magnitude: " << featExtractor.getAccelerationMagnitude(JOINT_RIGHT_HAND) << endl << endl;
        
        os << "LEFT HAND";
        if (ofGetElapsedTimef() - lastLeftBeat < 0.45) {
            os << "   [DETECTED BEAT]";
        }
        os << endl;
        os << "velocity" << endl;
        os << "  x: " << featExtractor.getVelocity(JOINT_LEFT_HAND).x << endl;
        os << "  y: " << featExtractor.getVelocity(JOINT_LEFT_HAND).x << endl;
        os << "  z: " << featExtractor.getVelocity(JOINT_LEFT_HAND).z << endl;
        os << "  magnitude: " << featExtractor.getVelocityMagnitude(JOINT_LEFT_HAND) << endl;
        os << "acceleration" << endl;
        os << "  x: " << featExtractor.getAcceleration(JOINT_LEFT_HAND).x << endl;
        os << "  y: " << featExtractor.getAcceleration(JOINT_LEFT_HAND).x << endl;
        os << "  z: " << featExtractor.getAcceleration(JOINT_LEFT_HAND).z << endl;
        os << "  magnitude: " << featExtractor.getAccelerationMagnitude(JOINT_LEFT_HAND) << endl << endl;
        
        os << "right arm angle: " << featExtractor.getAngle(JOINT_RIGHT_HAND, JOINT_RIGHT_ELBOW, JOINT_RIGHT_SHOULDER) << endl;
        os << "left arm angle: " << featExtractor.getAngle(JOINT_LEFT_HAND, JOINT_LEFT_ELBOW, JOINT_LEFT_SHOULDER) << endl;
    }
    
    ofSetColor(0,0,0,100);
    ofRect(2 * PADDING + kinect.getWidth(), 10, 500, 150);
    ofSetColor(255,255,255, 200);
    ofPushStyle();
    //ofDrawBitmapString(os.str(), 0, 0);
    font.drawString(os.str(), 50, 20);
    ofPopStyle();
    
    
    //FBO (skeleton)
    float kinectWidth = (4.0/3.0)*ofGetWindowHeight();
    ofSetColor(243,72,109);
    fbo.draw(0,0);
    //fbo.draw(0.5 * (ofGetWindowWidth() - kinectWidth), 0);//, kinectWidth, ofGetWindowHeight());
    ofSetColor(ofColor::white);
    drawNiceSkeleton(0.5 * (ofGetWindowWidth() - kinectWidth), 0, kinectWidth, ofGetWindowHeight());
    //drawNiceSkeleton(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
    
    //BEATS
    ofEnableAlphaBlending();
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofPushMatrix();
    ofTranslate(0.5 * (ofGetWindowWidth() - kinectWidth), 0);
    ofScale(kinectWidth/kinect.getWidth(), ofGetWindowHeight()/kinect.getHeight(), 1.0f);
    for (int i = 0; i < NUM_PARTICLES_BEAT_TOTAL; i++) {
        if (beatParticles[i].isAlive()) {
            ofSetColor(beatParticles[i].color, beatParticles[i].color.a * beatParticles[i].alphaF);
            ofRect(beatParticles[i].pos.x, beatParticles[i].pos.y, particleSize, particleSize);
        }
    }
    ofPopMatrix();
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
    //HAND TRAJECTORIES
    if (kinect.getNumTrackedUsers()){
        ofPushStyle();
        ofPushMatrix();
        ofTranslate(0.5 * (ofGetWindowWidth() - kinectWidth), 0);
        ofScale(kinectWidth/kinect.getWidth(), ofGetWindowHeight()/kinect.getHeight(), 1.0f);
        
        // Go through particles and paint
        for (int i = 0; i < NUM_PARTICLES_HAND-1; i++){
            ofSetLineWidth(10);
            ofSetColor(leftHand[i].color, 255 * leftHand[i].alphaF);
            //ofSetColor(250, 250, 250, leftHand[i].alphaF);
            ofLine(leftHand[i].pos.x, leftHand[i].pos.y, leftHand[i+1].pos.x, leftHand[i+1].pos.y);
            ofSetColor(rightHand[i].color, 255 * rightHand[i].alphaF);
            ofLine(rightHand[i].pos.x, rightHand[i].pos.y, rightHand[i+1].pos.x, rightHand[i+1].pos.y);
        }
        
        ofPopMatrix();
        ofPopStyle();
    }
}

//--------------------------------------------------------------
void ofApp::drawNiceSkeleton(float x, float y, float w, float h){
    for(int i = 0;  i < kinect.getNumTrackedUsers(); ++i){
        ofPushStyle();
        ofPushMatrix();
        ofTranslate(x, y);
        ofScale(w/kinect.getWidth(), h/kinect.getHeight(), 1.0f);
        
        ofPushStyle();
        for(int i = 0; i < JOINT_COUNT; i++){
            
            ofPushStyle();
            ofPushMatrix();
            
            //if(getPositionConfidence() > 0.0f){
            ofPoint jointPos = featExtractor.getPositionFiltered(i);
            ofPoint projectivePosition = kinect.worldToProjective(jointPos);
            ofTranslate(projectivePosition.x, projectivePosition.y);//, projectivePosition.z);
            // draw in green
            ofSetColor(250, 250, 250);
                
            ofFill();
            if (i == JOINT_HEAD) {
                ofCircle(0, 0, 15);
            }
            ofNoFill();
            
            ofSetLineWidth(5);
            if(kinect.getTrackedUser(0).getJoint((Joint)i).isParent() && i != JOINT_HEAD && i != JOINT_RIGHT_HIP && i != JOINT_LEFT_HIP && i!= JOINT_TORSO && i!=JOINT_NECK && i!=JOINT_LEFT_FOOT && i!=JOINT_LEFT_KNEE && i!=JOINT_RIGHT_FOOT && i!=JOINT_RIGHT_KNEE){
                int parentId = kinect.getTrackedUser(0).getJoint((Joint)i).getParent().getType();
                ofPoint parentPos = featExtractor.getPositionFiltered(parentId);
                if (i == JOINT_RIGHT_SHOULDER || i == JOINT_LEFT_SHOULDER) {
                    parentPos.y = parentPos.y - 10;
                }
                
                parentPos = kinect.worldToProjective(parentPos);
                
                ofVec2f bone =  ofVec2f(parentPos - ofVec2f(kinect.worldToProjective(jointPos)));
                // make "smaller" bones
                if (i == JOINT_RIGHT_HAND || i == JOINT_LEFT_HAND) {
                    ofLine(0.1*bone.x, 0.1*bone.y, 0, 0.9*bone.x, 0.9*bone.y, 0);
                } else if (i == JOINT_RIGHT_SHOULDER || i == JOINT_LEFT_SHOULDER) {
                    ofLine(0.1*bone.x, 0.1*bone.y, 0, bone.x, bone.y, 0);
                } else {
                    ofLine(0, 0, 0, bone.x, bone.y, 0);
                }
            }
            //}
            
            ofPopMatrix();
            ofPopStyle();
            
        }
        ofPopStyle();
        
        ofPopMatrix();
        ofPopStyle();
    }
}

//--------------------------------------------------------------
void ofApp::drawFbo(){
    //1 - Fade Fbo
    fadeAmnt = ofMap(featExtractor.getQom(), 0, 40, (1-alphaControl)*40, 0);
    ofFill();
    ofSetColor(255,255,255, fadeAmnt);
    ofRect(0,0,ofGetWindowWidth(),ofGetWindowHeight());
    
    //2 - Draw graphics
    float kinectWidth = (4.0/3.0)*ofGetWindowHeight();
    drawNiceSkeleton(0.5 * (ofGetWindowWidth() - kinectWidth), 0, kinectWidth, ofGetWindowHeight());
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        //SHADE ALPHA CONTROL
        case 'q':
        case 'Q':
            if (alphaControl < 1.0) {
                alphaControl += 0.1;
            }
            break;
        case 'a':
        case 'A':
            if (alphaControl > 0.0) {
                alphaControl -= 0.1;
            }
            break;
        
        //PARTICLE SIZE
        case 'w':
        case 'W':
            particleSize += 1;
            break;
        case 's':
        case 'S':
            if (particleSize > 1) {
                particleSize -= 1;
            }
            break;
            
        case 'e':
        case 'E':
            fontSize += 1;
            font.loadFont("Courier New.ttf", fontSize);
            break;
        case 'd':
        case 'D':
            if (fontSize > 1) {
                fontSize -= 1;
            }
            font.loadFont("Courier New.ttf", fontSize);
            break;
            
        case 'P':
        case 'p':
            ofToggleFullscreen();
#ifdef TARGET_OPENGLES
            fbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA ); // with alpha, 32 bits red, 32 bits green, 32 bits blue, 32 bits alpha, from 0 to 1 in 'infinite' steps
            ofLogWarning("ofApp") << "GL_RGBA32F_ARB is not available for OPENGLES.  Using RGBA.";
#else
            fbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA32F_ARB); // with alpha, 32 bits red, 32 bits green, 32 bits blue, 32 bits alpha, from 0 to 1 in 'infinite' steps
#endif
            break;
            
        case 'M':
        case 'm':
            cursor = !cursor;
            if (cursor) {
                ofShowCursor();
            } else {
                ofHideCursor();
            }
            break;
            
        case OF_KEY_UP:
            
            break;
        case OF_KEY_DOWN:
            
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::userEvent(ofxOpenNIUserEvent &event){
//    if (event.userStatus == USER_TRACKING_STOPPED) {
//        featExtractor.removeSkeleton(0);
//    }
}

void ofApp::mocapMax(MocapMaxEvent &e){
//    if (e.joint == JOINT_RIGHT_HAND && e.value > 5.0) {
//        cout << "Max in right hand axis " << ofGetTimestampString() << endl;
//    }
    
//    if (e.joint == JOINT_RIGHT_HAND && e.feature == FEAT_ACCELERATION && e.axis == MOCAP_Y && e.value > 2.0){
//        cout << "max in ACC Y!" << endl;
//    }
}

void ofApp::mocapMin(MocapMinEvent &e){
    if (e.joint == JOINT_RIGHT_HAND && e.feature == FEAT_ACCELERATION && e.axis == MOCAP_Y && e.value < -2.0){
        //cout << "min in ACC Y!" << endl;
        lastRightBeat = ofGetElapsedTimef();
        
        for (int i = particleIndex; i < particleIndex + NUM_PARTICLES_BEAT_INSTANCE; i++) {
            beatParticles[i].alphaF = 1;
            beatParticles[i].life = 200;
            beatParticles[i].initialLife = 200;
            beatParticles[i].size = 2;
            beatParticles[i].color.set(250, 250, 250, 255);
            beatParticles[i].damp = 0.98;
            beatParticles[i].pos = kinect.worldToProjective(featExtractor.getPositionFiltered(JOINT_RIGHT_HAND));
            beatParticles[i].vel.set(0.0, 0.0);
            ofVec2f beatVelocity;
            //float angle = ofRandom(-45, 45);
            float angle;
            if (particleIndex + NUM_PARTICLES_BEAT_INSTANCE - i < NUM_PARTICLES_BEAT_INSTANCE / 2) {
                angle = 5 * (*rndnorm)();
            } else {
                angle = 30 * (*rndnorm)();
            }
            

            beatVelocity.x =  - featExtractor.getVelocityHistory(JOINT_RIGHT_HAND)[3][0] / 10.0;
            beatVelocity.y = featExtractor.getVelocityHistory(JOINT_RIGHT_HAND)[3][1] / 10.0;
            beatVelocity.rotate(angle);
            //beatVelocity *= ofNormalize(abs(abs(angle)-5), 0, 5);
            beatVelocity *= ofRandom(1.0);
            if (particleIndex + NUM_PARTICLES_BEAT_INSTANCE - i >= NUM_PARTICLES_BEAT_INSTANCE / 2) {
                beatVelocity *= 0.2;
            }
            beatParticles[i].addForce(beatVelocity);
        }
        
        particleIndex += NUM_PARTICLES_BEAT_INSTANCE;
        if (particleIndex == NUM_PARTICLES_BEAT_TOTAL) {
            particleIndex = 0;
        }
    }
    
    else if (e.joint == JOINT_LEFT_HAND && e.feature == FEAT_ACCELERATION && e.axis == MOCAP_Y && e.value < -2.0){
        //cout << "min in ACC Y!" << endl;
        lastLeftBeat = ofGetElapsedTimef();
        
        for (int i = particleIndex; i < particleIndex + NUM_PARTICLES_BEAT_INSTANCE; i++) {
            beatParticles[i].alphaF = 1;
            beatParticles[i].life = 200;
            beatParticles[i].initialLife = 200;
            beatParticles[i].size = 2;
            beatParticles[i].color.set(250, 250, 250, 255);
            beatParticles[i].damp = 0.98;
            beatParticles[i].pos = kinect.worldToProjective(featExtractor.getPositionFiltered(JOINT_LEFT_HAND));
            beatParticles[i].vel.set(0.0, 0.0);
            ofVec2f beatVelocity;
            //float angle = ofRandom(-45, 45);
            float angle;
            if (particleIndex + NUM_PARTICLES_BEAT_INSTANCE - i < NUM_PARTICLES_BEAT_INSTANCE / 2) {
                angle = 5 * (*rndnorm)();
            } else {
                angle = 30 * (*rndnorm)();
            }
            
            
            beatVelocity.x =  - featExtractor.getVelocityHistory(JOINT_LEFT_HAND)[3][0] / 10.0;
            beatVelocity.y = featExtractor.getVelocityHistory(JOINT_LEFT_HAND)[3][1] / 10.0;
            beatVelocity.rotate(angle);
            //beatVelocity *= ofNormalize(abs(abs(angle)-5), 0, 5);
            beatVelocity *= ofRandom(1.0);
            if (particleIndex + NUM_PARTICLES_BEAT_INSTANCE - i >= NUM_PARTICLES_BEAT_INSTANCE / 2) {
                beatVelocity *= 0.2;
            }
            beatParticles[i].addForce(beatVelocity);
        }
        
        particleIndex += NUM_PARTICLES_BEAT_INSTANCE;
        if (particleIndex == NUM_PARTICLES_BEAT_TOTAL) {
            particleIndex = 0;
        }
    }
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