#include "ofxKinectFeaturesGraph.h"

ofxKinectFeaturesGraph::ofxKinectFeaturesGraph()
{
    ofAddListener(ofEvents().mousePressed, this, &ofxKinectFeaturesGraph::mousePressed);
    ofAddListener(ofEvents().mouseDragged, this, &ofxKinectFeaturesGraph::mouseDragged);
    ofAddListener(ofEvents().mouseReleased, this, &ofxKinectFeaturesGraph::mouseReleased);
    
    hue = 0.0;
    scale = 1.0;
    text = "graph";
    
    screenPos = ofVec2f(0, 0);
    size = ofVec2f(400, 300);
    
    mode = MODE_1D;
    for (size_t i = 0; i < N_POINTS; i++)
    {
        oneTimeSeries.push_back(0.0);
        threeTimeSeries.push_back(ofVec3f(0.0, 0.0, 0.0));
    }
    prevMouseMove = ofVec2f(0.0);
    prevMouseResize = ofVec2f(0.0);
}

ofxKinectFeaturesGraph::ofxKinectFeaturesGraph(float x, float y, float w, float h, float colorHue)
{
    screenPos = ofVec2f(x, y);
    size = ofVec2f(w, h);
    hue = colorHue;
    
    ofAddListener(ofEvents().mousePressed, this, &ofxKinectFeaturesGraph::mousePressed);
    ofAddListener(ofEvents().mouseDragged, this, &ofxKinectFeaturesGraph::mouseDragged);
    ofAddListener(ofEvents().mouseReleased, this, &ofxKinectFeaturesGraph::mouseReleased);
    
    scale = 1.0;
    text = "graph";
    
    mode = MODE_1D;
    for (size_t i = 0; i < N_POINTS; i++)
    {
        oneTimeSeries.push_back(0.0);
        threeTimeSeries.push_back(ofVec3f(0.0, 0.0, 0.0));
    }
    prevMouseMove = ofVec2f(0.0);
    prevMouseResize = ofVec2f(0.0);
}

void ofxKinectFeaturesGraph::setPos(float x, float y)
{
    screenPos = ofVec2f(x, y);
}

void ofxKinectFeaturesGraph::setSize(float w, float h)
{
    size = ofVec2f(w, h);
}

void ofxKinectFeaturesGraph::setHue(float colorHue)
{
    hue = colorHue;
}

float ofxKinectFeaturesGraph::getHue()
{
    return hue;
}

void ofxKinectFeaturesGraph::addValue(float value)
{
    mode = MODE_1D;
    if (oneTimeSeries.size() <= N_POINTS) {
        oneTimeSeries.insert(oneTimeSeries.begin(), value);
    }
    if (oneTimeSeries.size() > N_POINTS) {
        oneTimeSeries.pop_back();
    }
    
    for (auto &event : events)
        event += 1;
}

void ofxKinectFeaturesGraph::addValue(ofVec3f value)
{
    mode = MODE_3D;
    if (threeTimeSeries.size() <= N_POINTS) {
        threeTimeSeries.insert(threeTimeSeries.begin(), value);
    }
    if (threeTimeSeries.size() > N_POINTS) {
        threeTimeSeries.pop_back();
    }
    
    for (auto &event : events)
        event += 1;
}

void ofxKinectFeaturesGraph::newEvent()
{
    if (events.size() <= N_EVENTS) {
        events.insert(events.begin(), 0);
    }
    if (events.size() > N_EVENTS) {
        events.pop_back();
    }
}

void ofxKinectFeaturesGraph::draw()
{
    ofPushMatrix();
    ofPushStyle();
    ofTranslate(screenPos);
    ofScale(size.x, size.y);
    
    ofSetColor(200, 200);
    ofDrawRectangle(0, 0, 1, 1);
    //Change size square
    ofSetColor(0, 200);
    ofDrawRectangle(0.97, 1 - 0.03 * size.x / size.y, 0.03, 0.03 * size.x / size.y);
    //Change descriptor square
    ofSetColor(122, 20, 32, 200);
    ofDrawRectangle(0, 1 - 0.03 * size.x / size.y, 0.03, 0.03 * size.x / size.y);
    //Change joint square
    ofSetColor(204, 102, 114, 200);
    ofDrawRectangle(0.03, 1 - 0.03 * size.x / size.y, 0.03, 0.03 * size.x / size.y);
    
    //descriptor name
    ofDrawBitmapString(text, 0.07, 1 - 0.03 * size.x / size.y);
    
    if (mode == MODE_1D)
    {
        ofPolyline line;
        for (int i = 0; i < oneTimeSeries.size(); i++) {
            float x = 0.95 - (float)i / N_POINTS * 0.9;
            float y = 0.5 - scale * oneTimeSeries[i];
            line.addVertex(x, y);
        }
        ofSetColor(ofColor::fromHsb(hue, 255, 255));
        line.draw();
    }
    else if (mode == MODE_3D) {
        ofPolyline line1, line2, line3;
        for (int i = 0; i < threeTimeSeries.size(); i++) {
            float x = 0.95 - (float)i / N_POINTS * 0.9;
            ofVec3f y = 0.5 - scale * threeTimeSeries[i];
            line1.addVertex(x, y.x);
            line2.addVertex(x, y.y);
            line3.addVertex(x, y.z);
        }
        ofSetColor(ofColor::fromHsb(hue, 255, 50));
        line1.draw();
        ofSetColor(ofColor::fromHsb(hue, 255, 150));
        line2.draw();
        ofSetColor(ofColor::fromHsb(hue, 255, 255));
        line3.draw();
    }
    
    ofSetColor(ofColor::fromHsb(hue, 255, 10));
    for (auto event : events) {
        if (event < N_POINTS)
            ofDrawLine(0.95 - (float)event / N_POINTS * 0.9, 0, 0.95 - (float)event / N_POINTS * 0.9, 1);
    }
    
    ofPopStyle();
    ofPopMatrix();
    
}

void ofxKinectFeaturesGraph::mousePressed(ofMouseEventArgs& event) {
    //bottom-right corner (change size)
    if (event.x > screenPos.x + 0.97 * size.x && event.x < screenPos.x + size.x &&
        event.y > screenPos.y + size.y - 0.03 * size.x && event.y < screenPos.y + size.y)
        prevMouseResize = event;
    
    //bottom-left corner (change descriptor)
    else if (scale >= 0.0 &&
             event.x > screenPos.x && event.x < screenPos.x + 0.03 * size.x &&
             event.y > screenPos.y + size.y - 0.03 * size.x && event.y < screenPos.y + size.y)
        scale -= 0.05;
    
    //bottom-left corner 2 (change joint)
    else if (event.x > screenPos.x + 0.03 * size.x && event.x < screenPos.x + 0.06 * size.x &&
             event.y > screenPos.y + size.y - 0.03 * size.x && event.y < screenPos.y + size.y)
        scale += 0.05;
    
    //everywhere else (move)
    else if (event.x > screenPos.x && event.x < screenPos.x + size.x &&
             event.y > screenPos.y && event.y < screenPos.y + size.y)
        prevMouseMove = event;
}

void ofxKinectFeaturesGraph::mouseReleased(ofMouseEventArgs& event) {
    prevMouseMove = ofVec2f(0.0);
    prevMouseResize = ofVec2f(0.0);
}


void ofxKinectFeaturesGraph::mouseDragged(ofMouseEventArgs & event)
{
    if (prevMouseMove != ofVec2f(0.0)) {
        screenPos += event - prevMouseMove;
        prevMouseMove = event;
    } else if (prevMouseResize != ofVec2f(0.0)) {
        size += event - prevMouseResize;
        prevMouseResize = event;
    }
}
