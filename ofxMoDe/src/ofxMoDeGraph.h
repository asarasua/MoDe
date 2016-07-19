#ifndef conductvizz_graph
#define conductvizz_graph

#define N_POINTS 120
#define N_EVENTS 20
#define MODE_1D 0
#define MODE_3D 1

#include "ofMain.h"

class ofxMoDeGraph {
public:
    ofxMoDeGraph();
    ofxMoDeGraph(float x, float y, float w, float h, float colorHue, string graphText);
    void setText(string graphText);
    void setPos(float x, float y);
    void setSize(float w, float h);
    void setHue(float colorHue);
    float getHue();
    void addValue(float value);
    void addValue(ofVec3f threeTimeSeries);
    void newEvent();
    void draw();
private:
    void mousePressed(ofMouseEventArgs& event);
    void mouseDragged(ofMouseEventArgs& event);
    void mouseReleased(ofMouseEventArgs& event);
    ofVec2f screenPos, size, prevMouseMove, prevMouseResize, prevMouseScale;
    vector<float> oneTimeSeries;
    vector<ofVec3f> threeTimeSeries;
    vector<int> events;
    int mode;
    string text;
    float hue, scale;
};

#endif
