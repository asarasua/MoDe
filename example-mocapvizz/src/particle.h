//
//  ball.h
//  expworm
//
//  Created by Álvaro Sarasúa Berodia on 27/02/13.
//  Based on Patricio González Calvo's code
//

#ifndef __expworm__ball__
#define __expworm__ball__

#include "ofMain.h"

class Particle {
public:
    // Constructor
    Particle();
    
    //Dynamics
    void moveTo(ofVec3f _position);
    void addForce(ofVec2f _force);
    void addNoise(float _angle, float _turbulence);   //adds a random subtle movement
    
    //Fade outs
    void addAlphaFade(bool _fadeOut);
    void addScaleFade(bool _melt);
    
    //State checkers
    bool crossedBorder();
    bool isAlive();
    
    //update
    void update(float _speedLimit);
    void draw();
    
    ofColor color, strokeColor;
    ofVec2f vel, acc;
    ofVec3f pos;
    
    float alphaF, scaleF, size;
    
    float damp;
    
    int life, initialLife;
};

#endif /* defined(__expworm__ball__) */