#ofxKinectFeatures

ofxKinectFeatures: openFrameworks add-on for motion capture feature extraction from Kinect. Currently, it has been only tested for openFrameworks 0.8.3 in **Mac OSX**. 

##Dependencies

The current version needs **ofxOpenNI** (https://github.com/gameoverhack/ofxOpenNI) to work. Future versions will allow to input motion capture in some specific format (e.g. via OSC) from any driver.

##Features
Features can correspond to a specific joint (e.g. velocity of right hand) or general body movement (e.g. quantity of motion).
The current list of features is:
####Joint features
* Position / filtered position
* Velocity (3D)
* Velocity magnitude (speed)
* Velocity magnitude (speed) mean
* Acceleration (3D)
* Acceleration magnitude
* Acceleration magnitude mean
* Acceleration along the trajectory of movement
* Acceleration along the trajectory of movement mean
* Distance to torso
* Relative position to torso (3D) (goes from -1 to 1 along all 3 axes, calculated relative to user's height)

####Overall descriptors
* Quantity Of Motion
* Contraction Index
* Symmetry
* Maximum hand height

##Installation
Go to the addons directory and run

```
git clone https://github.com/asarasua/ofxKinectFeatures
```

Alternatively you can just click on "Downlaod ZIP" and extract the content in your addons directory.

##Getting it to work
The easiest way to go is from the example:
1. Open an openFrameworks project.
2. Copy the 'example' folder to your 'apps/myApps' (or equivalent) folder
3. Copy the folder called 'lib' from '**ofxOpenNI**/mac/copy_to_data_openni_path' to the 'bin/data/openni' directory of your example.

If you want to start a project from scratch: 
1. Follow the instructions in https://github.com/gameoverhack/ofxOpenNI to add ofxOpenNI and then:
2. Create a group under 'addons' named 'ofxKinectFeatures'
3. Drag the 'src' folder into this group.

##API
The features are accessed through an `ofxKinectFeatures` object, and for the moment we need an `ofxOpenNI` object to get kinect skeleton data. So in ofApp.h:

    ofxOpenNI kinect;
    ofxKinectFeatures featExtractor;

Then, in setup(), we need to tell the extractor where to get the data from:

    featExtractor.setKinect(&kinect);

And in update(), we just call the methods that update the kinect information and calculate descriptors:

    kinect.update();
    featExtractor.update();

From there, features can be accessed calling the appropriate methods.
For joint descriptors, using Joint constants. e.g., to get the x velocity on of the right hand:

    featExtractor.getVelocity(JOINT_RIGHT_HAND).x;

For ovearall descriptors, just by calling the method. E.g.:

    featExtractor.getQom();


