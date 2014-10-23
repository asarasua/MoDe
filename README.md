#ofxKinectFeatures

ofxKinectFeatures: openFrameworks add-on for motion capture feature extraction from Kinect. Currently, it has been only tested for openFrameworks 0.8.3 in **Mac OSX**. 

##Features
Features can correspond to a specific joint (e.g. velocity of right hand) or general body movement (e.g. quantity of motion). They can be extracted from more than one skeleton.
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

###New project

- Copy an empty example to your 'apps/myApps' (or equivalent) directory.
- Create a new group called 'ofxKinectFeatures' under the 'addons' group.
- Drag the 'src' folder from the 'addons/ofxKinectFeatures/' directory to the group you just created.

###From the example
The current version needs **ofxOpenNI** (https://github.com/gameoverhack/ofxOpenNI) to work. 

To make this example work, do the following:
- Download ofxOpenNI to your addons directory.
- Open an openFrameworks project.
- Copy the 'example' folder to your 'apps/myApps' (or equivalent) directory.
- Copy the 'bin' folder from 'addons/ofxOpenNI/examples/opeNI-SimpleExamples/' to your project directory (this is to have the correct folder structure and config files for openNI to work).
- Copy the 'lib' folder from '**ofxOpenNI**/mac/copy_to_data_openni_path' to the 'bin/data/openni' directory of your project.

##API
The API has been designed not to depend on a particular library for the skeleton tracking with Kinect. Currently, an example using **ofxOpenNI** is included.

The features are accessed through an `ofxKinectFeatures` object, so declare it in ofApp.h:

    ofxKinectFeatures featExtractor;

In update(), we just update skeletons data sending a `map<int, ofPoint>` where keys are integers identifying joints and values are the x, y, z positions of these joints. For example, when using ofxOpenNI, this can be done as follows:

    //kinect is an ofxOpenNI object
    for (int i = 0; i < kinect.getNumTrackedUsers(); i++) {
        ofxOpenNIUser user = kinect.getTrackedUser(i);
        map<int, ofPoint> joints;
        for (int j = 0; j < user.getNumJoints(); j++) {
            joints[j] = user.getJoint((Joint)j).getWorldPosition();
        }
        featExtractor.updateSkeleton(i, joints);
    }

From there, features of a particular skeleton can be accessed calling the appropriate methods. It is **necesarry** to check if the desired skeleton exists.
For joint descriptors, using skeleton and joint id's. e.g., to get the x velocity of the right hand (in ofxOpenNI, identified by the `JOINT_RIGHT_HAND`constant):

    if (featExtractor.skeletonExists(0)) {
        featExtractor.getSkeleton(0)->getVelocity(JOINT_RIGHT_HAND).x;
    }

For ovearall descriptors, just by calling the method. E.g.:

    featExtractor.getSkeleton(0)->getQom();

If a skeleton is lost, it has to be removed from the ofxKinectFeatures object:

    featExtractor.removeSkeleton(0);

In the ofxOpenNI example, this is done in a method that is called everytime an ofxOpenNIUserEvent occurs:

    void ofApp::userEvent(ofxOpenNIUserEvent &event){
        if (event.userStatus == USER_TRACKING_STOPPED) {
            featExtractor.removeSkeleton(0);
        }
    }

