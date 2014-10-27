#ofxKinectFeatures

ofxKinectFeatures: openFrameworks add-on for motion capture feature extraction from Kinect. It has been tested for openFrameworks 0.8.x in Mac OSX 10.9 and Windows 8.1. Working examples in both platforms are provided for XCode and Visual Studio 2012, respectively. 

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

##Installation
Go to the addons directory and run

```
git clone https://github.com/asarasua/ofxKinectFeatures
```

Alternatively you can just click on "Downlaod ZIP" and extract the content in your addons directory.

##Getting it to work
ofxKinectFeatures only computes descriptors from motion capture data. To track the skeleton from a Kinect device, other addons have to be included in the project. Examples using [**ofxOpenNI**](https://github.com/gameoverhack/ofxOpenNI) and [**ofxKinectNui**](https://github.com/sadmb/ofxKinectNui) are provided for OSX (+XCode) and Windows (+VS 2012).

###New project

- Copy an empty example to your 'apps/myApps' (or equivalent) directory.
- Include an addon for Kinect skeleton tracking following the indications provided in their repositories.
- Create a new group called 'ofxKinectFeatures' under the 'addons' group.
- Drag the 'src' folder from the 'addons/ofxKinectFeatures/' directory to the group you just created.

###From the examples
To make the **ofxOpenNI** example work in **OSX**, do the following:
- Download ofxOpenNI to your addons directory.
- Open an openFrameworks project.
- Copy the 'example' folder to your 'apps/myApps' (or equivalent) directory.
- Copy the 'bin' folder from 'addons/ofxOpenNI/examples/opeNI-SimpleExamples/' to your project directory (this is to have the correct folder structure and config files for openNI to work).
- Copy the 'lib' folder from '**ofxOpenNI**/mac/copy_to_data_openni_path' to the 'bin/data/openni' directory of your project.

For the **ofxKinectNui** example (**Windows**), you just need to have the latest Kinect 1.x SDK installed and ofxKinectNui in your addons directory.

##API
The API has been designed not to depend on a particular library for the skeleton tracking with Kinect. Examples using **ofxOpenNI** and **ofxKinectNui** are included.

The features are accessed through an `ofxKinectFeatures` object, so declare it in ofApp.h:

```cpp
ofxKinectFeatures featExtractor;
```

If you wish to extract features for more than one skeleton, just declare one ofxKinectFeatures object for each (e.g. by creating a `map<int,ofxKinectFeatures>` where keys correspond to an skeleton id).

In `setup()`, it is necessary to tell ofxKinectFeatures which indices correspond to the **head** and **torso** joints, as some calculations need to know about this. This is easily done depending on the addon or libray being used for skeleton tracking.

```cpp
featExtractor.setup(JOINT_HEAD, JOINT_TORSO); //ofxOpenNI
featExtractor.setup(NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SPINE); //ofxKinectNui
```

In `update()`, just update skeletons data sending a `map<int, ofPoint>` where keys are integers identifying joints and values are the x, y, z positions of these joints. For example, when using ofxOpenNI, this can be done as follows:

```cpp
//kinect is an ofxOpenNI object
if (kinect.getNumTrackedUsers()) {
    ofxOpenNIUser user = kinect.getTrackedUser(0);
    map<int, ofPoint> joints;
    for (int j = 0; j < user.getNumJoints(); j++) {
        joints[j] = user.getJoint((Joint)j).getWorldPosition();
    }
    featExtractor.update(joints);
}
```

From there, features can be accessed calling the appropriate methods.

For joint descriptors, joint id's. E.g., to get the x velocity of the right hand (in ofxOpenNI, identified by the `JOINT_RIGHT_HAND`constant):

    if (featExtractor.skeletonExists(0)) {
        featExtractor.getVelocity(JOINT_RIGHT_HAND).x;
    }

For ovearall descriptors, just by calling the method. E.g.:

    featExtractor.getQom();


