# MoDe

MoDe is a cross-platform C++ library for real-time feature extraction from Motion Capture (MoCap) data, particularly suited for Kinect devices. It has been developed for creative applications and as such it can be compiled as an addon for [**openFrameworks**](https://openframeworks.cc).

It uses nearly optimal features proposed by [Skogstad et al.](http://www.uio.no/english/research/groups/fourms/projects/sma/subprojects/mocapfilters/) for real-time filtering and differentiation. Even though it has been mainly used with Kinect devices, its API allows to provide positional data using standard C++ containers.

Examples and projects for Kinect V1 and Kinect V2 devices are provided for OSX (only V1) and Windows platforms.

<!-- MarkdownTOC autolink="true" autoanchor="true" bracket="round"-->

- [API](#api)
    - [MoDeExtractor](#modeextractor)
    - [MoDeJoint](#modejoint)
    - [MoDeDescriptor](#modedescriptor)
- [Documentation](#documentation)
- [Copyright](#copyright)
- [Contact](#contact)

<!-- /MarkdownTOC -->


<a name="api"></a>
## API

All MoDe classes and constants are accessible within the `MoDe` namespace. 

<a name="modeextractor"></a>
### MoDeExtractor

The `MoDeExtractor` class is used to compute descriptors for a body or set of joints. 

```cpp
MoDe::MoDeExtractor modeExtractor;
```

`MoDeExtractor` computes a number of features from all joints. It does not only give access to the current value of descriptors, but also to statistics such as the mean, standard deviation and RMS values. The number of stored frames from which this computation is done can be defined using the `setup` method. Default is 30 (1 second at 30 fps)

```cpp
modeExtractor.setup(30); 
```

Every time a new MoCap data frame arrives from the device, the `update` must be called passing a map<int, MoDePoint> as argument. This map contains pairs of joints IDs and positional data for the received frame. `MoDePoint` objects can cast `vector<double>`, `vector<float>` and `ofPoint` (for OpenFrameworks). Assuming we have an object `device` where the position of the jth joint at the current frame can be accessed through `device.getJoint(j).getPosition()`, the way to make `ModeExtractor` compute the descriptors for the current frame would be:

```cpp
map <int , MoDe::MoDePoint > joints;
for (int j = 0; j < device.getNumJoints (); j++) {
    joints[j] = device.getJoint(j).getPosition ();
}
modeExtractor.update(joints);
```

<a name="modejoint"></a>
### MoDeJoint
The `MoDeJoint` class objects contain all information from a joint, including the computed descriptor. If, for example, the right hand joint is associated with a constant `RIGHT_HAND`, it can be accessed using `getJoint` as:

```cpp
MoDe::MoDeJoint right_hand = modeExtractor.getJoint(RIGHT_HAND);
```

<a name="modedescriptor"></a>
### MoDeDescriptor
Descriptors can correspond to a single joint (joint descriptors) or can be computed combining information from different joints (body descriptors).

<a name="body-descriptors"></a>
#### Body descriptors
Currently, `MoDe` allows to compute the following body descriptors:

* Quantity Of Motion
* Contraction Index

Both can be accessed directly from the `MoDeExtractor` object as:

```cpp
MoDeDescriptor qom = modeExtractor.getDescriptor(MoDE::DESC_QOM);
```

<a name=""></a>
<a name="joint-descriptors"></a>
#### Joint descriptors
Joint descriptor are computed from a single joint. The current list of computed descriptors is the following:
* Position / filtered position (3D)
* Velocity (3D)
* Acceleration (3D)
* Jerk (3D)
* Acceleration along the trajectory of velocity (1D)

These can be accessed with `getDescriptor` from the `MoDeJoint` using constants defined in the `MoDe` namespace:

```cpp
MoDe::MoDeDescriptor rh_vel = right_hand.getDescriptor(MoDe::DESC_VELOCITY);
```

This ```MoDeDescriptor``` object now contains different information accessible for the right hand velocity descriptor:

```cpp
// MoDePoint with current value of the descriptor
rh_vel.getCurrent();
// double with current velocity in the y axis
rh_vel.getCurrent().y;
// MoDePoint with mean value during the number of configured number of frames
rh_vel.getMean(); 
// double with mean value along the x axis during the configured nr of frames
rh_vel.getMean().x; 
// double with current magnitude of the velocity vector
rh_vel.getMagnitude(); 
```

<a name="documentation"></a>
## Documentation
The automatically generated documentation is available at http://www.alvarosarasua.com/mode-documentation 

<a name="copyright"></a>
## Copyright
Copyright (C) 2017 MTG, Universitat Pompeu Fabra - Escola Superior de Música de Catalunya.
This code has been mainly developed by Álvaro Sarasúa during his PhD thesis, supervised by Emilia Gómez and Enric Guaus. 

<a name="contact"></a>
## Contact
Álvaro Sarasúa: [alvaro.sarasua@upf.edu](mailto:alvarosarasua@upf.edu)

