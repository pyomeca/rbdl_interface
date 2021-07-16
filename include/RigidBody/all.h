#ifndef BIORBD_RIGIDBODY_ALL_H
#define BIORBD_RIGIDBODY_ALL_H

#include "biorbdConfig.h"

#include "RigidBody/Segment.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/IMU.h"
#include "RigidBody/IMUs.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Markers.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/RotoTransNodes.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/RigidBodyEnums.h"
#ifdef MODULE_KALMAN
    #include "RigidBody/KalmanRecons.h"
    #include "RigidBody/KalmanReconsIMU.h"
    #include "RigidBody/KalmanReconsMarkers.h"
#endif

#endif // BIORBD_RIGIDBODY_ALL_H

