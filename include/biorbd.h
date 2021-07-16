#ifndef BIORBD_ALL_H
#define BIORBD_ALL_H

#include <rbdl/rbdl.h>

#include "biorbdConfig.h"
#include "BiorbdModel.h"
#include "ModelReader.h"
#include "ModelWriter.h"

#include "Utils/all.h"
#include "RigidBody/all.h"
#ifdef MODULE_MUSCLES
    #include "Muscles/all.h"
#endif
#ifdef MODULE_ACTUATORS
    #include "Actuators/all.h"
#endif

#endif // BIORBD_ALL_H

