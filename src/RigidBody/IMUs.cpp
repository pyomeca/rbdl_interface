#define BIORBD_API_EXPORTS
#include "RigidBody/IMUs.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"

biorbd::rigidbody::IMUs::IMUs() :
    m_IMUs(std::make_shared<std::vector<biorbd::rigidbody::IMU>>())
{
    //ctor
}

biorbd::rigidbody::IMUs::IMUs(const biorbd::rigidbody::IMUs &other)
{
    m_IMUs = other.m_IMUs;
}

biorbd::rigidbody::IMUs::~IMUs()
{

}

biorbd::rigidbody::IMUs biorbd::rigidbody::IMUs::DeepCopy() const
{
    biorbd::rigidbody::IMUs copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::IMUs::DeepCopy(const biorbd::rigidbody::IMUs &other)
{
    m_IMUs->resize(other.m_IMUs->size());
    for (unsigned int i=0; i<other.m_IMUs->size(); ++i) {
        (*m_IMUs)[i] = (*other.m_IMUs)[i].DeepCopy();
    }
}

void biorbd::rigidbody::IMUs::addIMU(
    bool technical,
    bool anatomical)
{
    m_IMUs->push_back(biorbd::rigidbody::IMU(technical, anatomical));
}

// Add a new marker to the existing pool of markers
void biorbd::rigidbody::IMUs::addIMU(
    const biorbd::utils::RotoTransNode &RotoTrans,
    bool technical,
    bool anatomical)
{
    m_IMUs->push_back(biorbd::rigidbody::IMU(RotoTrans, technical, anatomical));
}

unsigned int biorbd::rigidbody::IMUs::nbIMUs() const
{
    return static_cast<unsigned int>(m_IMUs->size());
}


// Get the markers in the global reference
const std::vector<biorbd::rigidbody::IMU>& biorbd::rigidbody::IMUs::IMU() const
{
    return *m_IMUs;
}

std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::IMU(
    const biorbd::utils::String& segmentName)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs();
            ++i) // Scan through all the markers and select the good ones
        if (!IMU(i).parent().compare(segmentName)) {
            pos.push_back(IMU(i));
        }
    return pos;
}

const biorbd::rigidbody::IMU& biorbd::rigidbody::IMUs::IMU(
    unsigned int idx)
{
    return (*m_IMUs)[idx];
}

// Get the IMUs at the position given by Q
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::IMU(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i) {
        pos.push_back(IMU(Q, i, updateKin));
        updateKin = false;
    }

    return pos;
}

// Get an IMU at the position given by Q
biorbd::rigidbody::IMU biorbd::rigidbody::IMUs::IMU(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>
                                       (*this);
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        model.UpdateKinematicsCustom (&Q);
    }

    biorbd::rigidbody::IMU node = IMU(idx);
    unsigned int id = static_cast<unsigned int>(model.GetBodyBiorbdId(
                          node.parent()));

    return model.globalJCS(id) * node;
}

// Get the technical IMUs
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::technicalIMU(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isTechnical() ) {
            pos.push_back(IMU(Q, i, updateKin));
            updateKin = false;
        }
    return pos;
}
// Get the technical IMUs in the local reference
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::technicalIMU()
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isTechnical() ) {
            pos.push_back(IMU(i));    // Forward kinematics
        }
    return pos;
}

// Get all the anatomical IMUs
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::anatomicalIMU(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isAnatomical() ) {
            pos.push_back(IMU(Q, i, updateKin));
            updateKin = false;
        }
    return pos;
}
// Get the anatomical IMUs in the local reference
std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::anatomicalIMU()
{
    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if ( IMU(i).isAnatomical() ) {
            pos.push_back(IMU(i));    // Forward kinematics
        }
    return pos;
}

std::vector<biorbd::rigidbody::IMU> biorbd::rigidbody::IMUs::segmentIMU(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    unsigned int idx,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>
                                       (*this);

    // Segment name to find
    biorbd::utils::String name(model.segment(idx).name());

    std::vector<biorbd::rigidbody::IMU> pos;
    for (unsigned int i=0; i<nbIMUs();
            ++i) // scan all the markers and select the right ones
        if (!((*m_IMUs)[i]).parent().compare(name)) {
            pos.push_back(IMU(Q,i,updateKin));
            updateKin = false;
        }

    return pos;
}

// Get the Jacobian of the markers
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::IMUs::IMUJacobian(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, false);
}

// Get the Jacobian of the technical markers
std::vector<biorbd::utils::Matrix>
biorbd::rigidbody::IMUs::TechnicalIMUJacobian(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin)
{
    return IMUJacobian(Q, updateKin, true);
}


// Protected function
std::vector<biorbd::utils::Matrix> biorbd::rigidbody::IMUs::IMUJacobian(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    bool updateKin,
    bool lookForTechnical)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    biorbd::rigidbody::Joints &model = dynamic_cast<biorbd::rigidbody::Joints &>
                                       (*this);

    std::vector<biorbd::utils::Matrix> G;

    for (unsigned int idx=0; idx<nbIMUs(); ++idx) {
        // Actual marker
        biorbd::rigidbody::IMU node = IMU(idx);
        if (lookForTechnical && !node.isTechnical()) {
            continue;
        }

        unsigned int id = model.GetBodyId(node.parent().c_str());
        biorbd::utils::Matrix G_tp(biorbd::utils::Matrix::Zero(9,model.dof_count));

        // Calculate the Jacobian of this Tag
        model.CalcMatRotJacobian(Q, id, node.rot(), G_tp, updateKin); // False for speed

        G.push_back(G_tp);
    }

    return G;
}

unsigned int biorbd::rigidbody::IMUs::nbTechIMUs()
{
    unsigned int nbTech = 0;
    if (nbTech == 0) // If the function has never been called before
        for (biorbd::rigidbody::IMU imu : *m_IMUs)
            if (imu.isTechnical()) {
                ++nbTech;
            }

    return nbTech;
}

unsigned int biorbd::rigidbody::IMUs::nbAnatIMUs()
{
    unsigned int nbAnat = 0;
    if (nbAnat == 0) // If the function has never been called before
        for (biorbd::rigidbody::IMU imu : *m_IMUs)
            if (imu.isAnatomical()) {
                ++nbAnat;
            }

    return nbAnat;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::IMUs::IMUsNames()
{
    // Extract the name of all the markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i) {
        names.push_back(IMU(i).biorbd::utils::Node::name());
    }
    return names;
}

std::vector<biorbd::utils::String> biorbd::rigidbody::IMUs::technicalIMUsNames()
{
    // Extract the names of all the technical markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if (IMU(i).isTechnical()) {
            names.push_back(IMU(i).biorbd::utils::Node::name());
        }

    return names;
}

std::vector<biorbd::utils::String>
biorbd::rigidbody::IMUs::anatomicalIMUsNames()
{
    // Extract the names of all the anatomical markers of a model
    std::vector<biorbd::utils::String> names;
    for (unsigned int i=0; i<nbIMUs(); ++i)
        if (IMU(i).isAnatomical()) {
            names.push_back(IMU(i).biorbd::utils::Node::name());
        }

    return names;
}
