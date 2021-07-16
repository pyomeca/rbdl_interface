#define BIORBD_API_EXPORTS
#include "Muscles/MuscleGroup.h"

#include "Utils/Error.h"
#include "Muscles/IdealizedActuator.h"
#include "Muscles/HillType.h"
#include "Muscles/HillThelenType.h"
#include "Muscles/HillThelenActiveOnlyType.h"
#include "Muscles/HillThelenTypeFatigable.h"
#include "Muscles/StateDynamicsBuchanan.h"
#include "Muscles/StateDynamicsDeGroote.h"

biorbd::muscles::MuscleGroup::MuscleGroup() :
    m_mus(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>>()),
    m_name(std::make_shared<biorbd::utils::String>()),
    m_originName(std::make_shared<biorbd::utils::String>()),
    m_insertName(std::make_shared<biorbd::utils::String>())
{

}

biorbd::muscles::MuscleGroup::MuscleGroup(const biorbd::muscles::MuscleGroup
        &other) :
    m_mus(other.m_mus),
    m_name(other.m_name),
    m_originName(other.m_originName),
    m_insertName(other.m_insertName)
{

}

biorbd::muscles::MuscleGroup::MuscleGroup(
    const biorbd::utils::String &name,
    const biorbd::utils::String &originName,
    const biorbd::utils::String &insertionName) :
    m_mus(std::make_shared<std::vector<std::shared_ptr<biorbd::muscles::Muscle>>>()),
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_originName(std::make_shared<biorbd::utils::String>(originName)),
    m_insertName(std::make_shared<biorbd::utils::String>(insertionName))
{
}

biorbd::muscles::MuscleGroup::~MuscleGroup()
{

}

biorbd::muscles::MuscleGroup biorbd::muscles::MuscleGroup::DeepCopy() const
{
    biorbd::muscles::MuscleGroup copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::MuscleGroup::DeepCopy(const biorbd::muscles::MuscleGroup
        &other)
{
    m_mus->resize(other.m_mus->size());
    for (unsigned int i=0; i<other.m_mus->size(); ++i) {
        if ((*other.m_mus)[i]->type() ==
                biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
            (*m_mus)[i] = std::make_shared<biorbd::muscles::IdealizedActuator>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() == biorbd::muscles::MUSCLE_TYPE::HILL) {
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillType>((*other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   biorbd::muscles::MUSCLE_TYPE::HILL_THELEN) {
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillThelenType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>((
                              *other.m_mus)[i]);
        } else if ((*other.m_mus)[i]->type() ==
                   biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
            (*m_mus)[i] = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>((
                              *other.m_mus)[i]);
        } else {
            biorbd::utils::Error::raise("DeepCopy was not prepared to copy " +
                                        biorbd::utils::String(
                                            biorbd::muscles::MUSCLE_TYPE_toStr((*other.m_mus)[i]->type())) + " type");
        }
    }
    *m_mus = *other.m_mus;
    *m_name = *other.m_name;
    *m_originName = *other.m_originName;
    *m_insertName = *other.m_insertName;
}

void biorbd::muscles::MuscleGroup::addMuscle(
    const biorbd::utils::String &name,
    biorbd::muscles::MUSCLE_TYPE type,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    biorbd::muscles::STATE_TYPE stateType,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    std::shared_ptr<biorbd::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == biorbd::muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    } else if (stateType == biorbd::muscles::STATE_TYPE::DYNAMIC) {
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    } else if (stateType == biorbd::muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<biorbd::muscles::StateDynamicsBuchanan>();
    } else if (stateType == biorbd::muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<biorbd::muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    }

    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,
                 characteristics, *state);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,
                 characteristics, *state);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,
                 characteristics, *state);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics, *state);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, *state,
                 dynamicFatigueType);
    } else {
        biorbd::utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
    const biorbd::utils::String &name,
    biorbd::muscles::MUSCLE_TYPE type,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,
                 characteristics);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,
                 characteristics);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,
                 characteristics);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics, dynamicFatigueType);
    } else {
        biorbd::utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
    const biorbd::utils::String& name,
    biorbd::muscles::MUSCLE_TYPE type,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    biorbd::muscles::STATE_TYPE stateType,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;
    std::shared_ptr<biorbd::muscles::StateDynamics> state;

    // Cast the dynamic type
    if (stateType == biorbd::muscles::STATE_TYPE::SIMPLE_STATE) {
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    } else if (stateType == biorbd::muscles::STATE_TYPE::BUCHANAN) {
        state = std::make_shared<biorbd::muscles::StateDynamicsBuchanan>();
    } else if (stateType == biorbd::muscles::STATE_TYPE::DE_GROOTE) {
        state = std::make_shared<biorbd::muscles::StateDynamicsDeGroote>();
    } else {
        state = std::make_shared<biorbd::muscles::StateDynamics>();
    }


    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR)
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL)
        muscle = std::make_shared<biorbd::muscles::HillType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN)
        muscle = std::make_shared<biorbd::muscles::HillThelenType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE)
        muscle = std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>
                 (name,geometry,characteristics,pathModifiers,*state);
    else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE)
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(
                     name,geometry,characteristics,pathModifiers,*state,dynamicFatigueType);
    else {
        biorbd::utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
    const biorbd::utils::String &name,
    biorbd::muscles::MUSCLE_TYPE type,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    std::shared_ptr<biorbd::muscles::Muscle> muscle;

    if (type == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        muscle = std::make_shared<biorbd::muscles::IdealizedActuator>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL) {
        muscle = std::make_shared<biorbd::muscles::HillType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN) {
        muscle = std::make_shared<biorbd::muscles::HillThelenType>(name,geometry,
                 characteristics,pathModifiers);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>(name,
                 geometry,characteristics,pathModifiers);
    } else if (type == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        muscle = std::make_shared<biorbd::muscles::HillThelenTypeFatigable>(name,
                 geometry,characteristics,pathModifiers,
                 dynamicFatigueType);
    } else {
        biorbd::utils::Error::raise("Wrong muscle type");
    }
    addMuscle(*muscle);
}

void biorbd::muscles::MuscleGroup::addMuscle(
    const biorbd::muscles::Muscle &muscle)
{
    biorbd::utils::Error::check(muscleID(muscle.name()) == -1,
                                "This muscle name was already defined for this muscle group");

    // Add muscle according to its type
    if (muscle.type() == biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR) {
        m_mus->push_back(std::make_shared<biorbd::muscles::IdealizedActuator>(muscle));
    } else if (muscle.type() ==
               biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE) {
        m_mus->push_back(std::make_shared<biorbd::muscles::HillThelenTypeFatigable>
                         (muscle));
    } else if (muscle.type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN) {
        m_mus->push_back(std::make_shared<biorbd::muscles::HillThelenType>(muscle));
    } else if (muscle.type() == biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE) {
        m_mus->push_back(std::make_shared<biorbd::muscles::HillThelenActiveOnlyType>
                         (muscle));
    } else if (muscle.type() == biorbd::muscles::MUSCLE_TYPE::HILL) {
        m_mus->push_back(std::make_shared<biorbd::muscles::HillType>(muscle));
    } else {
        biorbd::utils::Error::raise("Muscle type not found");
    }
    return;
}

biorbd::muscles::Muscle& biorbd::muscles::MuscleGroup::muscle(unsigned int idx)
{
    biorbd::utils::Error::check(idx<nbMuscles(),
                                "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}
const biorbd::muscles::Muscle& biorbd::muscles::MuscleGroup::muscle(
    unsigned int idx) const
{
    biorbd::utils::Error::check(idx<nbMuscles(),
                                "Idx asked is higher than number of muscles");
    return *(*m_mus)[idx];
}


unsigned int biorbd::muscles::MuscleGroup::nbMuscles() const
{
    return static_cast<unsigned int>(m_mus->size());
}

std::vector<std::shared_ptr<biorbd::muscles::Muscle>>&
        biorbd::muscles::MuscleGroup::muscles()
{
    return *m_mus;
}

const std::vector<std::shared_ptr<biorbd::muscles::Muscle>>&
        biorbd::muscles::MuscleGroup::muscles() const
{
    return *m_mus;
}

int biorbd::muscles::MuscleGroup::muscleID(const biorbd::utils::String&
        nameToFind)
{
    for (unsigned int i=0; i<m_mus->size(); ++i) {
        if (!nameToFind.compare( (*m_mus)[i]->name()) ) {
            return static_cast<int>(i);
        }
    }
    // If we get here, there is no muscle of that name in this group
    return -1;
}

void biorbd::muscles::MuscleGroup::setName(const biorbd::utils::String& name)
{
    *m_name = name;
}
const biorbd::utils::String &biorbd::muscles::MuscleGroup::name() const
{
    return *m_name;
}

void biorbd::muscles::MuscleGroup::setOrigin(const biorbd::utils::String& name)
{
    *m_originName = name;
}
const biorbd::utils::String &biorbd::muscles::MuscleGroup::origin() const
{
    return *m_originName;
}

void biorbd::muscles::MuscleGroup::setInsertion(const biorbd::utils::String&
        name)
{
    *m_insertName = name;
}
const biorbd::utils::String &biorbd::muscles::MuscleGroup::insertion() const
{
    return *m_insertName;
}
