#ifndef BIORBD_MATLAB_HELP_H
#define BIORBD_MATLAB_HELP_H

#include <iostream>

void Matlab_help()
{

    // Afficher toutes les commandes ainsi que leur définition

    std::cout << "Here are the commands you can call from this interface" <<
              std::endl;
    std::cout << std::endl;
    std::cout << "biorbd('help')" 				<< std::endl << "\t=> show help menu"
              << std::endl;
    std::cout << "h = biorbd('new', pathToModel)" << std::endl <<
              "\t=> load a new model, Return a handler (h)" 							<<
              std::endl;
    std::cout << "biorbd('delete', h)" 			<< std::endl <<
              "\t=> delete the loaded 'h' model" 									<< std::endl <<
              std::endl;

    std::cout << "biorbd('gravity, h, GravityVector)" << std::endl <<
              "\t=> Change gravity field for the given 3d column vector" << std::endl;
    std::cout << "nBody = biorbd('nBody', h)"			<< std::endl <<
              "\t=> Return the number of segments" << std::endl;
    std::cout << "nameBody = biorbd('nameBody', h)"			<< std::endl <<
              "\t=> Return the name of segments" << std::endl;
    std::cout << "Mass = biorbd('totalMass', h)" 		<< std::endl <<
              "\t=> Return the total mass of the model 'h'" 						 <<
              std::endl;
    std::cout << "Mass = biorbd('segmentmass', h, [idx])" 		<< std::endl <<
              "\t=> Return a vector of mass of each segment of the model 'h' or only idxth segment"
              << std::endl;
    std::cout << "massMatrix = biorbd('massMatrix', h, Q)" 		<< std::endl <<
              "\t=> Return the mass matrix for the model 'h' at Q" 									<< std::endl;
    std::cout << "nDof = biorbd('nDof', h)" 				<< std::endl <<
              "\t=> Return the number of Degrees of Freedom (DoF) for model 'h'" 	<<
              std::endl;
    std::cout << "nameDof = biorbd('nameDof', h)" 				<< std::endl <<
              "\t=> Return the name of Degrees of Freedom (DoF) for model 'h'" 	 << std::endl;
    std::cout << "nGeneralizedTorque = biorbd('nGeneralizedTorque', h)" 			<<
              std::endl <<
              "\t=> Return the number of GeneralizedTorque DoF for model 'h'" 			 <<
              std::endl;
    std::cout << "nRoot = biorbd('nRoot', h)" 			<< std::endl <<
              "\t=> Return the number of DoF on root segment for model 'h'" 		<< std::endl <<
              std::endl;

    std::cout << "CoM = biorbd('CoM', h, Q)" 				<< std::endl <<
              "\t=> Return the center of mass (CoM) for the model 'h' at Q" 							<<
              std::endl;
    std::cout << "CoMJac = biorbd('CoMJacobian', h, Q)" 		<< std::endl <<
              "\t=> Return the CoM jacobian for the model 'h' at Q" 	<< std::endl;
    std::cout << "CoMdot = biorbd('CoMdot', h, Q, Qdot)" 		<< std::endl <<
              "\t=> Return the CoM velocity for the model 'h' at Q and generalized velocities Qdot"
              << std::endl;
//    std::cout << "CoMddot = biorbd('CoMddot', h, Q, Qddot)" 		<< std::endl << "\t=> Return the CoM acceleration for the model 'h' at Q and generalized accelerations Qdot" 	<< std::endl;
    std::cout << "a = biorbd('CoMangularMomentum', h, Q, Qdot)"	<< std::endl <<
              "\t => Return the angular momentum at CoM for Q and Qdot" << std::endl <<
              std::endl;

    std::cout << "GL = biorbd('globalJCS', h, Q)" 			<< std::endl <<
              "\t=> Return joint coordinate systems (JCS) in global reference of all segment of model 'h' at generalized coordinates Q"
              << std::endl;
    std::cout << "CoM = biorbd('segmentCom', h, Q, [i])" 		<< std::endl <<
              "\t=> Return the segmental CoM for the model 'h' at Q. It returns it for all segments unless optional body index i is provided"
              << std::endl;
    std::cout << "CoMdot = biorbd('segmentComDot', h, Q, Qdot, [i])" 		<< std::endl
              <<
              "\t=> Return the segmental CoM velocity for the model 'h' at Q and Qdot. It returns it for all segments unless optional body index i is provided"
              << std::endl;
    std::cout << "CoMddot = biorbd('segmentComDDot', h, Q, Qdot, Qddot, [i])" 		<<
              std::endl <<
              "\t=> Return the segmental CoM acceleration for the model 'h' at Q, Qdot and Qddot. It returns it for all segments unless optional body index i is provided"
              << std::endl;
    std::cout << "a = biorbd('segmentAngularMomentum', h, Q, Qdot, [i])"	<<
              std::endl <<
              "\t => Return the segmental angular momentum at CoM for Q and Qdot in column matrix (sum of over columns should be equal to comAngularMomentum). If 'i is provided, it returns only for this segment"
              << std::endl;
    std::cout << "I = biorbd('segmentsInertia', h, Q)" 			<< std::endl <<
              "\t=> Return the 6x6 inertia matrix for all segment for the model 'h' at Q"
              << std::endl;
    std::cout << "I = biorbd('segmentsInertiaLocal', h, [idx])" 			<< std::endl <<
              "\t=> Return the 3x3 inertia matrix for all segment for the model 'h'. If idx is provided, only asked segment is returned"
              << std::endl;
    std::cout << "w = biorbd('segmentsVelocities', h, Q, Qdot)" 	<< std::endl <<
              "\t=> Return Plücker's velocity vector for each segment of the model 'h' at Q and Qdot"
              << std::endl << std::endl;

    std::cout << "nMarkers = biorbd('nMarkers', h)" 			<< std::endl <<
              "\t=> Return the number of markers for the model 'h'"
              << std::endl;
    std::cout << "nameMarkers = biorbd('nameMarkers', h)" 			<< std::endl <<
              "\t=> Return the names of the markers for the model 'h'" 				<< std::endl ;
    std::cout << "nameMarkers = biorbd('nameTechnicalMarkers', h)" 			<< std::endl
              <<
              "\t=> Return the names of the technical markers for the model 'h'" 				<<
              std::endl;
    std::cout << "nameMarkers = biorbd('nameAnatomicalMarkers', h)" 			<< std::endl
              <<
              "\t=> Return the names of the anatomical markers for the model 'h'" 				<<
              std::endl ;
    std::cout <<
              "T = biorbd('LocalMarkers', h, ['all' (default), 'technical', 'anatomical'])"
              << std::endl <<
              "\t=> Return the position (X,Y,Z) in lcoal reference of 'all', 'technical' or 'anatomical' markers of the model 'h' at Q"
              << std::endl;
    std::cout << "T = biorbd('Markers', h, Q)" 			<< std::endl <<
              "\t=> Return the position (X,Y,Z) in global reference of all markers of the model 'h' at Q"
              << std::endl;
    std::cout << "T = biorbd('segmentsMarkers', h, Q)" 	<< std::endl <<
              "\t=> Return Markers for each segment (in cells) of the model 'h' at Q" <<
              std::endl;
    std::cout << "TJ = biorbd('MarkersJacobian', h, Q)"	<< std::endl <<
              "\t=> Return the jacobian matrix of all markers of the model 'h' at Q" <<
              std::endl;
    std::cout << "C = biorbd('contacts', h, Q)" << std::endl <<
              "\t=> Return the position of contact points (p) of the model 'h' at Q" <<
              std::endl;
    std::cout << "m = biorbd('mesh', h, Q)" << std::endl <<
              "\t=> Return position of all points of the model meshing (m) of the model 'h' at Q"
              << std::endl << std::endl;

    std::cout << "IK = biorbd('ik', h, Mark, Qinit)"      << std::endl <<
              "\t=> Return inverse kinematics of the model 'h' for measured markers Mark (3xNxTime) with initial guess Q"
              <<
              std::endl;
    std::cout << "Dyn = biorbd('inverseDynamics', h, Q, Qdot, Qddot, [F])"      <<
              std::endl <<
              "\t=> Return GeneralizedTorque of the model 'h' at Q, Qdot, Qddot and optional Forces (F) [[MOMENT;FORCE] x NB_forcePlates matrix x time]"
              << std::endl;
    std::cout << "Dyn = biorbd('nlEffects', h, Q, Qdot)"      << std::endl <<
              "\t=> Return GeneralizedTorque from NonLinear Effects of the model 'h' at Q, Qdot"
              << std::endl;
    std::cout <<
              "Qddot = biorbd('forwardDynamics', h, Q, Qdot, GeneralizedTorque, [useContact])"
              << std::endl <<
              "\t=> Return Qddot of the model 'h' at Q, Qdot and GeneralizedTorque with optional boolean to process dynamic with contact or not"
              << std::endl;
    std::cout << "Dyn = biorbd('torqueActivation', h, Q, Qdot, activations)"      <<
              std::endl <<
              "\t=> Return GeneralizedTorque of the model 'h' at Q, Qdot and given torque activation"
              << std::endl;
    std::cout <<
              "musclesGeneralizedTorque = biorbd('jointTorqueFromForce', h, Q, Qdot, F)" <<
              std::endl <<
              "\t=> Return muscular GeneralizedTorque of the model 'h' generated by an activation e [nMuscle x time] at Q and QDot"
              <<
              std::endl;
    std::cout <<
              "musclesGeneralizedTorque = biorbd('jointTorqueFromActivation', h, Q, Qdot, a)"
              << std::endl <<
              "\t=> Return muscular GeneralizedTorque of the model 'h' generated by an activation a [nMuscle x time] at Q and QDot"
              <<
              std::endl << std::endl;
    std::cout <<
              "musclesGeneralizedTorque = biorbd('jointTorqueFromExcitation', h, Q, Qdot, a)"
              << std::endl <<
              "\t=> Return muscular GeneralizedTorque of the model 'h' generated by an excitation a [nMuscle x time] at Q and QDot"
              <<
              std::endl << std::endl;

    std::cout << "nMus = biorbd('nmuscles', h)" << std::endl <<
              "\t=> Return total number of muscles of model 'h'" <<
              std::endl;
    std::cout <<
              "[groupsNames, musclesNames, pathNames] = biorbd('musclesNames', h)" <<
              std::endl <<
              "\t=> Return all groups names, muscle names and via points/wrapping names of model 'h'"
              << std::endl;
    std::cout << "[m, w] = biorbd('musclepoints', h, Q)" << std::endl <<
              "\t=> Return all muscles points (m) in a cell and all wrapping objects info (w) of model 'h' at Q"
              << std::endl;
    std::cout << "l = biorbd('muscleLength', h, Q)" << std::endl <<
              "\t=> Return all muscles length of model 'h' at Q" <<
              std::endl;
    std::cout << "l = biorbd('muscleVelocity', h, Q, Qdot)" << std::endl <<
              "\t=> Return all muscles velocities of model 'h' at Q, Qdot" << std::endl;
    std::cout <<
              "adot = biorbd('muscleActivationDot', h, e, a, [alreadyNormalized=false])" <<
              std::endl <<
              "\t=> Return all muscles activation dot of model 'h' at excitation e and activation a, with default excitation already normalized set to false"
              << std::endl << std::endl;
    return;

}
#endif // BIORBD_MATLAB_HELP_H
