//
// Copyright (C) 2002 Carsten Kolve 
// 
// File: pluginMain.cpp
//
// Author: Maya SDK Wizard
//

#include <maya/MFnPlugin.h>

#include <bbCombineDesiresNode.h>
#include <bbSteeringDesireNode.h>
#include <maya/MPxNode.h>
// #include <maya/MPxFieldNode.h>

MStatus initializePlugin( MObject obj )

{ 
	MStatus   status;
	MFnPlugin plugin( obj, "Carsten Kolve", "4.0", "Any");

	// register steering desire node
	status = plugin.registerNode( "bbSteeringDesire", 
								  bbSteeringDesire::id, 
								  &bbSteeringDesire::creator,
								  &bbSteeringDesire::initialize,
								  MPxNode::kFieldNode );
	if (!status) 
	{
		status.perror("registerNodeBBSD");
		return status;
	}

	// register combine desires node
	status = plugin.registerNode( "bbCombineDesires", 
								  bbCombineDesires::id, 
								  bbCombineDesires::creator,
								  bbCombineDesires::initialize );

	if (!status) 
	{
		status.perror("registerNodeBBCD");
		return status;
	}

	// register ui

    status = plugin.registerUI("bbCreateUI", "bbDeleteUI");
    if (!status) {
        status.perror("registerUIScripts");
        return status;
    }


	return status;
}

MStatus uninitializePlugin( MObject obj )

{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( bbCombineDesires::id );
	if (!status) {
		status.perror("deregisterNodeBBCD");
		return status;
	}

	status = plugin.deregisterNode( bbSteeringDesire::id );
	
	if (!status) 
	{
		status.perror("deregisterNodeBBSD");
		return status;
	}
	
	return status;
}
