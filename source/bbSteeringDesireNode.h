#ifndef _bbSteeringDesireNode
#define _bbSteeringDesireNode
//
// Copyright (C) 2002 bbSteeringDesire 
// 
// File: bbSteeringDesireNode.h
//
// Dependency Graph Node: bbSteeringDesire
//
// Author: Maya SDK Wizard
//

#include <iostream.h>

 #include <maya/MVector.h>
 #include <maya/MObject.h>
 #include <maya/MPlug.h>
 #include <maya/MDataBlock.h>
 #include <maya/MPxFieldNode.h>


#define McheckErr(stat, msg)		\
	if ( MS::kSuccess != stat )		\
	{								\
		cerr << msg;				\
		return MS::kFailure;		\
	}

#ifdef className
#undef className
#endif

#define className bbSteeringDesire
 
class bbSteeringDesire : public MPxFieldNode
{
public:
	bbSteeringDesire() {};
	virtual	~bbSteeringDesire() {}; 

	static  void *creator();
	static  MStatus		initialize();
	
	virtual MStatus		compute( const MPlug& plug, MDataBlock& block );

    virtual MStatus getForceAtPoint(const MVectorArray& point,
									const MVectorArray& velocity,
									const MDoubleArray& mass,
									MVectorArray& force,
									double deltaTime);

	// attributes.

	#define declareVector(_attr)\
			static MObject	_attr;\
			static MObject	_attr##X;\
			static MObject	_attr##Y;\
			static MObject	_attr##Z;\

	// combination attributes

	static  MObject	cPriority;			// priority value for combination
	static  MObject	cWeight;			// weight value for combination

	
	// view attributes
	
	static  MObject	useSensorRange;
	static  MObject	sensorRange;		// radius in which bug can sense
	static  MObject	useSensorAngle;
	static  MObject	sensorAngle;		// pitch opening angle

	// COMMON

	static  MObject	steeringDesire;		// enum steering desire
	static  MObject desiredSpeed;
	static  MObject maximumForce;
	static  MObject bugRadius;
	static  MObject lastVector;		// storage for vector data from previous cycles



	static  MObject inverseDesiredSteeringForce;

	// target attributes

	static  MObject targetType;
	static  MObject targetRadius;
	static  MObject	stoppingRange;		// stopping range for arrival steering

	static  MObject inputPoint;	
	static  MObject inputCurve;
	static  MObject inputSurface;
	static  MObject inputMesh;

	static  MObject inputSelection;
	static  MObject inputIndex;

	static  MObject seekTargets;
	static  MObject tolerance;
	static  MObject subTargets;
	static  MObject probeLength;
	static  MObject shadowRange;

	declareVector(target);				// target vector
	declareVector(lastTarget);

	// bug attributes

	declareVector(direction);

		// wander

	static  MObject wanderSphereRadius;

	declareVector(wanderSphereOffset);
	declareVector(wanderMinimumRandomVector);
	declareVector(wanderMaximumRandomVector);

	static  MObject chooseSpeed;
	static  MObject wanderMinimumSpeed;
	static  MObject wanderMaximumSpeed;

	static  MObject currentTime;
	static  MObject initSeedFrame;
	static  MObject seed;

	static  MObject useRandom;

	// neighbor attr

	static  MObject bugDistance; 

	static	MTypeId	id;

private:

	void	getTargetsFromPoint(MDataBlock& block,MVectorArray &target);
	void	getTargetsFromCurve(MDataBlock& block,int posSize,MVectorArray &target);
	void	getTargetsFromSurface(MDataBlock& block,int posSize,MVectorArray &target);
	void	getTargetsFromMesh(MDataBlock& block,int posSize,MVectorArray &target);

	void	sdSeekTargets( MDataBlock& block, const MVectorArray &positions, const MVectorArray &velocities, const MVectorArray &target, MVectorArray &outputForce);
	void	sdMothSeekTargets(  MDataBlock& block, const MVectorArray &positions, const MVectorArray &velocities, const MVectorArray &target, MVectorArray &outputForce);
	void	sdArrivalTargets(  MDataBlock& block, const MVectorArray &positions, const MVectorArray &velocities, const MVectorArray &target, MVectorArray &outputForce);
	void	sdPursuitTargets(  MDataBlock& block, const MVectorArray &positions, const MVectorArray &velocities, const MVectorArray &target, MVectorArray &outputForce);
	void	sdShadowTargets(  MDataBlock& block, const MVectorArray &positions, const MVectorArray &velocities, const MVectorArray &target, const MTime &deltaTime, MVectorArray &outputForce);

	
	void	sdNeighborAlignment( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdNeighborCohesion( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdNeighborSeparation( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdNeighborUnalignedCollisionAvoidance(MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdNeighborKeepDistance(MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdNeighborOportunism(MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	
	void	sdBugHeadDirection( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdBugWander( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	
	void	sdPursuit( MDataBlock& block,
					const MVectorArray &points,
					const MVectorArray &velocities,
					MVectorArray &outputForce );

	void	sdCurveFollowing( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );

	void	sdSurfaceFollowing( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void	sdSurfaceObstacleAvoidance( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );

	void	sdMeshFollowing( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	void    sdMeshObstacleAvoidance( MDataBlock& block, const MVectorArray &points, const MVectorArray &velocities, MVectorArray &outputForce );
	
	// methods to get attribute value of local attributes.
	//
	short	steeringDesireValue( MDataBlock& block );
	short	targetTypeValue( MDataBlock& block );
	short	subTargetsValue(MDataBlock& block );
	short	inputSelectionValue(MDataBlock& block );
	short	inputIndexValue(MDataBlock& block );
	short   seedValue( MDataBlock& block );
	short   chooseSpeedValue(MDataBlock& block );

	double	sensorRangeValue( MDataBlock& block );
	double	sensorAngleValue( MDataBlock& block );
	double	scaleDesiredForceValue( MDataBlock& block );
	double	stoppingRangeValue( MDataBlock& block );
	double  desiredSpeedValue( MDataBlock& block );
	double  maximumForceValue( MDataBlock& block );
	double  targetRadiusValue( MDataBlock& block );
	double  toleranceValue( MDataBlock& block );
	double  bugRadiusValue( MDataBlock& block );
	double  wanderSphereRadiusValue( MDataBlock& block );
	double  wanderMinimumSpeedValue( MDataBlock& block );
	double  wanderMaximumSpeedValue( MDataBlock& block );
	double	shadowRangeValue( MDataBlock& block );
	double  probeLengthValue( MDataBlock& block );
	double  bugDistanceValue( MDataBlock& block );
	bool	useSensorRangeValue( MDataBlock& block );
	bool	useSensorAngleValue( MDataBlock& block );
	bool    inverseDesiredSteeringForceValue(MDataBlock& block);
	bool    useRandomValue(MDataBlock& block);

	MStatus lastTargetValue(MDataBlock& block, MVector &vector );
	MStatus	directionValue( MDataBlock& block, MVector &vector );
	MStatus	wanderMinimumRandomVectorValue( MDataBlock& block, MVector &vector );
	MStatus	wanderMaximumRandomVectorValue( MDataBlock& block, MVector &vector );
	MStatus	wanderSphereOffsetValue( MDataBlock& block, MVector &vector );

	
};

// inlines

// short inlines

#define getShortAttrInline(_attr)\
		inline short className##::##_attr##Value( MDataBlock& block )\
		{	MStatus status;\
			MDataHandle hValue = block.inputValue( _attr, &status );\
			short value = 0;\
			if( status == MS::kSuccess )\
				value = hValue.asShort();\
			return( value );}\

getShortAttrInline( steeringDesire);
getShortAttrInline( targetType);
getShortAttrInline( subTargets);
getShortAttrInline( inputSelection);
getShortAttrInline( inputIndex);
getShortAttrInline( seed);
getShortAttrInline( chooseSpeed);

// double inlines

#define getDoubleAttrInline(_attr)\
		inline double className##::##_attr##Value( MDataBlock& block )\
		{	MStatus status;\
			MDataHandle hValue = block.inputValue( _attr, &status );\
			double value = 0.0;\
			if( status == MS::kSuccess )\
				value = hValue.asDouble();\
			return( value );}\

getDoubleAttrInline( sensorRange);
getDoubleAttrInline( sensorAngle);
getDoubleAttrInline( stoppingRange);

getDoubleAttrInline( desiredSpeed);
getDoubleAttrInline( maximumForce);
getDoubleAttrInline( targetRadius);
getDoubleAttrInline( tolerance);
getDoubleAttrInline( bugRadius);
getDoubleAttrInline( wanderSphereRadius);
getDoubleAttrInline( wanderMinimumSpeed);
getDoubleAttrInline( wanderMaximumSpeed);
getDoubleAttrInline( probeLength);
getDoubleAttrInline( shadowRange);
getDoubleAttrInline( bugDistance);

// bool inlines

#define getBoolAttrInline(_attr)\
		inline bool className##::##_attr##Value( MDataBlock& block )\
		{	MStatus status;\
			MDataHandle hValue = block.inputValue( _attr, &status );\
			bool value = false;\
			if( status == MS::kSuccess )\
				value = hValue.asBool();\
			return( value );}\

getBoolAttrInline( useSensorRange);
getBoolAttrInline( useSensorAngle);
getBoolAttrInline( inverseDesiredSteeringForce);
getBoolAttrInline( useRandom);

// vector inlines

#define getVectorAttrInline(_attr)\
		inline MStatus className##::##_attr##Value(MDataBlock& block,MVector &vector)\
		{	MStatus status;\
			MDataHandle hValueX = block.inputValue( _attr##X, &status );\
			MDataHandle hValueY = block.inputValue( _attr##Y, &status );\
			MDataHandle hValueZ = block.inputValue( _attr##Z, &status );\
			if( status == MS::kSuccess )\
			{	vector[0] = hValueX.asDouble();\
				vector[1] = hValueY.asDouble();\
				vector[2] = hValueZ.asDouble();	}\
			return( status );}\

//getVectorAttrInline(inputPoint);
getVectorAttrInline(lastTarget);
getVectorAttrInline(direction);
getVectorAttrInline(wanderMinimumRandomVector);
getVectorAttrInline(wanderMaximumRandomVector);
getVectorAttrInline(wanderSphereOffset);

#endif
