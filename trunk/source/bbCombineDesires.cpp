
#include "bbCombineDesiresNode.h"

#include <maya/MPlug.h>
#include <maya/MDataBlock.h>

#include <maya/MFnVectorArrayData.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MVectorArray.h>
#include <maya/MObjectArray.h>
#include <maya/MDataHandle.h>

#include <maya/MGlobal.h>
#include <MTools.h>
#include <iostream.h>

MTypeId     bbCombineDesires::id( 0x3a002 );

// definitions

#define CM_LINEAR				0
#define CM_WEIGHTS				1
#define CM_PRIORITY				2
#define CM_STRONGEST			3
#define CM_WEAKEST				4
#define CM_SELECT				5

#define DEFAULT_WEIGHT			1.0
#define DEFAULT_PRIORITY		1
#define DEFAULT_DITHERINGFACTOR	1.0


// attributes

#define declareAttr(_attr)\
	MObject	className##::##_attr;\

declareAttr(inputForce );
declareAttr(outputForce );

declareAttr(weight);
declareAttr(priority);
declareAttr(ditheringFactor);
declareAttr(numForces);

declareAttr(combineMode );
declareAttr(selectedForce);


bbCombineDesires::bbCombineDesires() {}
bbCombineDesires::~bbCombineDesires() {}

/************************************************************************************/
MStatus bbCombineDesires::compute( const MPlug& plug, MDataBlock& data )
{
	MStatus stat;
//	MGlobal::displayInfo("compute");

	if ( plug == outputForce )	
	{
		// get input forces
		MArrayDataHandle inputForceAD = data.inputArrayValue( inputForce, &stat);
		McheckErr(stat,"bbCombineDesires::compute inputPrioritiesArrayData");
		int numForces  = inputForceAD.elementCount();
		inputForceAD.jumpToElement(0);

		if (numForces > 0)
		{

			// check size of input arrays weights priorities dithering factors
			
/*			// check size of influence data
	
			int influenceDataSize = influenceDataV.length();

			// if necessary update influence data size

			if (influenceDataSize != numForces)
			{
				if (influenceDataSize < numForces)
				{
					for (int e=influenceDataSize-1; e<numForces; e++)
						influenceDataV.append(1.0);	
				}
				else
				{
					do
					{
						influenceDataV.remove(influenceDataSize-1);
						influenceDataSize = influenceDataV.length();
					}
					while (influenceDataV.length() > numForces);
				}
			}
*/
			// get combine forces
			short combineModeV = combineModeValue(data);

			// create needed variables
			MVectorArray outForces;
	
			switch ( combineModeV ) 
			{
				case CM_LINEAR: cdLinear(inputForceAD,numForces,outForces);
								break;
				case CM_WEIGHTS: cdWeights(inputForceAD,numForces,data,outForces);
								break;
				case CM_PRIORITY: cdPriorities(inputForceAD,numForces,data,outForces);
								break;
				case CM_SELECT: cdSelect(inputForceAD,data,outForces);
								break;
			}

			// get output force array from block.
			//

			MDataHandle hOut = data.outputValue( outputForce, &stat);
			McheckErr(stat, "ERROR in hOut = bOutArray.addElement.\n");

			MFnVectorArrayData fnOutputForce;
			MObject dOutputForce = fnOutputForce.create( outForces, &stat );
			
			McheckErr(stat, "ERROR in dOutputForce = fnOutputForce.create\n");

			// update data block with new output force data.
			//
			hOut.set( dOutputForce );
			data.setClean( plug );

			// store influence data

//			hOut = data.outputValue( influenceData, &stat);
//			MFnDoubleArrayData fnInfluenceDataV;

//			dInfluenceDataV = fnInfluenceDataV.create( influenceDataV, &stat );

			// update data block with new influence data
//			hOut.set( dInfluenceDataV );


			outForces.clear();
		}		
		return MS::kSuccess;
	}

	return MS::kSuccess;
}

/************************************************************************************/
// linear combination

void bbCombineDesires::cdLinear(MArrayDataHandle &inputForceAD,int numForces,MVectorArray &outForces)
{
	MStatus stat;
	MDataHandle elementHandle;
	MVectorArray currForces;

	int aI; // array iterator;
	int eI; // element iterator;

	// get first force element
	inputForceAD.jumpToElement(0);
	MDataHandle hCurrForces = inputForceAD.inputValue(&stat);
	MObject dCurrForces = hCurrForces.data();
	currForces = MFnVectorArrayData(dCurrForces).array(&stat);

	int forcesSize = currForces.length();

	// variables to store weights
	MIntArray weights(numForces,0);
	MIntArray weightsSum(forcesSize,0);

	outForces.copy(currForces);
	

	for (aI=1; aI<numForces;aI++)
	{
		inputForceAD.next();
		currForces.clear();
						
		hCurrForces = inputForceAD.inputValue(&stat);
		dCurrForces = hCurrForces.data();
				
		currForces.copy(MFnVectorArrayData(dCurrForces).array(&stat));

		for (eI=0; eI<forcesSize; eI++)
			if (currForces[eI] != MVector::zero)
			{
				outForces[eI] += currForces[eI];
				weightsSum[eI] += 1;
			}
	}

	for (eI=0; eI<forcesSize; eI++)
		if (weightsSum[eI]>0)
			outForces[eI] *= 1/weightsSum[eI];

	weights.clear();
	weightsSum.clear();
}

/************************************************************************************/
// get weights

MDoubleArray bbCombineDesires::getWeights(MDataBlock& data, int numForces)
{
	MStatus stat;
	MDataHandle elementHandle;
	
	MArrayDataHandle weightAD = data.inputArrayValue( weight, &stat);
	McheckErr(stat,"bbCombineForces::getWeights inputWeightsArrayData");

	int numWeights = weightAD.elementCount();
	MDoubleArray weights(numForces, DEFAULT_WEIGHT);

	for (int i=0; i < numWeights; i++)
	{
		stat = weightAD.jumpToElement(i);
		if (stat == MS::kSuccess )
		{
			elementHandle = weightAD.inputValue(&stat);
			weights[i] = elementHandle.asDouble();
		}
	}

	return weights;
}
/************************************************************************************/
// get priorities

MIntArray bbCombineDesires::getPriorities(MDataBlock& data, int numForces)
{
	MStatus stat;
	MDataHandle elementHandle;
	
	MArrayDataHandle priorityAD = data.inputArrayValue( priority, &stat);
	McheckErr(stat,"bbCombineForces::getPriorities inputPrioritiesArrayData");

	int numPriorities = priorityAD.elementCount();

//	cout << "\nnumPrio: " << 	numPriorities ;

	MIntArray priorities(numForces, DEFAULT_PRIORITY);

	for (int i=0; i < numPriorities; i++)
	{
		stat = priorityAD.jumpToElement(i);
		if (stat == MS::kSuccess )
		{
			elementHandle = priorityAD.inputValue(&stat);
			priorities[i] = elementHandle.asShort();
//				cout << "\n+ ";
		}
	}
	
//	cout << "\nprio: " << 	priorities ;

	return priorities;
}
/************************************************************************************/
// weighted combination

void bbCombineDesires::cdWeights(MArrayDataHandle &inputForceAD,int numForces, MDataBlock& data, MVectorArray &outForces)
{
	MStatus stat;
	MDataHandle elementHandle;
	MVectorArray currForces;

	int aI; // array iterator;
	int eI; // element iterator;


	// get weights
	MDoubleArray weights = getWeights(data, numForces);
	

	// get first force element
	inputForceAD.jumpToElement(0);
	MDataHandle hCurrForces = inputForceAD.inputValue(&stat);
	MObject dCurrForces = hCurrForces.data();
	currForces = MFnVectorArrayData(dCurrForces).array(&stat);

	int forcesSize = currForces.length();

	// variables to store weights
	MDoubleArray weightsSum(forcesSize,0);
	MVectorArray emptyForces(forcesSize, MVector::zero);

	outForces.copy(emptyForces);

	// compute outForce on basis of handles
	for (aI=0; aI<numForces;aI++)
	{
		hCurrForces = inputForceAD.inputValue(&stat);
		dCurrForces = hCurrForces.data();
					
		currForces.copy(MFnVectorArrayData(dCurrForces).array(&stat));

		for (eI=0; eI<forcesSize; eI++)
			if (currForces[eI] != MVector::zero)
			{
				outForces[eI] += currForces[eI] * weights[aI] ;
				weightsSum[eI] += weights[aI];
			}
						
		inputForceAD.next();
		currForces.clear();
	}

	// scale outForce with weightedSum

	for (eI=0; eI<forcesSize; eI++)
		if (weightsSum[eI]>0)
			outForces[eI] = outForces[eI] * 1/double(weightsSum[eI]);

	weightsSum.clear();
}	

/************************************************************************************/
//priorized combination

void bbCombineDesires::cdPriorities(MArrayDataHandle &inputForceAD,int numForces, MDataBlock& data, MVectorArray &outForces)
{
	MStatus stat;
	MDataHandle elementHandle;
	MVectorArray currForces;

	int aI; // array iterator;
	int eI; // element iterator;


	// get priorities
	MIntArray priorities = getPriorities(data, numForces);
	
	// get first force element
	inputForceAD.jumpToElement(0);
	MDataHandle hCurrForces = inputForceAD.inputValue(&stat);
	MObject dCurrForces = hCurrForces.data();
	currForces = MFnVectorArrayData(dCurrForces).array(&stat);

	int forcesSize = currForces.length();

	MVectorArray emptyForces(forcesSize, MVector::zero);
	outForces.copy(emptyForces);

	// indicates per particle which priority is active

	MIntArray currPriorities(forcesSize, MAX_INT);

	for (aI=0; aI<numForces;aI++)
	{
		inputForceAD.jumpToElement(aI);
		hCurrForces = inputForceAD.inputValue(&stat);
		dCurrForces = hCurrForces.data();
					
		currForces.copy(MFnVectorArrayData(dCurrForces).array(&stat));
		
		for (eI=0; eI<forcesSize; eI++)
		{
			if ((priorities[aI]<currPriorities[eI])&&(currForces[eI] != MVector::zero))
			{
				outForces[eI] = currForces[eI];
				currPriorities[eI] = priorities[aI];
			}
		}
		currForces.clear();
					
	}

	priorities.clear();
}

/************************************************************************************/
void bbCombineDesires::cdSelect(MArrayDataHandle &inputForceAD, MDataBlock& data, MVectorArray &outForces)
{
	MStatus stat;

	MDataHandle hCurrForces; 
	MObject dCurrForces; 

	short selectedForceV = selectedForceValue(data);
					
	stat = inputForceAD.jumpToElement(selectedForceV);
					
	if (stat==MS::kSuccess)
	{
		hCurrForces = inputForceAD.inputValue(&stat);
		dCurrForces = hCurrForces.data();
		outForces.copy(MFnVectorArrayData(dCurrForces).array(&stat));
	}
}


/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
/************************************************************************************/
/************************************************************************************/

void* bbCombineDesires::creator()
{
	return new bbCombineDesires();
}

/************************************************************************************/

MStatus bbCombineDesires::initialize()
{
	MGlobal::displayInfo("bbCombineDesires... loaded");

	MFnNumericAttribute nAttr;
	MStatus				stat;

	MFnTypedAttribute	tAttr;
	MFnEnumAttribute	eAttr;
	
	inputForce=tAttr.create( "inputForce", "if", MFnVectorArrayData::kVectorArray , &stat );
	stat = tAttr.setArray( true );

//	influenceData = tAttr.create( "influenceData", "id", MFnDoubleArrayData::kDoubleArray , &stat );
//	tAttr.setStorable(true);

	selectedForce = nAttr.create("selectedForce","sef",MFnNumericData::kShort,0);
	nAttr.setMin(0);
	nAttr.setMax(50);
	nAttr.setStorable(true);

	outputForce=tAttr.create( "outputForce", "of", MFnVectorArrayData::kVectorArray , &stat );
	stat = tAttr.setStorable( false );

	weight = nAttr.create("weight","wgt",MFnNumericData::kDouble,1.0);
	nAttr.setArray( true );
	nAttr.setReadable( true );
	nAttr.setStorable( true );
	nAttr.setUsesArrayDataBuilder( true );

	
	priority = nAttr.create("priority","pri",MFnNumericData::kShort,1);
	nAttr.setArray( true );
	nAttr.setReadable( true );
	nAttr.setStorable( true );
	nAttr.setUsesArrayDataBuilder( true );	

	ditheringFactor = nAttr.create("ditheringFactor","df",MFnNumericData::kDouble,1.0);
	nAttr.setMin(0.0);
	nAttr.setMax(1.0);
	nAttr.setArray( true );
	nAttr.setReadable( true );
	nAttr.setStorable( true );
	nAttr.setUsesArrayDataBuilder( true );

	numForces = nAttr.create("numForces","nf",MFnNumericData::kShort,0);
	nAttr.setStorable(true);
	nAttr.setReadable( true );
	nAttr.setWritable( true );


	combineMode = eAttr.create( "combineMode", "cm", CM_LINEAR);
		eAttr.addField("Linear", CM_LINEAR);
		eAttr.addField("Weights", CM_WEIGHTS);
		eAttr.addField("Priority", CM_PRIORITY);
//		eAttr.addField("Strongest", CM_STRONGEST);
//		eAttr.addField("Weakest", CM_WEAKEST);
		eAttr.addField("Selection", CM_SELECT);
	//	eAttr.addField("Priorized Dithering", CM_PRIORIZEDDITHERING);

	eAttr.setStorable(true);

	// Add the attributes we have created to the node

	#define nodeAddAttribute(_attr) 	\
			stat = addAttribute( _attr );	\
			if (!stat) { stat.perror("addAttribute"); return stat;} \

	nodeAddAttribute(inputForce);
	nodeAddAttribute(outputForce);
	nodeAddAttribute(combineMode);
	nodeAddAttribute(selectedForce);
	nodeAddAttribute(weight);
	nodeAddAttribute(priority);
	nodeAddAttribute(ditheringFactor);
	nodeAddAttribute(numForces);

	// Set up a dependency between the input and the output.  

	#define nodeSetDependency(_attr1, _attr2) \
			stat = attributeAffects( _attr1, _attr2 ); \
			if (!stat) { stat.perror("attributeAffects"); return stat;} \

  	nodeSetDependency(inputForce,outputForce);
	nodeSetDependency(selectedForce,outputForce);
	nodeSetDependency(combineMode,outputForce);
	nodeSetDependency(weight,outputForce);
	nodeSetDependency(priority,outputForce);
	nodeSetDependency(ditheringFactor,outputForce);

	return MS::kSuccess;
}


