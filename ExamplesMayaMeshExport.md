Exporting meshes from maya into the custom cortex format is pretty straightforward. See how it can be used in ExamplesProceduralsAnimatedMesh .

```
import os

import maya.cmds

import IECore
import IECoreMaya

def exportMeshes( meshes, destinationDirectory ) :

	if not os.path.exists( destinationDirectory ) :
		os.makedirs( destinationDirectory )	

	for mesh in meshes :
	
		converter = IECoreMaya.FromMayaShapeConverter.create( mesh )
		converted = converter.convert()
		writer = IECore.ObjectWriter( converted, destinationDirectory + "/" + mesh + ".cob" )
		writer.write()

allMeshes = maya.cmds.ls( selection=True, noIntermediate=True, dag=True, type="mesh" )
exportMeshes( allMeshes, "/tmp/myMeshes" )
```