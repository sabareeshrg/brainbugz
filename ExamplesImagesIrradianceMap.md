# Introduction #

This example uses ImageConvolveOp to build an irradiance environment map from a given environment map image. It convolves image channels using Spherical Harmonics with any given number of bands. Using 3 bands you get diffuse irradiance environment maps. Using more bands you get sharper maps ( for specular convolution ).

Based on _"An efficient representation for irradiance environment maps."_ by Ramamoorthi, Ravi and Hanrahan, Pat - 2001.

# Creating an irradiance map #

The code below requires changing the the input and output images files.

```
import IECore

# creates the convolve Op. If you have more images, reusing this instance is faster.
convolve = IECore.ImageConvolveOp()

# load image
envMap = IECore.Reader.create( "/myPath/inputImage.exr" )()

# create irradiance map
irradianceMap = convolve( input = envMap, bands = 3, samples = 100000 )

# save to disk
IECore.Writer.create( irradianceMap, "/myPath/outputImage.exr" )()

```