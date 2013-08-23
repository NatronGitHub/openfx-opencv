#ifndef _OPENCV2FX_H
#define _OPENCV2FX_H

#include "cv.h"
#include "ofxImageEffect.h"
#include "ofxMemory.h"
#include "ofxMultiThread.h"
#include "ofxPixels.h"

#define PLUGIN_GROUPING "www.bratwurstandhaggis.org.uk"

// defines a new control for the plugin which controls a floating point variable
void defineDoubleParam( OfxPropertySuiteV1 *gPropHost,
			OfxParameterSuiteV1 *gParamHost,
			OfxParamSetHandle effectParams,
			const char *name,
			const char *label,
			const char *hint,
			double min,
			double max,
			double initial);


template <class T> inline T 
clamp(T v, int min, int max)
{
  if(v < T(min)) return T(min);
  if(v > T(max)) return T(max);
  return v;
}

#endif
