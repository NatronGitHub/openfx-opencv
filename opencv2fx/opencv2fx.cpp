#include "opencv2fx.h"

void defineDoubleParam( OfxPropertySuiteV1 *gPropHost,
			OfxParameterSuiteV1 *gParamHost,
			OfxParamSetHandle effectParams,
			const char *name,
			const char *label,
			const char *hint,
			double min,
			double max,
			double initial)
{
  OfxParamHandle param;
  OfxPropertySetHandle props;
  gParamHost->paramDefine(effectParams, kOfxParamTypeDouble, name, &props);

  // say we are a scaling parameter
  gPropHost->propSetString(props, kOfxParamPropDoubleType, 0, kOfxParamDoubleTypeScale);
  gPropHost->propSetDouble(props, kOfxParamPropDefault, 0, initial);
  gPropHost->propSetDouble(props, kOfxParamPropMin, 0, 0.0);
  gPropHost->propSetDouble(props, kOfxParamPropDisplayMin, 0, min);
  gPropHost->propSetDouble(props, kOfxParamPropDisplayMax, 0, max);
  gPropHost->propSetString(props, kOfxParamPropHint, 0, hint);
  gPropHost->propSetString(props, kOfxParamPropScriptName, 0, name);
  gPropHost->propSetString(props, kOfxPropLabel, 0, label);
}
