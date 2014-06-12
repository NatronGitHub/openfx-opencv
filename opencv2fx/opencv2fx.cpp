#include "opencv2fx.h"

#include <iostream>
#include <stdexcept>

namespace OFX {
    /** @brief Throws an @ref OFX::Exception depending on the status flag passed in */
    void throwSuiteStatusException(OfxStatus stat) throw(OFX::Exception::Suite, std::bad_alloc)
    {
        switch (stat)
        {
            case kOfxStatOK :
            case kOfxStatReplyYes :
            case kOfxStatReplyNo :
            case kOfxStatReplyDefault :
                break;

            case kOfxStatErrMemory :
                throw std::bad_alloc();

            default :
                std::cout << "Threw suite exception!" << std::endl;
                throw OFX::Exception::Suite(stat);
        }
    }

    void throwHostMissingSuiteException(std::string name) throw(OFX::Exception::Suite)
    {
        std::cout << "Threw suite exception! Host missing '" << name << "' suite." << std::endl;
        throw OFX::Exception::Suite(kOfxStatErrUnsupported);
    }


    /** @brief maps status to a string */
    const char* mapStatusToString(OfxStatus stat)
    {
        switch(stat)
        {
            case kOfxStatOK             : return "kOfxStatOK";
            case kOfxStatFailed         : return "kOfxStatFailed";
            case kOfxStatErrFatal       : return "kOfxStatErrFatal";
            case kOfxStatErrUnknown     : return "kOfxStatErrUnknown";
            case kOfxStatErrMissingHostFeature : return "kOfxStatErrMissingHostFeature";
            case kOfxStatErrUnsupported : return "kOfxStatErrUnsupported";
            case kOfxStatErrExists      : return "kOfxStatErrExists";
            case kOfxStatErrFormat      : return "kOfxStatErrFormat";
            case kOfxStatErrMemory      : return "kOfxStatErrMemory";
            case kOfxStatErrBadHandle   : return "kOfxStatErrBadHandle";
            case kOfxStatErrBadIndex    : return "kOfxStatErrBadIndex";
            case kOfxStatErrValue       : return "kOfxStatErrValue";
            case kOfxStatReplyYes       : return "kOfxStatReplyYes";
            case kOfxStatReplyNo        : return "kOfxStatReplyNo";
            case kOfxStatReplyDefault   : return "kOfxStatReplyDefault";
            case kOfxStatErrImageFormat : return "kOfxStatErrImageFormat";
        }
        return "UNKNOWN STATUS CODE";
    }
}

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
  //OfxParamHandle param;
  OfxPropertySetHandle props = 0;
  OfxStatus stat;

  stat = gParamHost->paramDefine(effectParams, kOfxParamTypeDouble, name, &props);
  OFX::throwSuiteStatusException(stat);

  // say we are a scaling parameter
  stat = gPropHost->propSetString(props, kOfxParamPropDoubleType, 0, kOfxParamDoubleTypeScale);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetDouble(props, kOfxParamPropDefault, 0, initial);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetDouble(props, kOfxParamPropMin, 0, 0.0);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetDouble(props, kOfxParamPropDisplayMin, 0, min);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetDouble(props, kOfxParamPropDisplayMax, 0, max);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetString(props, kOfxParamPropHint, 0, hint);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetString(props, kOfxParamPropScriptName, 0, name);
  OFX::throwSuiteStatusException(stat);
  stat = gPropHost->propSetString(props, kOfxPropLabel, 0, label);
  OFX::throwSuiteStatusException(stat);
}
