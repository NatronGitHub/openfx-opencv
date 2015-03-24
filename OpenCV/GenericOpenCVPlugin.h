/*
   OFX Generic OpenCV plugin.

   Copyright (C) 2014 INRIA

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

   Redistributions in binary form must reproduce the above copyright notice, this
   list of conditions and the following disclaimer in the documentation and/or
   other materials provided with the distribution.

   Neither the name of the {organization} nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   INRIA
   Domaine de Voluceau
   Rocquencourt - B.P. 105
   78153 Le Chesnay Cedex - France
 */

#ifndef __GenericOpenCVPlugin_h__
#define __GenericOpenCVPlugin_h__

#include "ofxsImageEffect.h"
#include "ofxsMacros.h"

#include <cv.h>

namespace OFX {
namespace Color {
class LutBase;
}
}


//8bit sRGB images
class CVImageWrapper
{
public:

    CVImageWrapper();

    ~CVImageWrapper();

    void initialize(OFX::ImageEffect* instance, const OfxRectI & bounds, OFX::PixelComponentEnum pixelComponents, OFX::BitDepthEnum bitDepth);

    unsigned char* getData() const;
    IplImage* getIplImage() const
    {
        return _cvImgHeader;
    }

private:

    IplImage* _cvImgHeader;
    std::auto_ptr<OFX::ImageMemory> _mem;
};

class GenericOpenCVPlugin
    : public OFX::ImageEffect
{
public:
    /** @brief ctor */
    GenericOpenCVPlugin(OfxImageEffectHandle handle);

private:
    // override the roi call
    //virtual void getRegionsOfInterest(const OFX::RegionsOfInterestArguments &args, OFX::RegionOfInterestSetter &rois) OVERRIDE FINAL;

    //virtual bool getRegionOfDefinition(const OFX::RegionOfDefinitionArguments &args, OfxRectD &rod) OVERRIDE FINAL;

    /* Override the render */
    //virtual void render(const OFX::RenderArguments &args) OVERRIDE FINAL;

    //virtual bool isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime) OVERRIDE FINAL;

protected:

    void fetchCVImage8U(const OFX::Image* img, const OfxRectI & renderWindow, bool copyData, CVImageWrapper* dstImg, OFX::PixelComponentEnum dstPixelComponents = OFX::ePixelComponentNone);

    void fetchCVImage8UGrayscale(const OFX::Image* img, const OfxRectI & renderWindow, bool copyData, CVImageWrapper* dstImg);

    void cvImageToOfxImage(const CVImageWrapper & cvImg, const OfxRectI & renderWindow, OFX::Image* img);

    // do not need to delete these, the ImageEffect is managing them for us
    OFX::Clip *_dstClip;
    OFX::Clip *_srcClip;
    const OFX::Color::LutBase* _srgbLut;
};

void genericCVDescribe(const std::string & pluginName,
                       const std::string & pluginGrouping,
                       const std::string & pluginDescription,
                       bool supportsTiles,
                       bool supportsMultiResolution,
                       bool temporalClipAccess,
                       OFX::RenderSafetyEnum threadSafety,
                       OFX::ImageEffectDescriptor & desc);


#endif /* defined(__GenericOpenCVPlugin_h__) */
