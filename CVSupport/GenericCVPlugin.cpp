/*
 OFX Generic OpenCV plug-in plugin.
 
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
#include "GenericCVPlugin.h"
#include "ofxsPixelProcessor.h"
#include "ofxsLut.h"

using namespace OFX;


CVImageWrapper::CVImageWrapper()
: _cvImgHeader(0)
, _mem(0)
{
    
}

void
CVImageWrapper::initialise(OFX::ImageEffect* instance,const OfxRectI& bounds,int nComps)
{
    _mem.reset(new ImageMemory((bounds.x2 - bounds.x1) * (bounds.y2 - bounds.y1) * sizeof(unsigned char) * nComps,instance));
    
    CvSize imageSize = cvSize(bounds.x2 - bounds.x1,
                              bounds.y2 - bounds.y1);
    

    _cvImgHeader = cvCreateImageHeader(imageSize,
                                       IPL_DEPTH_8U,
                                       nComps);
    
    _cvImgHeader->imageData = (char*) _mem->lock();
    _cvImgHeader->widthStep = bounds.x2 - bounds.x1;

}

CVImageWrapper::~CVImageWrapper()
{
    cvReleaseImageHeader(&_cvImgHeader);
}

unsigned char*
CVImageWrapper::getData() const
{
    return (unsigned char*)_cvImgHeader->imageData;
}

GenericCVPlugin::GenericCVPlugin(OfxImageEffectHandle handle)
: ImageEffect(handle)
, dstClip_(0)
, srcClip_(0)
, _srgbLut(OFX::Color::LutManager::sRGBLut<OFX::MultiThread::Mutex>())
{
    dstClip_ = fetchClip(kOfxImageEffectOutputClipName);
    srcClip_ = fetchClip(kOfxImageEffectSimpleSourceClipName);
}

void
GenericCVPlugin::fetchCVImage(OFX::Image* img,const OfxRectI& renderWindow,bool copyData,CVImageWrapper* cvImg)
{
  
    void* pixelData = NULL;
    OfxRectI bounds;
    OFX::PixelComponentEnum pixelComponents;
    OFX::BitDepthEnum bitDepth;
    int rowBytes;
    getImageData(img, &pixelData, &bounds, &pixelComponents, &bitDepth, &rowBytes);
    
    int nChannels;
    switch (pixelComponents) {
        case OFX::ePixelComponentAlpha:
            nChannels = 1;
            break;
        case OFX::ePixelComponentRGB:
            nChannels = 3;
            break;
        case OFX::ePixelComponentRGBA:
            nChannels = 4;
            break;
        default:
            assert(false);
            break;
    }

    
    cvImg->initialise(this, renderWindow, pixelComponents);
    

    
    if (copyData) {
        
        OfxRectI convertWindow;
        convertWindow.x1 = convertWindow.y1 = 0;
        convertWindow.x2 = renderWindow.x2 - renderWindow.x1;
        convertWindow.y2 = renderWindow.y2 - renderWindow.y1;
        _srgbLut->to_byte_packed(cvImg->getData(),
                                 (const float*)img->getPixelAddress(renderWindow.x1, renderWindow.y1),
                                 convertWindow,
                                 nChannels,
                                 bounds,
                                 rowBytes,
                                 renderWindow,
                                 (renderWindow.x2 - renderWindow.x1) * sizeof(unsigned char) * nChannels);
    }
}

void
GenericCVPlugin::cvImageToOfxImage(OFX::Image* img,const OfxRectI& renderWindow,const CVImageWrapper& cvImg)
{
    void* pixelData = NULL;
    OfxRectI bounds;
    OFX::PixelComponentEnum pixelComponents;
    OFX::BitDepthEnum bitDepth;
    int rowBytes;
    getImageData(img, &pixelData, &bounds, &pixelComponents, &bitDepth, &rowBytes);
    int nChannels;
    switch (pixelComponents) {
        case OFX::ePixelComponentAlpha:
            nChannels = 1;
            break;
        case OFX::ePixelComponentRGB:
            nChannels = 3;
            break;
        case OFX::ePixelComponentRGBA:
            nChannels = 4;
            break;
        default:
            assert(false);
            break;
    }
    _srgbLut->from_byte_packed((float*)img->getPixelAddress(renderWindow.x1, renderWindow.y1),
                               cvImg.getData(),
                               renderWindow,
                               nChannels,
                               renderWindow,
                               (renderWindow.x2 - renderWindow.x1) * sizeof(unsigned char) * nChannels,
                               bounds,
                               rowBytes);
}

void
genericCVDescribe(const std::string& pluginName,const std::string& pluginGrouping,
                       const std::string& pluginDescription,
                       bool supportsTile,bool supportsMultiResolution,bool temporalClipAccess,
                       OFX::RenderSafetyEnum threadSafety,
                       OFX::ImageEffectDescriptor& desc)
{
    // basic labels
    desc.setLabels(pluginName, pluginName, pluginName);
    desc.setPluginGrouping(pluginGrouping);
    desc.setPluginDescription(pluginDescription);
    
    // add the supported contexts
    desc.addSupportedContext(eContextFilter);
    desc.addSupportedContext(eContextGeneral);
    
    // add supported pixel depths
    desc.addSupportedBitDepth(eBitDepthFloat);
    
    // set a few flags
    desc.setSingleInstance(false);
    desc.setHostFrameThreading(false);
    desc.setSupportsMultiResolution(supportsMultiResolution);
    desc.setSupportsTiles(supportsTile);
    desc.setTemporalClipAccess(temporalClipAccess);
    desc.setRenderTwiceAlways(false);
    desc.setSupportsMultipleClipPARs(false);
    desc.setRenderThreadSafety(threadSafety);
}

void
genericCVDescribeInContextBegin(bool supportsTiles,OFX::ImageEffectDescriptor &desc, OFX::ContextEnum context)
{
    // Source clip only in the filter context
    // create the mandated source clip
    ClipDescriptor *srcClip = desc.defineClip(kOfxImageEffectSimpleSourceClipName);
    srcClip->addSupportedComponent(ePixelComponentRGBA);
    srcClip->addSupportedComponent(ePixelComponentRGB);
    srcClip->addSupportedComponent(ePixelComponentAlpha);
    srcClip->setTemporalClipAccess(false);
    srcClip->setSupportsTiles(supportsTiles);
    srcClip->setIsMask(false);
    
    // create the mandated output clip
    ClipDescriptor *dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
    dstClip->addSupportedComponent(ePixelComponentRGBA);
    dstClip->addSupportedComponent(ePixelComponentRGB);
    dstClip->addSupportedComponent(ePixelComponentAlpha);
    dstClip->setSupportsTiles(supportsTiles);

 
}


