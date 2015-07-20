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
#include "GenericOpenCVPlugin.h"
#include "ofxsPixelProcessor.h"
#include "ofxsLut.h"

using namespace OFX;


CVImageWrapper::CVImageWrapper():
#if CV_MAJOR_VERSION < 3
_cvImgHeader(0),
#else
_cvMat(0),
#endif
_mem(0)
{
}

void
CVImageWrapper::initialize(OFX::ImageEffect* instance,
                           const OfxRectI & bounds,
                           OFX::PixelComponentEnum pixelComponents,
                           int pixelComponentCount,
                           unsigned int rowBytes,
                           OFX::BitDepthEnum bitDepth)
{
    int depth;
    std::size_t bytes;
#if CV_MAJOR_VERSION < 3
    (void)rowBytes;
    switch (bitDepth) {
    case eBitDepthUByte:
        depth = IPL_DEPTH_8U;
        break;
    case eBitDepthUShort:
        depth = IPL_DEPTH_16U;
        break;
    case eBitDepthFloat:
        depth = IPL_DEPTH_32F;
        break;
    default:
        throwSuiteStatusException(kOfxStatErrImageFormat);
        return;
    }
    CvSize imageSize = cvSize(bounds.x2 - bounds.x1,
                              bounds.y2 - bounds.y1);

    _cvImgHeader = cvCreateImageHeader(imageSize,
                                       depth,
                                       pixelComponentCount);
    bytes = _cvImgHeader->imageSize;

#else
    
    switch (bitDepth) {
        case eBitDepthUByte:
            depth = CV_8U;
            break;
        case eBitDepthUShort:
            depth = CV_16U;
            break;
        case eBitDepthFloat:
            depth = CV_32F;
            break;
        default:
            throwSuiteStatusException(kOfxStatErrImageFormat);
            return;
    }
    
    switch (pixelComponentCount) {
        case 1:
            depth = CV_MAKETYPE(depth, 1);
            break;
        case 2:
            depth = CV_MAKETYPE(depth, 2);
            break;
        case 3:
            depth = CV_MAKETYPE(depth, 3);
            break;
        case 4:
            depth = CV_MAKETYPE(depth, 4);
            break;
        default:
            assert(false);
            break;
    }
    bytes = rowBytes * (bounds.y2 - bounds.y1);
#endif

    _mem.reset( new ImageMemory(bytes,instance) );

#if CV_MAJOR_VERSION < 3
    _cvImgHeader->imageData = (char*) _mem->lock();
#else
    _cvMat = new cv::Mat(bounds.y2 - bounds.y1, bounds.x2 - bounds.x1, depth, _mem->lock(),rowBytes);
#endif
}

CVImageWrapper::~CVImageWrapper()
{
#if CV_MAJOR_VERSION < 3
    cvReleaseImageHeader(&_cvImgHeader);
#else
    delete _cvMat;
#endif
}

unsigned char*
CVImageWrapper::getData() const
{
#if CV_MAJOR_VERSION < 3
    return (unsigned char*)_cvImgHeader->imageData;
#else
    return  _cvMat->data;
#endif
}

GenericOpenCVPlugin::GenericOpenCVPlugin(OfxImageEffectHandle handle)
    : ImageEffect(handle)
      , _dstClip(0)
      , _srcClip(0)
      , _srgbLut( OFX::Color::LutManager::sRGBLut<OFX::MultiThread::Mutex>() )
{
    _dstClip = fetchClip(kOfxImageEffectOutputClipName);
    _srcClip = fetchClip(kOfxImageEffectSimpleSourceClipName);
}

void
GenericOpenCVPlugin::fetchCVImage8U(const OFX::Image* img,
                                    const OfxRectI & renderWindow,
                                    bool copyData,
                                    CVImageWrapper* dstImg,
                                    OFX::PixelComponentEnum dstPixelComponents,
                                    int dstPixelComponentCount)
{
    const void* pixelData = NULL;
    OfxRectI bounds;
    OFX::PixelComponentEnum pixelComponents;
    OFX::BitDepthEnum bitDepth;
    int rowBytes;

    getImageData(img, &pixelData, &bounds, &pixelComponents, &bitDepth, &rowBytes);
    int pixelComponentCount = img->getPixelComponentCount();

    const OfxRectI &dstBounds = renderWindow;
    if (dstPixelComponents == ePixelComponentNone && dstPixelComponentCount == 0) {
        dstPixelComponents = pixelComponents;
        dstPixelComponentCount = pixelComponentCount;
    }
    
    //Force 8bit for OpenCV images
    const OFX::BitDepthEnum dstBitDepth = eBitDepthUByte;

    dstImg->initialize(this, dstBounds, dstPixelComponents, dstPixelComponentCount, rowBytes, dstBitDepth);
    unsigned char* dstPixelData = dstImg->getData();
    
#if CV_MAJOR_VERSION < 3
    int dstRowBytes = dstImg->getIplImage()->widthStep;
#else
    int dstRowBytes = rowBytes;
#endif
    
    if (copyData) {
        OfxRectI convertWindow;
        convertWindow.x1 = convertWindow.y1 = 0;
        convertWindow.x2 = renderWindow.x2 - renderWindow.x1;
        convertWindow.y2 = renderWindow.y2 - renderWindow.y1;
        _srgbLut->to_byte_packed_nodither(pixelData, bounds, pixelComponents, pixelComponentCount, bitDepth, rowBytes,
                                          renderWindow,
                                          dstPixelData, dstBounds, dstPixelComponents, dstPixelComponentCount, dstBitDepth, dstRowBytes);
    }
}

void
GenericOpenCVPlugin::fetchCVImage8UGrayscale(const OFX::Image* img,
                                             const OfxRectI & renderWindow,
                                             bool copyData,
                                             CVImageWrapper* cvImg)
{
    const void* pixelData = NULL;
    OfxRectI bounds;
    OFX::PixelComponentEnum pixelComponents;
    OFX::BitDepthEnum bitDepth;
    int rowBytes;

    getImageData(img, &pixelData, &bounds, &pixelComponents, &bitDepth, &rowBytes);
    int pixelComponentCount = img->getPixelComponentCount();
    assert(pixelComponents == ePixelComponentRGBA || pixelComponents == ePixelComponentRGB);
    const OfxRectI &dstBounds = renderWindow;
    const OFX::PixelComponentEnum dstPixelComponents = ePixelComponentAlpha;
    const int dstPixelComponentCount = 1;
    
    //Force 8bit for OpenCV images
    const OFX::BitDepthEnum dstBitDepth = eBitDepthUByte;

    int expectedRowBytes = rowBytes / pixelComponentCount;
    
    cvImg->initialize(this, dstBounds, dstPixelComponents, dstPixelComponentCount, expectedRowBytes, dstBitDepth);
    unsigned char* dstPixelData = cvImg->getData();
    
#if CV_MAJOR_VERSION < 3
    int dstRowBytes = cvImg->getIplImage()->widthStep;
#else
    int dstRowBytes = expectedRowBytes;
#endif

    if (copyData) {
        OfxRectI convertWindow;
        convertWindow.x1 = convertWindow.y1 = 0;
        convertWindow.x2 = renderWindow.x2 - renderWindow.x1;
        convertWindow.y2 = renderWindow.y2 - renderWindow.y1;
        _srgbLut->to_byte_grayscale_nodither(pixelData, bounds, pixelComponents, pixelComponentCount, bitDepth, rowBytes,
                                             renderWindow,
                                             dstPixelData, dstBounds, dstPixelComponents, dstPixelComponentCount, dstBitDepth, dstRowBytes);
    }
}

void
GenericOpenCVPlugin::cvImageToOfxImage(const CVImageWrapper & cvImg,
                                       const OfxRectI & renderWindow,
                                       OFX::Image* dstImg)
{
    void* pixelData = cvImg.getData();
    const OfxRectI & bounds = renderWindow;
    int rowBytes;
    int pixelComponentCount;
    
#if CV_MAJOR_VERSION < 3
    rowBytes = (int)img->widthStep;
    const IplImage* img = cvImg.getIplImage();
    assert( (renderWindow.x2 - renderWindow.x1) == img->width &&
            (renderWindow.y2 - renderWindow.y1) == img->height &&
            img->depth == IPL_DEPTH_8U &&
            (img->nChannels == 1 || img->nChannels == 3 || img->nChannels == 4) );
    pixelComponentCount = img->nChannels;
    
#else
    cv::Mat* img = cvImg.getCvMat();
    rowBytes = (int)img->step[0];
    pixelComponentCount = img->channels();
    assert((renderWindow.x2 - renderWindow.x1) == img->cols &&
           (renderWindow.y2 - renderWindow.y1) == img->rows &&
           img->depth() == CV_8U &&
           (pixelComponentCount == 1 || pixelComponentCount == 3 || pixelComponentCount == 4));
#endif
    
    OFX::PixelComponentEnum pixelComponents = ePixelComponentNone;
    
    switch (pixelComponentCount) {
        case 1:
            pixelComponents = ePixelComponentAlpha;
            break;
        case 3:
            pixelComponents = ePixelComponentRGB;
            break;
        case 4:
            pixelComponents = ePixelComponentRGBA;
            break;
        default:
            throwSuiteStatusException(kOfxStatErrImageFormat);
            
            return;
    }
    OFX::BitDepthEnum bitDepth = eBitDepthUByte;
    void* dstPixelData;
    OfxRectI dstBounds;
    OFX::PixelComponentEnum dstPixelComponents;
    OFX::BitDepthEnum dstBitDepth;
    int dstRowBytes;
    getImageData(dstImg, &dstPixelData, &dstBounds, &dstPixelComponents, &dstBitDepth, &dstRowBytes);
    int dstPixelComponentCount = dstImg->getPixelComponentCount();

    _srgbLut->from_byte_packed(pixelData, bounds, pixelComponents, pixelComponentCount, bitDepth, rowBytes,
                               renderWindow,
                               dstPixelData, dstBounds, dstPixelComponents, dstPixelComponentCount, dstBitDepth, dstRowBytes);
}

void
genericCVDescribe(const std::string & pluginName,
                  const std::string & pluginGrouping,
                  const std::string & pluginDescription,
                  bool supportsTiles,
                  bool supportsMultiResolution,
                  bool temporalClipAccess,
                  OFX::RenderSafetyEnum threadSafety,
                  OFX::ImageEffectDescriptor & desc)
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
    desc.setSupportsTiles(supportsTiles);
    desc.setTemporalClipAccess(temporalClipAccess);
    desc.setRenderTwiceAlways(false);
    desc.setSupportsMultipleClipPARs(false);
    desc.setRenderThreadSafety(threadSafety);
}

