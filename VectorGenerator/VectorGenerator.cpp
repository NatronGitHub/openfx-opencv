/*
   OFX VectorGenerator plugin.

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


   The skeleton for this source file is from:
   OFX Invert Example plugin, a plugin that illustrates the use of the OFX Support library.

   Copyright (C) 2007 The Open Effects Association Ltd
   Author Bruno Nicoletti bruno@thefoundry.co.uk

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name The Open Effects Association Ltd, nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   The Open Effects Association Ltd
   1 Wardour St
   London W1D 6PA
   England


 */


#include "GenericOpenCVPlugin.h"

#include "ofxsLut.h"

#define kPluginName "VectorGeneratorOFX"
#define kPluginGrouping "Time"
#define kPluginDescription ""
#define kPluginIdentifier "net.sf.openfx.VectorGenerator"
#define kPluginVersionMajor 1 // Incrementing this number means that you have broken backwards compatibility of the plug-in.
#define kPluginVersionMinor 0 // Increment this when you have fixed a bug or made it faster.

#define kSupportsTiles 0
#define kSupportsMultiResolution 1
#define kSupportsRenderScale 1
#define kRenderThreadSafety eRenderFullySafe

#define kParamRChannel "rChannel"
#define kParamRChannelLabel "R channel"
#define kParamRChannelHint "Selects which component of the motion vectors to set in the red channel of the output image"

#define kParamGChannel "gChannel"
#define kParamGChannelLabel "G channel"
#define kParamGChannelHint "Selects which component of the motion vectors to set in the green channel of the output image"

#define kParamBChannel "bChannel"
#define kParamBChannelLabel "B channel"
#define kParamBChannelHint "Selects which component of the motion vectors to set in the blue channel of the output image"

#define kParamAChannel "aChannel"
#define kParamAChannelLabel "A channel"
#define kParamAChannelHint "Selects which component of the motion vectors to set in the alpha channel of the output image"

#define kChannelNone "-"

#define kChannelBackwardU "backward.u"
#define kChannelBackwardV "backward.v"

#define kChannelForwardU "forward.u"
#define kChannelForwardV "forward.v"

#define kParamMethod "method"
#define kParamMethodLabel "Method"
#define kParamMethodHint ""

#define kParamAdvanced "advanced"
#define kParamAdvancedLabel "Advanced"


//Farneback
#define kParamLevels "levels"
#define kParamLevelsLabel "Levels"
#define kParamLevelsHint "Number of pyramid levels including initial image. If 1 that means no extra layer will be created and only the original images are used."

//Farneback && Dual TV L1
#define kParamIterations "iterations"
#define kParamIterationsLabel "Iterations"
#define kParamIterationsHint "Number of iterations the algorithm uses at each pyramid level"

//Farneback
#define kParamPixelNeighborhood "neighborhood"
#define kParamPixelNeighborhoodLabel "Neighborhood"
#define kParamPixelNeighborhoodHint "Size of the pixel neighborhood used to find the polynomial expansion in each pixel. Larger values mean that the image will " \
    "be approximated with smoother surfaces, yielding a more robust algorithm and more blurred motion field."

//Farneback
#define kParamSigma "sigma"
#define kParamSigmaLabel "Sigma"
#define kParamSigmaHint "Standard deviation of the Gaussian used to smooth derivatives used as a basis of the  polynomial expansion. For a Neighborhood of 5 " \
    "you can set Sigma to 1.1. For a Neighborhood of 7, a good value for sigma would be 1.5."

//Simple flow
#define kParamLayers "layers"
#define kParamLayersLabel "Layers"
#define kParamLayersHint "Recommendation for number of layers:\n" \
    " 4K         : 7 \n" \
    " 1080P      : 6 \n" \
    " 720P       : 5 \n"

//Simple flow
#define kParamBlockSize "blockSize"
#define kParamBlockSizeLabel "Block Size"
#define kParamBlockSizeHint "Size of pixels block through which we sum up when calculating cost function for each pixel"

//Simple flow
#define kParamMaxFlow "maxFlow"
#define kParamMaxFlowLabel "Max flow"
#define kParamMaxFlowHint "Maximum flow that we search at each level"

//Dual TV L1
#define kParamTau "tau"
#define kParamTauLabel "Tau"
#define kParamTauHint "Time step of the numerical scheme"

//Dual TV L1
#define kParamLambda "lambda"
#define kParamLambdaLabel "Lambda"
#define kParamLambdaHint "This determines the smoothness of the output. The smaller the parameter is, the smoother the solutions we obtain."

//Dual TV L1
#define kParamTheta "theta"
#define kParamThetaLabel "Theta"
#define kParamThetaHint "It serves as a link between the attachment and the regularization terms. It should have a small value in order to maintain both parts in " \
    "correspondance."

//Dual TV L1
#define kParamNScales "nScales"
#define kParamNScalesLabel "N. Scales"
#define kParamNScalesHint "Number of scales used to create the pyramid image"

//Dual TV L1
#define kParamWarps "warps"
#define kParamWarpsLabel "Warps"
#define kParamWarpsHint "Number of warpings per scale. This affects the stability of the method at the expense of running time."

//Dual TV L1
#define kParamEpsilon "epsilon"
#define kParamEpsilonLabel "Epsilon"
#define kParamEpsilonHint "Stopping criterion theshold which is a trade-off between accuracy and running time. A small value will yield more accurate solutions."


enum OpticalFlowMethodEnum
{
    eOpticalFlowFarneback = 0,
    eOpticalFlowSimpleFlow,
    eOpticalFlowDualTVL1
};

using namespace OFX;
using namespace cv;

class VectorGeneratorPlugin
    : public GenericOpenCVPlugin
{
public:
    /** @brief ctor */
    VectorGeneratorPlugin(OfxImageEffectHandle handle)
        : GenericOpenCVPlugin(handle)
          , _rChannel(0)
          , _gChannel(0)
          , _bChannel(0)
          , _aChannel(0)
          , _method(0)
    {
        _rChannel = fetchChoiceParam(kParamRChannel);
        _gChannel = fetchChoiceParam(kParamGChannel);
        _bChannel = fetchChoiceParam(kParamBChannel);
        _aChannel = fetchChoiceParam(kParamAChannel);
        _method = fetchChoiceParam(kParamMethod);
        assert(_rChannel && _gChannel && _bChannel && _aChannel);
    }

private:
    /* Override the render */
    virtual void render(const OFX::RenderArguments &args) OVERRIDE FINAL;

    /**
     * @brief Compute motion vectors from 'ref' to 'other' on the given renderWindow and set them in the given channel indexes
     * of the dst Image.
     * @param channelIndex[in] A vector for each coordinate (x,y) of components of the output image to fill.
     * Each componentn will be a value between [0 , 3] targeting one of the R/G/B/A channels of the dst image.
     **/
    void calcOpticalFlow(OFX::Image* ref,OFX::Image* other,const OfxRectI & renderWindow,std::vector<int> channelIndex[2],
                         OpticalFlowMethodEnum method,OFX::Image* dst);

private:

    ChoiceParam* _rChannel;
    ChoiceParam* _gChannel;
    ChoiceParam* _bChannel;
    ChoiceParam* _aChannel;
    ChoiceParam* _method;
};

static IplImage*
createLuminance8bitImage(OFX::Image* img,
                         const OfxRectI & renderWindow)
{
    IplImage* cvImg = cvCreateImage(cvSize(renderWindow.x2 - renderWindow.x1, renderWindow.y2 - renderWindow.y1), IPL_DEPTH_8U, 1);
    int srcRowElements = img->getRowBytes() / sizeof(float);
    int dstRowElements = cvImg->widthStep;
    const float* src_pixels = (const float*)img->getPixelAddress(renderWindow.x1, renderWindow.y1);
    char* dst_pixels = cvImg->imageData;
    int nComps;

    switch ( img->getPixelComponents() ) {
    case OFX::ePixelComponentAlpha:
        nComps = 1;
        break;
    case OFX::ePixelComponentRGB:
        nComps = 3;
        break;
    case OFX::ePixelComponentRGBA:
        nComps = 4;
        break;
    default:
        assert(false);
        break;
    }

    for (int y = 0; y < cvImg->height; ++y, src_pixels += (srcRowElements) - cvImg->width * nComps, dst_pixels += (dstRowElements) - cvImg->width) {
        for (int x = 0; x < cvImg->width; ++x,src_pixels += nComps, ++dst_pixels) {
            if (nComps == 1) {
                *dst_pixels = OFX::Color::floatToInt<256>(*src_pixels);
            } else {
                double r = src_pixels[0];
                double g = src_pixels[1];
                double b = src_pixels[2];
                *dst_pixels = OFX::Color::floatToInt<256>(0.299 * r + 0.587 * g + 0.114 * b);
            }
        }
    }

    return cvImg;
}

void
VectorGeneratorPlugin::calcOpticalFlow(OFX::Image* ref,
                                       OFX::Image* other,
                                       const OfxRectI & renderWindow,
                                       std::vector<int> channelIndex[2],
                                       OpticalFlowMethodEnum method,
                                       OFX::Image* dst)
{
    assert(dst->getPixelComponents() == OFX::ePixelComponentRGBA);


    cv::Mat flow(renderWindow.x2 - renderWindow.x1,renderWindow.y2 - renderWindow.y1,CV_32FC2);

    if (method == eOpticalFlowFarneback) {
        IplImage* srcRefCVImg = createLuminance8bitImage(ref, renderWindow);
        IplImage* srcNextCVImg = createLuminance8bitImage(other, renderWindow);
        cv::Mat srcRefMatImg(srcRefCVImg, false /*copyData*/);
        cv::Mat srcNextMatImg(srcNextCVImg, false /*copyData*/);


        assert( srcRefMatImg.rows == (renderWindow.y2 - renderWindow.y1) && srcRefMatImg.cols == (renderWindow.x2 - renderWindow.x1) );
        assert( srcNextMatImg.rows == (renderWindow.y2 - renderWindow.y1) && srcNextMatImg.cols == (renderWindow.x2 - renderWindow.x1) );

        int nbLevels = 3;
        double pyrScale = 0.5;
        int nbIterations = 15;
        int polyN = 5;
        int polySigma = 1.1;
        int winSize = 3;

        calcOpticalFlowFarneback(srcRefMatImg, srcNextMatImg, flow, pyrScale, nbLevels, winSize, nbIterations, polyN, polySigma, 0);

        cvReleaseImage(&srcRefCVImg);
        cvReleaseImage(&srcNextCVImg);
    } else if (method == eOpticalFlowSimpleFlow) {
        CVImageWrapper srcRef,srcNext;
        fetchCVImage(ref, renderWindow, true, &srcRef);
        fetchCVImage(other, renderWindow, true, &srcNext);

        cv::Mat srcRefMatImg(srcRef.getIplImage(), false /*copyData*/);
        cv::Mat srcNextMatImg(srcNext.getIplImage(), false /*copyData*/);
        int nbLayers = 3;
        int avgBlockSize = 2;
        int maxFlow = 4;

        calcOpticalFlowSF(srcRefMatImg, srcNextMatImg, flow, nbLayers, avgBlockSize, maxFlow);
    } else if (method == eOpticalFlowDualTVL1) {
        IplImage* srcRefCVImg = createLuminance8bitImage(ref, renderWindow);
        IplImage* srcNextCVImg = createLuminance8bitImage(other, renderWindow);
        cv::Mat srcRefMatImg(srcRefCVImg, false /*copyData*/);
        cv::Mat srcNextMatImg(srcNextCVImg, false /*copyData*/);


        Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();
        tvl1->set("tau", 0.25);
        tvl1->set("lambda", 0.15);
        tvl1->set("theta", 0.3);
        tvl1->set("nscales", 5);
        tvl1->set("warps", 5);
        tvl1->set("epsilon", 0.01);
        tvl1->set("iterations", 300);
        tvl1->calc(srcRefMatImg,srcNextMatImg,flow);


        cvReleaseImage(&srcRefCVImg);
        cvReleaseImage(&srcNextCVImg);
    }

    IplImage flowImg = (IplImage)flow;

    assert( flowImg.imageSize >= (flow.rows * flow.cols * sizeof(float) * 2) );

    int dstElemCount = dst->getRowBytes() / sizeof(float);
    int flowElemCount = flowImg.widthStep / sizeof(float);
    float* dst_pixels = (float*)dst->getPixelAddress(renderWindow.x1, renderWindow.y1);
    const float* src_pixels = reinterpret_cast<const float*>(flowImg.imageData);
    assert(dst_pixels && src_pixels);


    for (int y = 0; y < flowImg.height; ++y) {
        for (int x = 0; x < flowImg.width; ++x) {
            for (int coord = 0; coord < 2; ++coord) {
                for (int k = 0; k < channelIndex[coord].size(); ++k) {
                    assert(channelIndex[coord][k] >= 0 && channelIndex[coord][k] <= 3);
                    dst_pixels[x * 4 + channelIndex[coord][k]] = src_pixels[x * 2 + coord];
                }
            }
        }
        dst_pixels += dstElemCount;
        src_pixels += flowElemCount;
    }
} // calcOpticalFlow

// the overridden render function
void
VectorGeneratorPlugin::render(const OFX::RenderArguments &args)
{
    std::auto_ptr<OFX::Image> dst( dstClip_->fetchImage(args.time) );

    if ( !dst.get() ) {
        OFX::throwSuiteStatusException(kOfxStatFailed);
    }
    if ( (dst->getRenderScale().x != args.renderScale.x) ||
         ( dst->getRenderScale().y != args.renderScale.y) ||
         ( dst->getField() != args.fieldToRender) ) {
        setPersistentMessage(OFX::Message::eMessageError, "", "OFX Host gave image with wrong scale or field properties");
        OFX::throwSuiteStatusException(kOfxStatFailed);
    }

    //Original source image at current time
    std::auto_ptr<OFX::Image> srcRef( srcClip_->fetchImage(args.time) );

    if ( !srcRef.get() ) {
        OFX::throwSuiteStatusException(kOfxStatFailed);
    }


    int rChannel,bChannel,gChannel,aChannel;
    _rChannel->getValue(rChannel);
    _gChannel->getValue(gChannel);
    _bChannel->getValue(bChannel);
    _aChannel->getValue(aChannel);

    bool backwardNeeded = rChannel == 1 || rChannel == 2 || gChannel == 1 || gChannel == 2 || bChannel == 1 || bChannel == 2 || aChannel == 1 || aChannel == 2;
    bool forwardNeeded = rChannel == 3 || rChannel == 4 || gChannel == 3 || gChannel == 4 || bChannel == 3 || bChannel == 4 || aChannel == 3 || aChannel == 4;
    int method_i;
    _method->getValue(method_i);
    OpticalFlowMethodEnum method = (OpticalFlowMethodEnum)method_i;

    if (backwardNeeded) {
        //Other image for "backward" optical flow computation
        std::auto_ptr<OFX::Image> srcPrev( srcClip_->fetchImage(args.time - 1) );
        if ( !srcPrev.get() ) {
            OFX::throwSuiteStatusException(kOfxStatFailed);
        }

        std::vector<int> channelIndex[2];
        channelIndex[0] = std::vector<int>();
        channelIndex[1] = std::vector<int>();
        if (rChannel == 1) {
            channelIndex[0].push_back(0);
        }
        if (gChannel == 1) {
            channelIndex[0].push_back(1);
        }
        if (bChannel == 1) {
            channelIndex[0].push_back(2);
        }
        if (aChannel == 1) {
            channelIndex[0].push_back(3);
        }

        if (rChannel == 2) {
            channelIndex[1].push_back(0);
        }
        if (gChannel == 2) {
            channelIndex[1].push_back(1);
        }
        if (bChannel == 2) {
            channelIndex[1].push_back(2);
        }
        if (aChannel == 2) {
            channelIndex[1].push_back(3);
        }

        calcOpticalFlow( srcRef.get(), srcPrev.get(),  args.renderWindow, channelIndex, method, dst.get() );
    }

    if (forwardNeeded) {
        //Other image for "backward" optical flow computation
        std::auto_ptr<OFX::Image> srcNext( srcClip_->fetchImage(args.time + 1) );
        if ( !srcNext.get() ) {
            OFX::throwSuiteStatusException(kOfxStatFailed);
        }


        std::vector<int> channelIndex[2];
        channelIndex[0] = std::vector<int>();
        channelIndex[1] = std::vector<int>();
        if (rChannel == 3) {
            channelIndex[0].push_back(0);
        }
        if (gChannel == 3) {
            channelIndex[0].push_back(1);
        }
        if (bChannel == 3) {
            channelIndex[0].push_back(2);
        }
        if (aChannel == 3) {
            channelIndex[0].push_back(3);
        }

        if (rChannel == 4) {
            channelIndex[1].push_back(0);
        }
        if (gChannel == 4) {
            channelIndex[1].push_back(1);
        }
        if (bChannel == 4) {
            channelIndex[1].push_back(2);
        }
        if (aChannel == 4) {
            channelIndex[1].push_back(3);
        }

        calcOpticalFlow( srcRef.get(), srcNext.get(), args.renderWindow, channelIndex, method, dst.get() );
    }
} // render

mDeclarePluginFactory(VectorGeneratorPluginFactory, {}, {}
                      );

using namespace OFX;
void
VectorGeneratorPluginFactory::describe(OFX::ImageEffectDescriptor &desc)
{
    genericCVDescribe(kPluginName, kPluginGrouping, kPluginDescription, kSupportsTiles, kSupportsMultiResolution, true, kRenderThreadSafety, desc);
}

void
VectorGeneratorPluginFactory::describeInContext(OFX::ImageEffectDescriptor &desc,
                                                OFX::ContextEnum context)
{
    // Source clip only in the filter context
    // create the mandated source clip
    ClipDescriptor *srcClip = desc.defineClip(kOfxImageEffectSimpleSourceClipName);

    //srcClip->addSupportedComponent(ePixelComponentRGBA);
    srcClip->addSupportedComponent(ePixelComponentRGB);
    srcClip->setTemporalClipAccess(false);
    srcClip->setSupportsTiles(kSupportsTiles);
    srcClip->setIsMask(false);

    // create the mandated output clip
    ClipDescriptor *dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
    dstClip->addSupportedComponent(ePixelComponentRGBA);
    dstClip->setSupportsTiles(kSupportsTiles);


    // make some pages and to things in
    PageParamDescriptor *page = desc.definePageParam("Controls");

    {
        ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamRChannel);
        param->setLabels(kParamRChannelLabel, kParamRChannelLabel, kParamRChannelLabel);
        param->setHint(kParamRChannelHint);
        param->appendOption(kChannelNone);
        param->appendOption(kChannelBackwardU);
        param->appendOption(kChannelBackwardV);
        param->appendOption(kChannelForwardU);
        param->appendOption(kChannelForwardV);
        param->setDefault(1);
        param->setAnimates(true);
        page->addChild(*param);
    }

    {
        ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamGChannel);
        param->setLabels(kParamGChannelLabel, kParamGChannelLabel, kParamGChannelLabel);
        param->setHint(kParamGChannelHint);
        param->appendOption(kChannelNone);
        param->appendOption(kChannelBackwardU);
        param->appendOption(kChannelBackwardV);
        param->appendOption(kChannelForwardU);
        param->appendOption(kChannelForwardV);
        param->setDefault(2);
        param->setAnimates(true);
        page->addChild(*param);
    }
    {
        ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamBChannel);
        param->setLabels(kParamBChannelLabel, kParamBChannelLabel, kParamBChannelLabel);
        param->setHint(kParamBChannelHint);
        param->appendOption(kChannelNone);
        param->appendOption(kChannelBackwardU);
        param->appendOption(kChannelBackwardV);
        param->appendOption(kChannelForwardU);
        param->appendOption(kChannelForwardV);
        param->setDefault(3);
        param->setAnimates(true);
        page->addChild(*param);
    }
    {
        ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamAChannel);
        param->setLabels(kParamAChannelLabel, kParamAChannelLabel, kParamAChannelLabel);
        param->setHint(kParamAChannelHint);
        param->appendOption(kChannelNone);
        param->appendOption(kChannelBackwardU);
        param->appendOption(kChannelBackwardV);
        param->appendOption(kChannelForwardU);
        param->appendOption(kChannelForwardV);
        param->setDefault(4);
        param->setAnimates(true);
        page->addChild(*param);
    }

    {
        ChoiceParamDescriptor *param = desc.defineChoiceParam(kParamMethod);
        param->setLabels(kParamMethodLabel, kParamMethodLabel, kParamMethodLabel);
        param->setHint(kParamMethodHint);
        assert(param->getNOptions() == eOpticalFlowFarneback);
        param->appendOption("Farneback");
        assert(param->getNOptions() == eOpticalFlowSimpleFlow);
        param->appendOption("Simple flow");
        assert(param->getNOptions() == eOpticalFlowDualTVL1);
        param->appendOption("Dual TV L1");
        param->setDefault(0);
        param->setAnimates(false);
        page->addChild(*param);
    }

    //Farneback
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamLevels);
        param->setLabels(kParamLevelsLabel, kParamLevelsLabel, kParamLevelsLabel);
        param->setHint(kParamLevelsHint);
        param->setDefault(3);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Farneback @ Dual TV L1
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamIterations);
        param->setLabels(kParamIterationsLabel, kParamIterationsLabel, kParamIterationsLabel);
        param->setHint(kParamIterationsHint);
        param->setDefault(15);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Farneback
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamPixelNeighborhood);
        param->setLabels(kParamPixelNeighborhoodLabel, kParamPixelNeighborhoodLabel, kParamPixelNeighborhoodLabel);
        param->setHint(kParamPixelNeighborhoodHint);
        param->setDefault(5);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Farneback
    {
        DoubleParamDescriptor *param = desc.defineDoubleParam(kParamSigma);
        param->setLabels(kParamSigmaLabel, kParamSigmaLabel, kParamSigmaLabel);
        param->setHint(kParamSigmaHint);
        param->setDefault(1.1);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Simple flow
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamLayers);
        param->setLabels(kParamLayersLabel, kParamLayersLabel, kParamLayersLabel);
        param->setHint(kParamLayersHint);
        param->setDefault(3);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Simple flow
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamLayers);
        param->setLabels(kParamLayersLabel, kParamLayersLabel, kParamLayersLabel);
        param->setHint(kParamLayersHint);
        param->setDefault(3);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Simple flow
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamBlockSize);
        param->setLabels(kParamBlockSizeLabel, kParamBlockSizeLabel, kParamBlockSizeLabel);
        param->setHint(kParamBlockSizeHint);
        param->setDefault(2);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Simple flow
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamMaxFlow);
        param->setLabels(kParamMaxFlowLabel, kParamMaxFlowLabel, kParamMaxFlowLabel);
        param->setHint(kParamMaxFlowHint);
        param->setDefault(4);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Dual TV L1
    {
        DoubleParamDescriptor *param = desc.defineDoubleParam(kParamTau);
        param->setLabels(kParamTauLabel, kParamTauLabel, kParamTauLabel);
        param->setHint(kParamTauHint);
        param->setDefault(0.25);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Dual TV L1
    {
        DoubleParamDescriptor *param = desc.defineDoubleParam(kParamLambda);
        param->setLabels(kParamLambdaLabel, kParamLambdaLabel, kParamLambdaLabel);
        param->setHint(kParamLambdaHint);
        param->setDefault(0.15);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Dual TV L1
    {
        DoubleParamDescriptor *param = desc.defineDoubleParam(kParamTheta);
        param->setLabels(kParamThetaLabel, kParamThetaLabel, kParamThetaLabel);
        param->setHint(kParamThetaHint);
        param->setDefault(0.3);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Dual TV L1
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamNScales);
        param->setLabels(kParamNScalesLabel, kParamNScalesLabel, kParamNScalesLabel);
        param->setHint(kParamNScalesHint);
        param->setDefault(5);
        param->setAnimates(true);
        page->addChild(*param);
    }

    //Dual TV L1
    {
        IntParamDescriptor *param = desc.defineIntParam(kParamWarps);
        param->setLabels(kParamWarpsLabel,kParamWarpsLabel, kParamWarpsLabel);
        param->setHint(kParamWarpsHint);
        param->setDefault(5);
        param->setAnimates(true);
        page->addChild(*param);
    }
    //Dual TV L1
    {
        DoubleParamDescriptor *param = desc.defineDoubleParam(kParamEpsilon);
        param->setLabels(kParamEpsilonLabel, kParamEpsilonLabel, kParamEpsilonLabel);
        param->setHint(kParamEpsilonHint);
        param->setDefault(0.01);
        param->setAnimates(true);
        page->addChild(*param);
    }
} // describeInContext

OFX::ImageEffect*
VectorGeneratorPluginFactory::createInstance(OfxImageEffectHandle handle,
                                             OFX::ContextEnum /*context*/)
{
    return new VectorGeneratorPlugin(handle);
}

void
getVectorGeneratorPluginID(OFX::PluginFactoryArray &ids)
{
    static VectorGeneratorPluginFactory p(kPluginIdentifier, kPluginVersionMajor, kPluginVersionMinor);

    ids.push_back(&p);
}

