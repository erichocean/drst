/* Copyright (c) 2013, Intel Corporation
*
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice, 
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice, 
*   this list of conditions and the following disclaimer in the documentation 
*   and/or other materials provided with the distribution.
* - Neither the name of Intel Corporation nor the names of its contributors 
*   may be used to endorse or promote products derived from this software 
*   without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef __EMBREE_PATH_TRACE_STREAM_INTEGRATOR_H__
#define __EMBREE_PATH_TRACE_STREAM_INTEGRATOR_H__

#include "integrators/integrator.h"
#include "renderers/renderer.h"
#include "image/image.h"
#include "embree/common/alignedallocator.h"

namespace embree {
  
  class PathTraceStream;
  
  class PathTraceStreamIntegrator : public Integrator {
    friend class PathTraceStream;
    
  private:
    size_t maxDepth;
    float minContribution;
    float epsilon;
    Ref<Image> backplate;
    
    int lightSampleID;
    int firstScatterSampleID;
    int firstScatterTypeSampleID;
    std::vector<int, AlignedAllocator<int, 64> > precomputedLightSampleID;

    unsigned maxSize;
    
  public:
    PathTraceStreamIntegrator(const Parms& parms, unsigned maxSize);

    void requestSamples(Ref<SamplerFactory>& samplerFactory, const Ref<BackendScene>& scene);

    Color Li(size_t thread, Ray& ray, const Ref<BackendScene>& scene, IntegratorState& state);
    
    TileIntegrator* createTileIntegrator();
  };
  
}

#endif
