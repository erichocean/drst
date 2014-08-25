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

#include "pathtracestreamintegrator.h"
#include "pathtracestream.h"

using namespace embree;

PathTraceStreamIntegrator::PathTraceStreamIntegrator(const Parms& parms, unsigned maxSize)
: lightSampleID(-1), firstScatterSampleID(-1), firstScatterTypeSampleID(-1), maxSize(maxSize)
{
  maxDepth        = parms.getInt  ("maxDepth"       ,10    );
  minContribution = parms.getFloat("minContribution",0.01f );
  epsilon         = parms.getFloat("epsilon"        ,8.0f)*float(ulp);
  backplate       = parms.getImage("backplate");

  std::cout << "using streaming traversal" << std::endl;
}

void PathTraceStreamIntegrator::requestSamples(Ref<SamplerFactory>& samplerFactory, const Ref<BackendScene>& scene)
{
  precomputedLightSampleID.resize(scene->allLights.size());
  
  lightSampleID = samplerFactory->request2D();
  for (size_t i=0; i<scene->allLights.size(); i++) {
    precomputedLightSampleID[i] = -1;
    if (scene->allLights[i]->precompute())
      precomputedLightSampleID[i] = samplerFactory->requestLightSample(lightSampleID, scene->allLights[i]);
  }
  firstScatterSampleID = samplerFactory->request2D((int)maxDepth);
  firstScatterTypeSampleID = samplerFactory->request1D((int)maxDepth);
}

/*void PathTraceStreamIntegrator::Li(size_t thread, unsigned sampleCount, const Ref<Camera>& camera, const Ref<BackendScene>& scene, IntegratorStreamState& state) {
  PathTraceStream* stream = threadStates[thread];
  RayStream<false>& rays = stream->getRayStream();

  const unsigned char* __restrict pixelSampleSets = state.pixelSampleSets;
  const Vec2f* __restrict pixelLocations = state.pixelLocations;
  const PrecomputedSample* const __restrict * __restrict sampleSets = state.sampleSets;
  unsigned samplesPerPixelLog2 = state.samplesPerPixelLog2;
  unsigned samplesPerPixelMask = (1 << samplesPerPixelLog2)-1;

  for (unsigned i = 0; i < sampleCount; ++i) {
    unsigned pixel = i >> samplesPerPixelLog2;
    unsigned sampleSet = pixelSampleSets[pixel];
    
    const PrecomputedSample* __restrict precomputedSample = &sampleSets[sampleSet][i & samplesPerPixelMask];

    Ray primary; camera->ray(precomputedSample->pixel + pixelLocations[pixel], precomputedSample->getLens(), primary);
    primary.time = precomputedSample->getTime();
    rays.nextRay()->set(primary.org, primary.dir, primary.tnear, primary.tfar);
    rays.nextRayExtra()->set(primary.org, primary.dir, primary.time);
    rays.increment();
  }
  
  stream->process(scene.ptr, state, (unsigned)thread);
}*/

Color PathTraceStreamIntegrator::Li(size_t thread, Ray& ray, const Ref<BackendScene>& scene, IntegratorState& state) {
  return zero;
}

TileIntegrator* PathTraceStreamIntegrator::createTileIntegrator() {
  return new PathTraceStream(*this, maxSize);
}
