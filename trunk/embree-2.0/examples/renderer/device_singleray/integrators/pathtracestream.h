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

#ifndef __EMBREE_PATH_TRACE_STREAM_H__
#define __EMBREE_PATH_TRACE_STREAM_H__

#include "raystream.h"
#include "integrator.h"
#include "integrators/integrator.h"
#include "renderers/renderer.h"
#include "pathtracestreamintegrator.h"

namespace embree {
  
  class __align(64) PathTraceStream : public TileIntegrator {
    ALIGNED_CLASS;
    
  private:
    struct __align(32) LightPath {
      union {
        float throughput[3];
        unsigned flags[3];
      } throughputAndFlags;
      float probability;
      float mediumTransmission[3];
      float mediumEta;
      
      __forceinline void set(const Color& throughput, const Medium& medium, unsigned ignoreVisibleLights = 0, unsigned unbend = 1, float probability = 1.0f) {
        __m128 tro = _mm_insert_ps(throughput, _mm_set1_ps(probability), 3 << 4);
        __m128 tra = _mm_insert_ps(medium.transmission, _mm_set1_ps(medium.eta), 3 << 4);
        
        _mm_store_ps(this->throughputAndFlags.throughput, tro);
        _mm_store_ps(this->mediumTransmission, tra);

        if (ignoreVisibleLights)
          throughputAndFlags.flags[0] |= 0x80000000;
        if (unbend)
          throughputAndFlags.flags[1] |= 0x80000000;
      }
    };
    
    struct __align(16) ShadowPath {
      float weight[3];
      unsigned pixelIndex;
      
      __forceinline void set(const Color& weight, unsigned pixelIndex) {
        __m128 w = _mm_castsi128_ps(_mm_insert_epi32(_mm_castps_si128(weight), pixelIndex, 3));
        _mm_store_ps(this->weight, w);
      }
    };
    
    PathTraceStreamIntegrator& integrator;
    
    unsigned maxSize;

    RayStream<false> rays;
    uint16* sampleIndices;
    LightPath* lightPaths;
    
    RayStream<true> shadowRays;
    ShadowPath* shadowPaths;
    unsigned maxShadowRays;

    Tile* tile;
    BackendScene* scene;
    IntegratorStreamState* state;
    unsigned thread;
    unsigned depth;

    unsigned currentRayCount;
    
  public:
    PathTraceStream(PathTraceStreamIntegrator& integrator, unsigned maxSize) : integrator(integrator), maxSize(maxSize), rays(maxSize), shadowRays(maxSize*8), maxShadowRays(maxSize*8) {
      sampleIndices = static_cast<uint16*>(alignedMalloc(sizeof(uint16)*maxSize, 128));
      lightPaths = static_cast<LightPath*>(alignedMalloc(sizeof(LightPath)*maxSize));
      shadowPaths = static_cast<ShadowPath*>(alignedMalloc(sizeof(ShadowPath)*maxShadowRays));
    }

    RayStream<false>& getRayStream() {
      return rays;
    }
    
    virtual void dispatch(Tile* tile, BackendScene* scene, IntegratorStreamState& state, unsigned thread) {
      this->tile = tile;
      this->scene = scene;
      this->state = &state;
      this->thread = thread;
      this->depth = depth;

      AVX_ZERO_UPPER();
      
      setupInitialIndices();
      setupInitialLightPaths();
      
      if (maxShadowRays < maxSize * scene->allLights.size()) {
        maxShadowRays = maxSize * scene->allLights.size();
        shadowRays.reset(maxShadowRays);
        
        alignedFree(shadowPaths);
        shadowPaths = static_cast<ShadowPath*>(alignedMalloc(sizeof(ShadowPath)*maxShadowRays));
      }
      
      depth = 0;
      currentRayCount = rays.getSize();
      rays.dispatch(scene, thread);

      AVX_ZERO_UPPER();
    }

    virtual bool process() {
      dispatch();
      ++depth;

      if (!rays.getSize()) {
        AVX_ZERO_UPPER();
        return false;
      }
      
      currentRayCount = rays.getSize();
      rays.dispatch(scene, thread);
      return true;
    }

    ~PathTraceStream() {
      alignedFree(sampleIndices);
      alignedFree(lightPaths);
      alignedFree(shadowPaths);
    }
    
  private:
    __forceinline void setupInitialLightPaths() {
      LightPath defaultLightPath;
      defaultLightPath.set(one, Medium::Vacuum());
      
      assert(sizeof(LightPath) == 32);
      __m256 lp = _mm256_load_ps(reinterpret_cast<const float*>(&defaultLightPath));
      
      float* start = reinterpret_cast<float*>(lightPaths);
      float* end = start + rays.getSize()*8;
      
      for (float* paths = start; paths < end; paths += 8*4) {
        _mm256_store_ps(paths + 8*0, lp);
        _mm256_store_ps(paths + 8*1, lp);
        _mm256_store_ps(paths + 8*2, lp);
        _mm256_store_ps(paths + 8*3, lp);
      }
    }
    
    void setupInitialIndices() {
      __m256i a0 = _mm256_setr_epi16( 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15);
      __m256i a1 = _mm256_setr_epi16(16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31);
      __m256i a2 = _mm256_setr_epi16(32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47);
      __m256i a3 = _mm256_setr_epi16(48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63);
      __m256i r = _mm256_set1_epi16(64);
      
      __m256i* start = reinterpret_cast<__m256i*>(sampleIndices);
      __m256i* end = start + ((rays.getSize() + 15) >> 4);
      
      for (__m256i* indices = start; indices < end; indices += 4) {
        _mm256_store_si256(indices + 0, a0);
        _mm256_store_si256(indices + 1, a1);
        _mm256_store_si256(indices + 2, a2);
        _mm256_store_si256(indices + 3, a3);
        
        a0 = _mm256_add_epi16(a0, r);
        a1 = _mm256_add_epi16(a1, r);
        a2 = _mm256_add_epi16(a2, r);
        a3 = _mm256_add_epi16(a3, r);
      }
    }
    
    void dispatch() {
      const StreamRay* __restrict rays = this->rays.getRays();
      const StreamRayExtra* __restrict rayExtras = this->rays.getRayExtras();
      const StreamHit* __restrict hits = this->rays.getHits();
      unsigned count = currentRayCount;

      IntegratorStreamState& streamState = *state;

      Color* __restrict pixelOutput = streamState.pixelOutput;
      const unsigned char* __restrict pixelSampleSets = streamState.pixelSampleSets;
      const Vec2f* __restrict pixelLocations = streamState.pixelLocations;
      const PrecomputedSample* const __restrict * __restrict sampleSets = streamState.sampleSets;
      unsigned samplesPerPixelLog2 = streamState.samplesPerPixelLog2;
      unsigned samplesPerPixelMask = (1 << samplesPerPixelLog2)-1;
      
      streamState.numRays += count;

      const Ref<Light>* lights = scene->allLights.data();
      size_t lightCount = scene->allLights.size();

/*#pragma warning(push)
#pragma warning(disable : 2308)
      _mm_prefetch(reinterpret_cast<const char*>(sampleSets), _MM_HINT_T0);
      _mm_prefetch(reinterpret_cast<const char*>(sampleSets)+64, _MM_HINT_T0);
      _mm_prefetch(reinterpret_cast<const char*>(sampleSets)+128, _MM_HINT_T0);
      _mm_prefetch(reinterpret_cast<const char*>(sampleSets)+192, _MM_HINT_T0);
#pragma warning(pop)*/

      int scatterSampleID = integrator.firstScatterSampleID + (int)depth;
      int scatterTypeSampleID = integrator.firstScatterTypeSampleID + (int)depth;
      unsigned lightSampleID = integrator.lightSampleID;

      float epsilon = integrator.epsilon;
      float minContribution = integrator.minContribution;

      unsigned maxDepth = (unsigned)integrator.maxDepth;

      for (unsigned i = 0; i < count; ++i) {
        unsigned sample = sampleIndices[i];

        const StreamRay& ray = rays[i];
        const StreamRayExtra& rayExtra = rayExtras[i];
        const StreamHit& hit = hits[i];

        DifferentialGeometry dg;

        unsigned pixelIndex = sample >> samplesPerPixelLog2;

#if 0
        pixelOutput[pixelIndex] = Color(hit.id1*(1.0f/8.0f), 0.0f, 0.0f);
        continue;
#endif

        scene->postIntersect(ray, rayExtra, hit, dg);

        unsigned sampleSet = pixelSampleSets[pixelIndex];
        //_mm_prefetch(reinterpret_cast<const char*>(pixelOutput + pixelIndex), _MM_HINT_T0);

        const PrecomputedSample* __restrict precomputedSample = &sampleSets[sampleSet][sample & samplesPerPixelMask];
        const LightPath& lightPath = lightPaths[i];

        BRDFType directLightingBRDFTypes = (BRDFType)(DIFFUSE);
        BRDFType giBRDFTypes = (BRDFType)(ALL);

        _mm_prefetch(reinterpret_cast<const char*>(precomputedSample), _MM_HINT_T0);
        Color weight = _mm_load_ps(lightPath.throughputAndFlags.throughput);

        /*! Environment shading when nothing hit. */
        if (!hit)
        {
          Vec2f pixel = precomputedSample->pixel + pixelLocations[pixelIndex];
          Vector3f rd(rayExtra.dir());

          unsigned flags = _mm_movemask_ps(weight);
          weight = abs(weight);

          unsigned ignoreVisibleLights = flags & 1;
          unsigned unbend = flags & 2;

          const Vector3f wo = -rd;

          Color L;
          if (integrator.backplate && unbend) {
            const int x = clamp(int(pixel.x * (int)integrator.backplate->width ), 0, int(integrator.backplate->width )-1);
            const int y = clamp(int(pixel.y * (int)integrator.backplate->height), 0, int(integrator.backplate->height)-1);
            L = integrator.backplate->get(x, y);
          }
          else {
            L = Color(zero);
            if (!ignoreVisibleLights)
              for (size_t i=0; i<scene->envLights.size(); i++)
                L += scene->envLights[i]->Le(wo);
          }

          pixelOutput[pixelIndex] = Color(madd((__m128)L, (__m128)weight, (__m128)pixelOutput[pixelIndex]));
          continue;
        }

        Medium lastMedium(_mm_load_ps(lightPath.mediumTransmission), lightPath.mediumEta);
        Vector3f rd(rayExtra.dir());

        unsigned flags = _mm_movemask_ps(weight);
        weight = abs(weight);

        /*! face forward normals */
        bool backfacing = false;
        if (dot(dg.Ng, rd) > 0) {
          backfacing = true; dg.Ng = -dg.Ng; dg.Ns = -dg.Ns;
        }

        unsigned ignoreVisibleLights = flags & 1;
        unsigned unbend = flags & 2;

        /*! Shade surface. */
        CompositedBRDF brdfs;
        if (dg.material) dg.material->shade(lastMedium, dg, brdfs);
      
        /*! Add light emitted by hit area light source. */
        if (!ignoreVisibleLights && dg.light && !backfacing)
          pixelOutput[pixelIndex] = Color(madd((__m128)dg.light->Le(dg,-rd), (__m128)weight, (__m128)pixelOutput[pixelIndex]));

        const float* __restrict samples1D = precomputedSample->samples1D;
        const Vec2f* __restrict samples2D = precomputedSample->samples2D;

        //_mm_prefetch(reinterpret_cast<const char*>(this->rays.nextRay()), _MM_HINT_T0);
        //_mm_prefetch(reinterpret_cast<const char*>(this->rays.nextRayExtra()), _MM_HINT_T0);
        //_mm_prefetch(reinterpret_cast<const char*>(&lightPaths[this->rays.getSize()]), _MM_HINT_T0);

        /*! Global illumination. Pick one BRDF component and sample it. */
        if (depth+1 < maxDepth)
        {
          const Vec2f& s = samples2D[scatterSampleID];
          float ss = samples1D[scatterTypeSampleID];

          /*! sample brdf */
          Sample3f wi; BRDFType type;
          Color c = brdfs.sample(-rd, dg, wi, type, s, ss, giBRDFTypes);
        
          /*! Continue only if we hit something valid. */
          if (c != Color(zero) && wi.pdf > 0.0f)
          {
            /*! Compute  simple volumetric effect. */
            const Color& transmission = lastMedium.transmission;
            if (transmission != Color(one)) c *= pow(transmission, ray.tfar);
          
            /*! Tracking medium if we hit a medium interface. */
            Medium nextMedium = lastMedium;
            if (type & TRANSMISSION) nextMedium = dg.material->nextMedium(lastMedium);
          
            /*! Continue the path. */
            Color scatteredWeight = weight*c;
            float probability = lightPath.probability;
          
            this->rays.nextRay()->set(dg.P, wi, dg.error*epsilon, inf);
            this->rays.nextRayExtra()->set(dg.P, wi, rayExtra.time);
            unsigned index = this->rays.getSize();
            sampleIndices[index] = sample;
            lightPaths[index].set(scatteredWeight * rcp(wi.pdf), nextMedium, (type & directLightingBRDFTypes) != NONE, unbend && (wi == rd), probability * wi.pdf);

            if (_mm_movemask_ps(_mm_cmpge_ps(scatteredWeight*probability, _mm_set1_ps(minContribution))) & 0x7)
              this->rays.increment();
          }
        }
      
        /*! Check if any BRDF component uses direct lighting. */
        bool useDirectLighting = false;
        for (size_t i=0; i<brdfs.size(); i++)
          useDirectLighting |= (brdfs[i]->type & directLightingBRDFTypes) != NONE;
      
        /*! Direct lighting. Shoot shadow rays to all light sources. */
        if (useDirectLighting)
        {
          const Vector3f wo = -rd;

          for (size_t i=0; i<lightCount; i++)
          {
            if ((lights[i]->illumMask & dg.illumMask) == 0)
              continue;
          
            /*! Either use precomputed samples for the light or sample light now. */
            LightSample ls;
            if (lights[i]->precompute()) ls = precomputedSample->getLightSample(integrator.precomputedLightSampleID[i]);
            else ls.L = lights[i]->sample(dg, ls.wi, ls.tMax, precomputedSample->getVec2f(lightSampleID));
          
            /*! Ignore zero radiance or illumination from the back. */
            //if (ls.L == Color(zero) || ls.wi.pdf == 0.0f || dot(dg.Ns,Vector3f(ls.wi)) <= 0.0f) continue;
            if (ls.L == Color(zero) || ls.wi.pdf == 0.0f) continue;
          
            /*! Evaluate BRDF */
            Color brdf = brdfs.eval(wo, dg, ls.wi, directLightingBRDFTypes);
            if (brdf == Color(zero)) continue;
          
            float eps = dg.error*epsilon;

            /*! Test for shadows. */
            shadowPaths[shadowRays.getSize()].set(weight * ls.L * brdf * rcp(ls.wi.pdf), pixelIndex);
            shadowRays.nextRay()->set(dg.P, ls.wi, eps, ls.tMax-eps);//,dg.shadowMask);
            shadowRays.nextRayExtra()->set(dg.P, ls.wi, rayExtra.time);
            shadowRays.increment();
          }
        }
      }
      
      if (shadowRays.getSize()) {
        RayStreamResult<true> result = shadowRays.dispatch(scene, thread);
        
        const unsigned char* hits = result.hits;
        unsigned count = result.rayCount;
        
        streamState.numRays += count;

        for (unsigned i = 0; i < count; ++i) {
          if (hits[i])
            continue;
          
          const ShadowPath& path = shadowPaths[i];
          pixelOutput[path.pixelIndex] += Color(_mm_load_ps(path.weight));
        }
      }
    }
    
  };
  
}

#endif
