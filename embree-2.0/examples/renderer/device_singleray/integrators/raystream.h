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

#ifndef __EMBREE_RAY_STREAM_H__
#define __EMBREE_RAY_STREAM_H__

#include "embree/common/ray.h"
#include "embree/common/stream_ray.h"
#include "embree/common/alignedallocator.h"
#include "../api/scene.h"
#include "../renderers/renderer.h"
#include <vector>

namespace embree {

  template<bool BooleanTest>
  struct RayStreamResult {
  };
  
  template<>
  struct RayStreamResult<true> {
    unsigned rayCount;
    const unsigned char* __restrict hits;
  };
  
  template<bool BooleanTest>
  class __align(64) RayStream {
  };
  
  template<>
  class __align(64) RayStream<false> {
  private:
    unsigned size;
    StreamRay* __restrict  rays;
    StreamRayExtra* __restrict  rayExtras;
    StreamHit* __restrict  hits;

  public:
    __forceinline RayStream(unsigned maxSize) {
      size = 0;
      rays = static_cast<StreamRay*>(alignedMalloc(sizeof(StreamRay)*maxSize));
      rayExtras = static_cast<StreamRayExtra*>(alignedMalloc(sizeof(StreamRayExtra)*maxSize));
      hits = static_cast<StreamHit*>(alignedMalloc(sizeof(StreamHit)*maxSize));
    }
    
    __forceinline unsigned getSize() const {
      return size;
    }

    __forceinline StreamRay* __restrict  getRays() const {
      return rays;
    }

    __forceinline StreamRayExtra* __restrict  getRayExtras() const {
      return rayExtras;
    }

    __forceinline StreamHit* __restrict getHits() const {
      return hits;
    }
    
    __forceinline StreamRay* __restrict nextRay() {
      return rays + size;
    }

    __forceinline StreamRayExtra* __restrict nextRayExtra() {
      return rayExtras + size;
    }

    __forceinline void increment() {
      hits[size].id0 = -1;
      ++size;
    }
    
    void reset(unsigned maxSize) {
      alignedFree(rays);
      alignedFree(hits);
      size = 0;
      rays = static_cast<StreamRay*>(alignedMalloc(sizeof(StreamRay)*maxSize));
      rayExtras = static_cast<StreamRayExtra*>(alignedMalloc(sizeof(StreamRayExtra)*maxSize));
      hits = static_cast<StreamHit*>(alignedMalloc(sizeof(StreamHit)*maxSize));
    }

    __forceinline void dispatch(BackendScene* scene, unsigned thread) {
      unsigned count = size;
      size = 0;
      scene->intersectorStream->intersect(rays, rayExtras, hits, count, thread);
    }

    __forceinline ~RayStream() {
      alignedFree(rays);
      alignedFree(rayExtras);
    }
  };
  
  template<>
  class __align(64) RayStream<true> {
  private:
    unsigned size;
    StreamRay* __restrict rays;
    StreamRayExtra* __restrict rayExtras;
    unsigned char* hits;
    
  public:
    __forceinline RayStream(unsigned maxSize) {
      size = 0;
      rays = static_cast<StreamRay*>(alignedMalloc(sizeof(StreamRay)*maxSize));
      rayExtras = static_cast<StreamRayExtra*>(alignedMalloc(sizeof(StreamRayExtra)*maxSize));
      hits = static_cast<unsigned char*>(alignedMalloc(maxSize));
    }
    
    __forceinline unsigned getSize() const {
      return size;
    }
    
    __forceinline StreamRay* __restrict nextRay() {
      return rays + size;
    }

    __forceinline StreamRayExtra* __restrict nextRayExtra() {
      return rayExtras + size;
    }

    __forceinline void increment() {
      ++size;
    }
    
    void reset(unsigned maxSize) {
      alignedFree(rays);
      alignedFree(rayExtras);
      alignedFree(hits);
      size = 0;
      rays = static_cast<StreamRay*>(alignedMalloc(sizeof(StreamRay)*maxSize));
      rayExtras = static_cast<StreamRayExtra*>(alignedMalloc(sizeof(StreamRayExtra)*maxSize));
      hits = static_cast<unsigned char*>(alignedMalloc(maxSize));
    }
    
    __forceinline RayStreamResult<true> dispatch(BackendScene* scene, unsigned thread) {
      unsigned offset = 0;
      unsigned remaining = size;

      const unsigned batchSize = Renderer::TILE_SIZE * Renderer::TILE_SIZE * 16;
      
      while (remaining > batchSize) {
        scene->intersectorStream->occluded(rays + offset, rayExtras + offset, batchSize, hits + offset, thread);
        offset += batchSize;
        remaining -= batchSize;
      }

      scene->intersectorStream->occluded(rays + offset, rayExtras + offset, remaining, hits + offset, thread);
      
      RayStreamResult<true> result = {
        size,
        hits,
      };
      
      size = 0;
      return result;
    }
    
    __forceinline ~RayStream() {
      alignedFree(rays);
      alignedFree(rayExtras);
      alignedFree(hits);
    }
  };
  
}

#endif
