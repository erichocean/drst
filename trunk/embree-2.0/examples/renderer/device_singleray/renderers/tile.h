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

#ifndef __EMBREE_TILE_H__
#define __EMBREE_TILE_H__

#include "../integrators/integrator.h"
#include "renderer.h"

namespace embree {

  class Tile {
    ALIGNED_CLASS

  private:
    unsigned spp;
    TileIntegrator* tileIntegrator;
    
    std::vector<Vec2f, AlignedAllocator<Vec2f, 64> > pixelLocations;
    std::vector<Color, AlignedAllocator<Color, 64> > pixelRadiance;
    std::vector<unsigned char, AlignedAllocator<unsigned char, 64> > pixelSampleSets;

    size_t tile_x;
    size_t tile_y;

    bool accumulate;
    SwapChain* swapchain;
    ToneMapper* toneMapper;
    FrameBuffer* framebuffer;

  public:
    IntegratorStreamState streamState;

    Tile(Integrator* integrator, unsigned spp, const PrecomputedSample* const* sampleSets, bool accumulate, SwapChain* swapchain, ToneMapper* toneMapper, FrameBuffer* framebuffer) :
      spp(spp), accumulate(accumulate), swapchain(swapchain), toneMapper(toneMapper), framebuffer(framebuffer) {
      tileIntegrator = integrator->createTileIntegrator();

      unsigned sppLog2 = _tzcnt_u32(spp);

      if (spp != (1 << sppLog2))
        throw std::runtime_error("samples per pixel not pow 2");

      size_t pixelsPerTile = Renderer::TILE_SIZE * Renderer::TILE_SIZE;

      pixelLocations.resize(pixelsPerTile);
      pixelRadiance.resize(pixelsPerTile);
      pixelSampleSets.resize(pixelsPerTile);

      streamState.pixelLocations = &pixelLocations[0];
      streamState.pixelOutput = &pixelRadiance[0];
      streamState.pixelSampleSets = &pixelSampleSets[0];
      streamState.sampleSets = sampleSets;
      streamState.samplesPerPixelLog2 = sppLog2;
      streamState.numRays = 0;
    }

    unsigned setup(Camera* camera, size_t tile_x, size_t tile_y, Random& randomNumberGenerator, unsigned sampleSetCount, float rcpWidth, float rcpHeight) {
      unsigned currentPixel = 0;

      this->tile_x = tile_x;
      this->tile_y = tile_y;

      for (size_t dy=0; dy<Renderer::TILE_SIZE; dy++) {
        size_t y = tile_y+dy;
        if (y >= swapchain->getHeight()) continue;

        for (size_t dx=0; dx<Renderer::TILE_SIZE; dx++) {
          size_t x = tile_x+dx;
          if (x >= swapchain->getWidth()) continue;

          const int set = randomNumberGenerator.getInt(sampleSetCount);
          pixelSampleSets[currentPixel] = set;
          pixelLocations[currentPixel] = Vec2f(float(x)*rcpWidth, float(y)*rcpHeight);
          pixelRadiance[currentPixel] = Color(0.0f, 0.0f, 0.0f);
          ++currentPixel;
        }
      }

      unsigned pixelCount = currentPixel;
      unsigned sampleCount = pixelCount*spp;

      RayStream<false>& rays = tileIntegrator->getRayStream();

      const unsigned char* __restrict pixelSampleSets = streamState.pixelSampleSets;
      const Vec2f* __restrict pixelLocations = streamState.pixelLocations;
      const PrecomputedSample* const __restrict * __restrict sampleSets = streamState.sampleSets;
      unsigned samplesPerPixelLog2 = streamState.samplesPerPixelLog2;
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

      return pixelCount;
    }

    void dipatch(BackendScene* scene, unsigned thread) {
      tileIntegrator->dispatch(this, scene, streamState, thread);
    }

    bool process() {
      if (!tileIntegrator->process()) {
        resolve();
        return false;
      }
      return true;
    }

    ~Tile() {
      delete tileIntegrator;
    }

  private:
    void resolve() {
      unsigned currentPixel = 0;

      for (size_t dy=0; dy<Renderer::TILE_SIZE; dy++)
      {
        size_t y = tile_y+dy;
        if (y >= swapchain->getHeight()) continue;

        for (size_t dx=0; dx<Renderer::TILE_SIZE; dx++)
        {
          size_t x = tile_x+dx;
          if (x >= swapchain->getWidth()) continue;

          Color L = pixelRadiance[currentPixel++];
          const Color L0 = swapchain->update(x, y, L, spp, accumulate);
          const Color L1 = toneMapper->eval(L0,x,y,swapchain);
          framebuffer->set(x, y, L1);
        }
      }

      framebuffer->finishTile();
    }
  };

}

#endif
