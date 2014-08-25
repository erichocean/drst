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

#ifndef __EMBREE_TILEPIPELINE_H__
#define __EMBREE_TILEPIPELINE_H__

#include "../integrators/integrator.h"
#include "tile.h"
#include <vector>
#include <algorithm>

namespace embree {

  class TilePipeline {
  private:
    std::vector<Tile*> freeTiles;
    std::vector<Tile*> busyTiles;

  public:
    TilePipeline(Integrator* integrator, unsigned tileCount, unsigned spp, const PrecomputedSample* const* sampleSets, bool accumulate, SwapChain* swapchain, ToneMapper* toneMapper, FrameBuffer* framebuffer) {
      for (unsigned i = 0; i < tileCount; ++i)
        freeTiles.push_back(new Tile(integrator, spp, sampleSets, accumulate, swapchain, toneMapper, framebuffer));
    }

    Tile* advanceUntilFree() {
      while (freeTiles.empty()) {
        for (size_t i = 0; i < busyTiles.size(); ++i) {
          if (!busyTiles[i]->process()) {
            freeTiles.push_back(busyTiles[i]);
            busyTiles[i] = 0;
          }
        }
        busyTiles.resize(std::distance(busyTiles.begin(), std::remove(busyTiles.begin(), busyTiles.end(), (Tile*)0)));
      }

      Tile* tile = freeTiles.back();
      freeTiles.pop_back();
      busyTiles.push_back(tile);
      return tile;
    }

    void processOutstanding() {
      while (!busyTiles.empty()) {
        for (size_t i = 0; i < busyTiles.size(); ++i) {
          if (!busyTiles[i]->process()) {
            freeTiles.push_back(busyTiles[i]);
            busyTiles[i] = 0;
          }
        }
        busyTiles.resize(std::distance(busyTiles.begin(), std::remove(busyTiles.begin(), busyTiles.end(), (Tile*)0)));
      }
    }

    unsigned computeNumRays() {
      unsigned numRays = 0;

      for (size_t i = 0; i < freeTiles.size(); ++i)
        numRays += freeTiles[i]->streamState.numRays;

      return numRays;
    }

    ~TilePipeline() {
      processOutstanding();

      for (size_t i = 0; i < freeTiles.size(); ++i)
        delete freeTiles[i];
    }
  };

}

#endif
