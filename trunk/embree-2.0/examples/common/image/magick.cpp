// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifdef USE_IMAGEMAGICK

#include "image/image.h"
#include <iostream>

/*! include Image Magick headers */
#include <Magick++.h>
using namespace Magick;

namespace embree
{
  Ref<Image> loadMagick(const FileName& fileName)
  {
#if 0
    if (std::string(fileName.c_str()).size() > 2 && fileName.c_str()[1] == ' ') {
      Magick::Image image(fileName.c_str()+2);
      Image* out = new Image4f(image.columns(),image.rows(),fileName);
      float rcpQuantumRange = 1.0f/float(QuantumRange);
      Magick::Pixels pixel_cache(image);
      Magick::PixelPacket* pixels = pixel_cache.get(0,0,out->width,out->height);
    
      for (size_t y=0; y<out->height; y++) {
        for (size_t x=0; x<out->width; x++) {
          Color4 c;
          if (fileName.c_str()[0] == 'a') {
            c.r =
            c.g =
            c.b = float(pixels[y*out->width+x].opacity )*rcpQuantumRange;
            c.a = 1.f;
          } else {
            c.r = float(pixels[y*out->width+x].red  )*rcpQuantumRange;
            c.g = float(pixels[y*out->width+x].green)*rcpQuantumRange;
            c.b = float(pixels[y*out->width+x].blue )*rcpQuantumRange;
            c.a = 1.f;
          }
          out->set(x,y,c);
        }
      }

      return out;
      // if (fileName[0] == 'd') {
      //   Ref<Image> img = loadMagick(fileName.c_str()+2);
        
      //   return img;
      // }
      // if (fileName[0] == 'a')
      //   return NULL;
    }
#endif
    Magick::Image image(fileName.c_str());
    Image* out = new Image4c(image.columns(),image.rows(),fileName);
    float rcpQuantumRange = 1.0f/float(QuantumRange);
    Magick::Pixels pixel_cache(image);
    Magick::PixelPacket* pixels = pixel_cache.get(0,0,out->width,out->height);
    
    for (size_t y=0; y<out->height; y++) {
      for (size_t x=0; x<out->width; x++) {
        Color4 c;
        c.r = float(pixels[y*out->width+x].red    )*rcpQuantumRange;
        c.g = float(pixels[y*out->width+x].green  )*rcpQuantumRange;
        c.b = float(pixels[y*out->width+x].blue   )*rcpQuantumRange;
        c.a = 1.0f - float(pixels[y*out->width+x].opacity)*rcpQuantumRange;
    c.r *= c.a;
    c.g *= c.a;
    c.b *= c.a;
        out->set(x,y,c);
      }
    }

    return out;
  }

  void storeMagick(const Ref<Image>& img, const FileName& fileName)
  {
    Magick::Image image(Magick::Geometry(img->width,img->height),
                        Magick::Color(0,0,0,0));
    image.modifyImage();

    Magick::Pixels pixel_cache(image);
    Magick::PixelPacket* pixels = pixel_cache.get(0,0,img->width,img->height);
    for (size_t y=0; y<img->height; y++) {
      for (size_t x=0; x<img->width; x++) {
        Color4 c = img->get(x,y);
        pixels[y*img->width+x] = Magick::Color(Magick::Quantum(clamp(c.r)*QuantumRange),
                                               Magick::Quantum(clamp(c.g)*QuantumRange),
                                               Magick::Quantum(clamp(c.b)*QuantumRange),
                                               Magick::Quantum(clamp(c.a)*QuantumRange));
      }
    }
    pixel_cache.sync();
    image.write(fileName.c_str());
  }
}

#endif
