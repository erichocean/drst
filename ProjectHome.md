Dynamic Ray Stream Traversal implemented in Embree 2.0.

Paper: http://fileadmin.cs.lth.se/graphics/research/papers/2014/drst

# Software and hardware requirements #

Requires Microsoft Visual Studio 2013, Intel C++ Compiler XE 14.0 and Windows 7 64-bit.

Requires a CPU with AVX2 support and has only been tested on a mobile Haswell chip with 128 MB L4 cache. It is likely that the optimal ray stream size is different on a processor with a different cache hierarchy.

# Getting started #

You can export the code using the following svn-command:

```
svn export http://drst.googlecode.com/svn/trunk/embree-2.0
```

The project requires that Intel C++ Compiler XE 14.0 is installed into `C:\Program Files (x86)\Intel\Composer XE 2013 SP1`. If your path differs you need to change the VC++ Directories in each project.

Set startup-project to "renderer" and working directory to `$(SolutionDir)`.

For best results, run Embree without the Visual Studio debugger, 16 spp and bvh4.triangle8 acceleration structure. Example command-line arguments:

```
-c ../crown/crown.ecs -spp 16 -accel bvh4.triangle8
```

where crown.ecs is the scene to be rendered (assumes that the crown has been stored in that location, look for it at the Embree project's home page).

## Switching between single-ray and dynamic ray stream traversal ##

You can switch single-ray traversal on and off by changing the bool-variable `streamTraversal` in integratorrenderer.cpp in device\_singleray.