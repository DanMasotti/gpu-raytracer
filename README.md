# GPU Raytracing

## Idea: 
Use the graphics card for doing our ray tracing computations in parallel.

### How:
We can render our scene to a textured fullscreen quad, doing all the compute in a shader.  In this way, we cast a ray from each fragment, doing all the compute for a single ray in the shader.  We tried this two ways:


- Fragment Shader:  We've seen these before.  We're doing very similar things as we did in Lab 7 and 8 with frame buffers.  The difference is that instead of doing blur compute, we're writing a whole ray tracer in the fragment shader!  The idea here is that rays are cast from each corner of the quad and the fragment shader interpolates between rays in the quad.  We render this color to a color attachment on an FBO, because although we didn't get to it by the deadline, we could then use the texture in the FBO to do post processing effects like motion blur.  

- I worked on rendering the fragment shader.  This meant consuming fragment coordinates from the vertex shader and doing ray tracing inside the shader. There's considerable things to keep in mind how this changes from doing our Ray assignment in C++:
	
	- The shader is one ray trace job, and we do this per fragment coordinate on the film plane.
	- Recursion is non-trivial in a shader. So I converted recursion to an iterative algorithm, in which we ping the rays one by one. Importantly, we render reflections and refractions of reflections, but not reflections and refractions of refractions. I tried going down this route but it was challenging without a stack-like data structure (while stack not empty, pop ray, refract reflect, etc.)
	- Adding refractions and also refracting each wavelength differently depending on the IOR of materials.  I didn't get the chromatic abberations I wanted but I can clearly see different "bands" of refractions on the center sphere.
	- Fresnel Equations for determining how much light is reflected/refracted
	- Ironically, I was going for realism, but while playing with coefficients, I found a nice toon looking render.  Rather than play with parameters further, I just stayed with it.  We also didn't use image textures so modelling things like metal photorealistically was challenging.  I pretty much experimented with coefficients until I found something that I aesthically liked.  I think messing with Fresnel gave this cool bright highlights around the edges, which I appreciated.
	- To demonstrate the GPU capabilities, I passed in a timer to oscillate the spheres instead of rendering a static scene.  This was boiler plate before Amir got the physics sim up and running.


## NOTES:
	In View.cpp
	- I had to turn down the fps because of my computer using the parameter `m_sleepTime`, which stops the program from updating too frequently.  Feel free to change this.  
	- Also note that I switched to Windows 10 from Mac to work on this as not all the newer OpenGL features are supported.  I think things should generally work.  Amir was able to run the raytracer on Linux.




