#include <iostream>

#include <GL/glew.h>

#include <pangolin/pangolin.h>
#include <pangolin/gl/glcuda.h>
#include <pangolin/gl/glvbo.h>

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <vector_types.h>

using namespace pangolin;
using namespace std;

// Mesh size
const int mesh_width=256;
const int mesh_height=256;

extern "C" void launch_kernel(float4* dVertexArray, uchar4* dColourArray, unsigned int width, unsigned int height, float time);

int main( int /*argc*/, char* argv[] )
{
//  cudaGLSetGLDevice(0);

  pangolin::CreateWindowAndBind("Main",640,480);
  glewInit();
  
  // 3D Mouse handler requires depth testing to be enabled  
  glEnable(GL_DEPTH_TEST);  

  // Create vertex and colour buffer objects and register them with CUDA
  GlBufferCudaPtr vertex_array(
      GlArrayBuffer, mesh_width*mesh_height, GL_FLOAT, 4,
      cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW
  );
  GlBufferCudaPtr colour_array(
      GlArrayBuffer, mesh_width*mesh_height, GL_UNSIGNED_BYTE, 4,
      cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW
  );

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
    ModelViewLookAt(-0,2,-2, 0,0,0, AxisY)
  );
  const int UI_WIDTH = 180;

  // Add named OpenGL viewport to window and provide 3D Handler
  View& d_cam = pangolin::Display("cam")
    .SetBounds(0.0, 1.0, Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
    .SetHandler(new Handler3D(s_cam));

  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  View& d_panel = pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, Attach::Pix(UI_WIDTH));

  // Default hooks for exiting (Esc) and fullscreen (tab).
  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    static double time = 0;
    static Var<double> delta("ui.time delta", 0.001, 0, 0.005);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glColor3f(1.0,1.0,1.0);

    {
      CudaScopedMappedPtr var(vertex_array);
      CudaScopedMappedPtr car(colour_array);
      launch_kernel((float4*)*var,(uchar4*)*car,mesh_width,mesh_height,time);
      time += delta;
    }

    pangolin::RenderVboCbo(vertex_array, colour_array);

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
