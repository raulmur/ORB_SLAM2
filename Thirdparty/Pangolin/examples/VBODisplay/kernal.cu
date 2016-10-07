// Colour Sine wave Kernal
// Based on kernal_colour in kernelVBO.cpp by Rob Farber
__global__ void kernel(float4* dVertexArray, uchar4 *dColorArray,
           unsigned int width, unsigned int height, float time)
{
    unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

    // Each thread is unique point (u,v) in interval [-1,1],[-1,1]
    const float u = 2.0f* (x/(float)width)  - 1.0f;
    const float v = 2.0f* (y/(float)height) - 1.0f;
    const float w = 0.5f * sinf(4.0f*u + time) * cosf(4.0f*v + time);

    // Update vertex array for point
    dVertexArray[y*width+x] = make_float4(u, w, v, 1.0f);

    // Update colour array for point
    dColorArray[y*width+x].w = 0.0f;
    dColorArray[y*width+x].x = 255.0f *0.5f*(1.f+sinf(w+x));
    dColorArray[y*width+x].y = 255.0f *0.5f*(1.f+sinf(x)*cosf(y));
    dColorArray[y*width+x].z = 255.0f *0.5f*(1.f+sinf(w+time/10.0f));
}

extern "C" void launch_kernel(float4* dVertexArray, uchar4* dColourArray,
            unsigned int width, unsigned int height, float time)
{
  dim3 block(8, 8, 1);
  dim3 grid(width / block.x, height / block.y, 1);
  kernel<<< grid, block>>>(dVertexArray, dColourArray, width, height, time);
}
