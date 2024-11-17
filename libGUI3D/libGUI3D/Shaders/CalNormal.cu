#include <cuda_runtime.h>

__global__ void ComputeNormalMap_kernel(int width, int height, float3 *vertexMap, float3 *normalMap)
{
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
    int idx = y * width +x;
    if( x < 1 || x >= width - 1 || y < 1 || y >= height - 1 ){
        normalMap[idx] = make_float3(0.f,0.f,0.f);
        return;
    }
    
    float3 left =  vertexMap[idx - 1];
    float3 right = vertexMap[idx + 1];
    float3 upper = vertexMap[idx - width];
    float3 lower = vertexMap[idx + width];
    
    float3 normal;
    if(left.z == 0 || right.z == 0 || upper.z == 0 || lower.z == 0 ){
        normal.x = normal.y = normal.z = 0;
    }else{
    
        float3 hor = left - right;
        float3 ver = upper - lower;
        
        normal = cross(hor, ver);
        normal = normalize( normal );
        
        if (normal.z > 0){
            normal *= -1;
        }
        
    }
    
    normalMap[idx] = normal;
}

void ComputeNormalMap(int width, int height, float3 *vertexMap, float3 *normalMap)
{
    dim3 threads(16, 16);
    dim3 blocks( (width + threads.x-1) / threads.x, (height + threads.y-1) / threads.y );
    ComputeNormalMap_kernel<<<blocks, threads>>>( width, height, vertexMap, normalMap );
    cudaDeviceSynchronize();
}
