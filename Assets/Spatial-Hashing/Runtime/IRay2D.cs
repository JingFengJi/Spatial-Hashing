using Unity.Mathematics;

namespace SpatialHashing
{
    public interface IRay2D
    {
        bool OnTraversingVoxel(int2 voxelIndex);

        int2 GetIndexVoxel(float2    position);
        float2 GetPositionVoxel(int2 index, bool center);
        float2 CellSize { get; }
    }
}