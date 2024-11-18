using Unity.Mathematics;

namespace SpatialHashing
{
    public interface IRay
    {
        /// <summary>
        /// The operation to execute when traversing a voxel.The return value defines if the algorithm should stop.
        /// </summary>
        /// <param name="voxelIndex"></param>
        /// <returns></returns>
        bool OnTraversingVoxel(int3 voxelIndex);

        int3 GetIndexVoxel(float3    position);
        float3 GetPositionVoxel(int3 index, bool center);
        float3 CellSize { get; }
    }
}