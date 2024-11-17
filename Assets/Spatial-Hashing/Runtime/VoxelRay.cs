using System;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace HMH.ECS.SpatialHashing
{
    public struct VoxelRayIterator
    {
        public int3 Current;       // 当前体素索引
        private int3 _step;        // 步进方向（-1 或 1）
        private float3 _tMax;      // 射线到达下一个体素边界的距离
        private float3 _tDelta;    // 射线穿越一个体素所需的距离
        private int3 _voxelIndex;  // 当前体素索引（私有）
        private int3 _gridSize;    // 网格大小（体素数量）
        private float3 _origin;    // 射线起点
        private float3 _direction; // 射线方向（归一化）
        private float3 _voxelSize; // 单个体素的大小
        private float3 _gridOrigin;// 网格的起点（最小边界）
        private float _maxDistance;// 射线的最大距离

        // 构造函数
        public VoxelRayIterator(float3 origin, float3 direction, float3 voxelSize, float3 gridOrigin, int3 gridSize)
        {
            // 初始化成员变量
            _origin = origin;
            _direction = direction;
            _voxelSize = voxelSize;
            _gridOrigin = gridOrigin;
            _gridSize = gridSize;
            _maxDistance = float.PositiveInfinity;

            // 计算初始体素索引
            float3 relativeOrigin = (origin - gridOrigin) / voxelSize;
            _voxelIndex = (int3)math.floor(relativeOrigin);
            Current = _voxelIndex;

            // 计算步长和初始 tMax、tDelta
            _step = new int3(
                _direction.x > 0 ? 1 : (_direction.x < 0 ? -1 : 0),
                _direction.y > 0 ? 1 : (_direction.y < 0 ? -1 : 0),
                _direction.z > 0 ? 1 : (_direction.z < 0 ? -1 : 0)
            );

            float3 nextVoxelBoundary = (_voxelIndex + (_step + 1) / 2) * voxelSize + gridOrigin;

            _tMax = new float3(
                _direction.x != 0 ? (nextVoxelBoundary.x - origin.x) / _direction.x : float.PositiveInfinity,
                _direction.y != 0 ? (nextVoxelBoundary.y - origin.y) / _direction.y : float.PositiveInfinity,
                _direction.z != 0 ? (nextVoxelBoundary.z - origin.z) / _direction.z : float.PositiveInfinity
            );

            _tDelta = new float3(
                _direction.x != 0 ? voxelSize.x / math.abs(_direction.x) : float.PositiveInfinity,
                _direction.y != 0 ? voxelSize.y / math.abs(_direction.y) : float.PositiveInfinity,
                _direction.z != 0 ? voxelSize.z / math.abs(_direction.z) : float.PositiveInfinity
            );
        }

        // MoveNext 方法
        public bool MoveNext(float maxDistance)
        {
            // 实现迭代逻辑
            // 检查是否超过最大距离
            if (math.min(_tMax.x, math.min(_tMax.y, _tMax.z)) > maxDistance)
                return false;

            if (_tMax.x < _tMax.y && _tMax.x < _tMax.z)
            {
                _voxelIndex.x += _step.x;
                _tMax.x += _tDelta.x;
            }
            else if (_tMax.y < _tMax.z)
            {
                _voxelIndex.y += _step.y;
                _tMax.y += _tDelta.y;
            }
            else
            {
                _voxelIndex.z += _step.z;
                _tMax.z += _tDelta.z;
            }

            Current = _voxelIndex;

            // 检查体素是否在网格范围内
            if (math.any(_voxelIndex < 0) || math.any(_voxelIndex >= _gridSize))
                return false;

            return true;
        }
    }
    
    /// <summary>
    /// Ray for ray casting inside a voxel world. Each voxel is considered as a cube within this ray. A ray consists of a starting position, a direction and a length.
    /// Adaptation from https://www.gamedev.net/blogs/entry/2265248-voxel-traversal-algorithm-ray-casting/
    /// </summary>
    public struct VoxelRay<T> where T : IRay
    {
        /**
         * Casts the ray from its starting position towards its direction whilst keeping in mind its length. A lambda parameter is supplied and called each time a voxel is traversed.
         * This allows the lambda to stop anytime the algorithm to continue its loop.
         *
         * This method is local because the parameter voxelIndex is locally changed to avoid creating a new instance of {@link Vector3i}.
         *
         * @param voxelHalfExtent   The half extent (radius) of a voxel.
         * @param onTraversingVoxel The operation to execute when traversing a voxel. This method called the same number of times as the value of {@link #getVoxelDistance()}. The
         *                          supplied {@link Vector3i} parameter is not a new instance but a local instance, so it is a reference. The return value {@link Boolean} defines if
         *                          the algorithm should stop.
         * @param voxelIndex        The voxel index to locally modify in order to traverse voxels. This parameter exists simply to avoid creating a new {@link Vector3i} instance.
         *
         * @see <a href="http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf">Axel Traversal Algorithm</a>
         */
        public bool RayCast(ref T asker, float3 start, float3 direction, float length)
        {
            if (math.any(math.isnan(direction)))
                return true;

            Assert.IsTrue(Math.Abs(math.length(direction) - 1F) < 0.00001F);

            var currentVoxel = asker.GetIndexVoxel(start);

            var voxelDistance = ComputeVoxelDistance(ref asker, start + direction * length, currentVoxel);

            // In which direction the voxel ids are incremented.
            var directionStep = GetSignZeroPositive(direction);

            // Distance along the ray to the next voxel border from the current position (max.x, max.y, max.z).
            var nextVoxelBoundary = asker.GetPositionVoxel(currentVoxel + GetNegativeSign(directionStep) + 1, false);

            // distance until next intersection with voxel-border
            // the value of t at which the ray crosses the first vertical voxel boundary
            var max = new float3
            {
                x = math.abs(direction.x) > 0.00001F ? (nextVoxelBoundary.x - start.x) / direction.x : float.MaxValue,
                y = math.abs(direction.y) > 0.00001F ? (nextVoxelBoundary.y - start.y) / direction.y : float.MaxValue,
                z = math.abs(direction.z) > 0.00001F ? (nextVoxelBoundary.z - start.z) / direction.z : float.MaxValue
            };

            // how far along the ray we must move for the horizontal component to equal the width of a voxel
            // the direction in which we traverse the grid
            // can only be FLT_MAX if we never go in that direction
            var delta = new float3
            {
                x = math.abs(direction.x) > 0.00001F ? directionStep.x * asker.CellSize.x / direction.x : float.MaxValue,
                y = math.abs(direction.y) > 0.00001F ? directionStep.y * asker.CellSize.y / direction.y : float.MaxValue,
                z = math.abs(direction.z) > 0.00001F ? directionStep.z * asker.CellSize.z / direction.z : float.MaxValue
            };

            if (asker.OnTraversingVoxel(currentVoxel))
                return true;

            int traversedVoxelCount = 0;

            while (++traversedVoxelCount < voxelDistance)
            {
                if (max.x < max.y && max.x < max.z)
                {
                    currentVoxel.x += directionStep.x;
                    max.x          += delta.x;
                }
                else if (max.y < max.z)
                {
                    currentVoxel.y += directionStep.y;
                    max.y          += delta.y;
                }
                else
                {
                    currentVoxel.z += directionStep.z;
                    max.z          += delta.z;
                }

                if (asker.OnTraversingVoxel(currentVoxel))
                    return true;
            }

            return false;
        }

        /**
         * Computes the voxel distance, a.k.a. the number of voxel to traverse, for the ray cast.
         *
         * @param voxelExtent The extent of a voxel, which is the equivalent for a cube of a sphere's radius.
         * @param startIndex The starting position's index.
         */
        private int ComputeVoxelDistance(ref T asker, float3 end, int3 startIndex)
        {
            return 1 + math.abs(asker.GetIndexVoxel(end) - startIndex).Sum();
        }

        /// <summary>
        /// Gets the sign of the supplied number. The method being "zero position" means that the sign of zero is 1.
        /// </summary>
        public static int3 GetSignZeroPositive(float3 number)
        {
            return GetNegativeSign(number) | 1;
        }

        /// <summary>
        /// Gets the negative sign of the supplied number. So, in other words, if the number is negative, -1 is returned but if the number is positive or zero, then zero is returned.
        /// </summary>
        /// <param name="number"></param>
        /// <returns></returns>
        public static int3 GetNegativeSign(float3 number)
        {
            return new int3(math.asint(number.x) >> (32 - 1),
                            math.asint(number.y) >> (32 - 1),
                            math.asint(number.z) >> (32 - 1)); //float are always 32bit in c# and -1 for sign bit which is at position 31
        }

        /// <summary>
        /// Gets the negative sign of the supplied number. So, in other words, if the number is negative, -1 is returned but if the number is positive or zero, then zero is returned.
        /// </summary>
        /// <param name="number"></param>
        /// <returns></returns>
        public static int3 GetNegativeSign(int3 number)
        {
            return new int3(math.asint(number.x) >> (32 - 1),
                            math.asint(number.y) >> (32 - 1),
                            math.asint(number.z) >> (32 - 1)); //int are always 32bit in c# and -1 for sign bit which is at position 31
        }

    }

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

    public interface IRay2D
    {
        bool OnTraversingVoxel(int2 voxelIndex);

        int2 GetIndexVoxel(float2    position);
        float2 GetPositionVoxel(int2 index, bool center);
        float2 CellSize { get; }
    }
}