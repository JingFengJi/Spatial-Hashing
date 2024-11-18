﻿using System;
using System.Diagnostics.Contracts;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine.Assertions;

namespace HMH.ECS.SpatialHashing
{
    public unsafe struct SpatialHash2D<T> : IDisposable where T : unmanaged, ISpatialHashing2DItem<T>
    {
        #region Variables
        
        private static readonly float3 Forward = new float3(0F, 0F, 1F);
        private static readonly float3 Up     = new float3(0F, 1F, 0F);
        private static readonly float3 Right  = new float3(1F, 0F, 0F);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        // ReSharper disable InconsistentNaming
        private AtomicSafetyHandle m_Safety;
        [NativeSetClassTypeToNullOnSchedule]
        private DisposeSentinel m_DisposeSentinel;
        // ReSharper restore InconsistentNaming
#endif

        private Allocator _allocatorLabel;

        [NativeDisableUnsafePtrRestriction]
        private SpatialHash2DData* _data;
        private NativeParallelMultiHashMap<uint, int> _buckets;        //4
        private NativeParallelHashMap<int, Bounds2D>    _itemIDToBounds; //4
        private NativeParallelHashMap<int, T>         _itemIDToItem;   //4

        private NativeParallelHashSet<int2>      _helpMoveHashMapOld;
        private NativeParallelHashSet<int2>      _helpMoveHashMapNew;
        //private VoxelRay<SpatialHash2D<T>> _voxelRay;
        private int                      _rayHitValue;

        #endregion

        #region Proprieties

        public bool IsCreated => _data != null;

        public int ItemCount
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _itemIDToBounds.Count();
            }
        }

        public int BucketItemCount
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _buckets.Count();
            }
        }

        public float2 CellSize
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> CellSize;
            }
        }

        public Bounds2D WorldBounds
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> WorldBounds;
            }
        }
        
        public float2 WorldBoundsMin
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> WorldBoundsMin;
            }
        }

        public int2 CellCount
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> CellCount;
            }
        }

#if UNITY_EDITOR
        public NativeParallelMultiHashMap<uint, int> DebugBuckets => _buckets;
        public NativeParallelHashMap<int, T> DebugIDToItem => _itemIDToItem;
        public Bounds2D DebugRayCastBounds => _data -> RayCastBound;
        //public VoxelRay<SpatialHash<T>> DebugVoxelRay => _voxelRay;
#endif

        #endregion
        
        public SpatialHash2D(Bounds2D worldBounds, float2 cellSize, Allocator label)
            : this(worldBounds, cellSize, worldBounds.GetCellCount(cellSize).Mul() * 3, label)
        { }

        public SpatialHash2D(Bounds2D worldBounds, float2 cellSize, int initialSize, Allocator allocator)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            switch (allocator)
            {
                case <= Allocator.None:
                    throw new ArgumentException("Allocator must be Temp, TempJob or Persistent", nameof (allocator));
                case >= Allocator.FirstUserIndex:
                    throw new ArgumentException("Allocator must be Temp, TempJob or Persistent", nameof (allocator));
            }

            if (initialSize < 1)
                throw new ArgumentOutOfRangeException(nameof(initialSize), "InitialSize must be > 0");

            DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, 0, allocator);
#endif
            
            _allocatorLabel         = allocator;
            _data                   = (SpatialHash2DData*)UnsafeUtility.MallocTracked(sizeof(SpatialHash2DData), UnsafeUtility.AlignOf<SpatialHash2DData>(), allocator, 0);
            _data -> WorldBounds    = worldBounds;
            _data -> WorldBoundsMin = worldBounds.Min;
            _data -> CellSize       = cellSize;
            _data -> CellCount      = worldBounds.GetCellCount(cellSize);
            _data -> RayCastBound   = new Bounds2D();
            _data -> HasHit         = false;
            _data -> Counter        = 0;
            _data -> RayOrigin      = float2.zero;
            _data -> RayDirection   = float2.zero;

            _buckets            = new NativeParallelMultiHashMap<uint, int>(initialSize, allocator);
            _itemIDToBounds     = new NativeParallelHashMap<int, Bounds2D>(initialSize >> 1, allocator);
            _itemIDToItem       = new NativeParallelHashMap<int, T>(initialSize >> 1, allocator);
            _helpMoveHashMapOld = new NativeParallelHashSet<int2>(128, allocator);
            _helpMoveHashMapNew = new NativeParallelHashSet<int2>(128, allocator);

            //_voxelRay    = new VoxelRay<SpatialHash<T>>();
            _rayHitValue = 0;
        }

        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif
            UnsafeUtility.FreeTracked(_data, _allocatorLabel);

            _buckets.Dispose();
            _itemIDToBounds.Dispose();
            _itemIDToItem.Dispose();
            _helpMoveHashMapOld.Dispose();
            _helpMoveHashMapNew.Dispose();
        }

        #region I/O

        public void Add(ref T item)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var bounds = new Bounds2D(item.GetCenter(), item.GetSize());

            bounds.Clamp(_data -> WorldBounds);

            // TODO Maintien free id to replace hashmap by array
            var itemID = ++_data -> Counter;

            item.SpatialHashingIndex = itemID;
            _itemIDToBounds.TryAdd(itemID, bounds);
            _itemIDToItem.TryAdd(itemID, item);

            CalculateStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int2(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    AddInternal(hashPosition, itemID);
                }
            }
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddInternal(int2 position, int itemID)
        {
            _buckets.Add(Hash(position), itemID);
        }
        
        
        public void Remove(int itemID)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var success = _itemIDToBounds.TryGetValue(itemID, out var bounds);

            Assert.IsTrue(success);

            _itemIDToBounds.Remove(itemID);
            _itemIDToItem.Remove(itemID);

            CalculateStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int2(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    RemoveInternal(hashPosition, itemID);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void RemoveInternal(int2 voxelPosition, int itemID)
        {
            _buckets.Remove(Hash(voxelPosition), itemID);
        }
        
        /// <summary>
        /// 移动对象
        /// </summary>
        /// <param name="item"></param>
        public void Move(T item)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var itemID  = item.SpatialHashingIndex;
            var success = _itemIDToBounds.TryGetValue(itemID, out var oldBounds);
            Assert.IsTrue(success);

            var newBounds = new Bounds2D(item.GetCenter(), item.GetSize());
            newBounds.Clamp(_data -> WorldBounds);
            _itemIDToBounds.Remove(itemID);
            _itemIDToBounds.TryAdd(itemID, newBounds);
            _itemIDToItem.Remove(itemID);
            _itemIDToItem.TryAdd(itemID, item);

            //先根据旧的包围盒计算出旧的体素位置
            _helpMoveHashMapOld.Clear();
            SetVoxelIndexForBounds(_data, oldBounds, _helpMoveHashMapOld);
            
            //根据新的包围盒计算出新的体素位置
            _helpMoveHashMapNew.Clear();
            SetVoxelIndexForBounds(_data, newBounds, _helpMoveHashMapNew);

            //遍历旧的体素位置，如果新的体素位置不包含旧的体素位置，则删除
            foreach (var oldVoxelPosition in _helpMoveHashMapOld)
            {
                if (_helpMoveHashMapNew.Contains(oldVoxelPosition) == false)
                {
                    RemoveInternal(oldVoxelPosition, itemID);
                }
            }

            //遍历新的体素位置，如果旧的体素位置不包含新的体素位置，则添加
            foreach (var newVoxelPosition in _helpMoveHashMapNew)
            {
                if (_helpMoveHashMapOld.Contains(newVoxelPosition) == false)
                {
                    AddInternal(newVoxelPosition, itemID);
                }
            }
        }
        
        
        /// <summary>
        /// 根据包围盒计算出体素位置
        /// </summary>
        /// <param name="data"></param>
        /// <param name="bounds"></param>
        /// <param name="collection"></param>
        /// <exception cref="Exception"></exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void SetVoxelIndexForBounds(SpatialHash2DData* data, Bounds2D bounds, NativeParallelHashSet<int2> collection)
        {
            CalculateStartEndIterationInternal(data, bounds, out var start, out var end);

            var position = new int2(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                position.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    position.y = y;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
                    bool success = collection.Add(position);

                    if (success == false)
                    {
                        throw new Exception("Try to add a position already in bound collection");
                    }
#else
                        collection.Add(position);
#endif
                }
            }
        }
        
        
        public static uint Hash(int2 cellIndex)
        {
            return math.hash(cellIndex);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CalculateStartEndIterationInternal(SpatialHash2DData* data, Bounds2D bounds, out int2 start, out int2 end)
        {
            start = ((bounds.Min - data -> WorldBoundsMin) / data -> CellSize).FloorToInt();
            end   = ((bounds.Max - data -> WorldBoundsMin) / data -> CellSize).CeilToInt();
        }
        
        #endregion

        #region Query
        
        /// <summary>
        /// Bound搜索
        /// </summary>
        /// <param name="bounds"></param>
        /// <param name="resultList"></param>
        public void Query(Bounds2D bounds, NativeList<T> resultList)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            // Clamp 查询的 bounds 到世界范围内
            bounds.Clamp(_data->WorldBounds);

            // 计算查询范围覆盖的网格单元起始和结束索引
            CalculateStartEndIterationInternal(_data, bounds, out var start, out var end);

            // 用于跟踪已处理的 itemID，避免重复
            var processedItemIDs = new NativeParallelHashSet<int>(32, Allocator.Temp);

            var hashPosition = new int2(0);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    uint cellHash = Hash(hashPosition);

                    // 获取该网格单元中的所有 itemID
                    if (_buckets.TryGetFirstValue(cellHash, out int itemID, out var iterator))
                    {
                        do
                        {
                            // 检查是否已经处理过该 itemID
                            if (!processedItemIDs.Contains(itemID))
                            {
                                processedItemIDs.Add(itemID);

                                // 获取该 itemID 的 Bounds2D
                                if (_itemIDToBounds.TryGetValue(itemID, out var itemBounds))
                                {
                                    // 检查 itemBounds 是否与查询的 bounds 相交
                                    if (bounds.Intersects(itemBounds))
                                    {
                                        // 获取对应的 T 对象
                                        if (_itemIDToItem.TryGetValue(itemID, out var item))
                                        {
                                            resultList.Add(item);
                                        }
                                    }
                                }
                            }
                        } while (_buckets.TryGetNextValue(out itemID, ref iterator));
                    }
                }
            }

            // 释放临时的 HashSet
            processedItemIDs.Dispose();
        }

        private float2 Rotate(float2 v, float radians)
        {
            float sin = math.sin(radians);
            float cos = math.cos(radians);
            return new float2(
                v.x * cos - v.y * sin,
                v.x * sin + v.y * cos
            );
        }
        
        public Bounds2D CalculateSectorAABB(float2 origin, float2 direction, float angle, float radius)
        {
            float halfAngle = angle * 0.5f;

            // 计算左边界和右边界的方向向量
            float2 dirLeft = Rotate(direction, -halfAngle);
            float2 dirRight = Rotate(direction, halfAngle);

            // 初始化 AABB 的最小和最大点
            float2 min = origin;
            float2 max = origin;

            // 将扇形的边缘点加入 AABB 的计算
            float2 pointLeft = origin + dirLeft * radius;
            float2 pointRight = origin + dirRight * radius;
            min = math.min(min, math.min(pointLeft, pointRight));
            max = math.max(max, math.max(pointLeft, pointRight));

            // 如果扇形角度大于 180 度，需要考虑圆的最大外接矩形
            if (angle >= math.PI)
            {
                min = origin - new float2(radius, radius);
                max = origin + new float2(radius, radius);
            }
            else
            {
                // 检查扇形是否跨越了坐标轴，以包含可能的最大值
                float startAngle = math.atan2(dirRight.y, dirRight.x);
                float endAngle = math.atan2(dirLeft.y, dirLeft.x);

                if (IsAngleBetween(0, startAngle, endAngle))
                {
                    float2 point = origin + new float2(radius, 0);
                    min = math.min(min, point);
                    max = math.max(max, point);
                }
                if (IsAngleBetween(math.PI * 0.5f, startAngle, endAngle))
                {
                    float2 point = origin + new float2(0, radius);
                    min = math.min(min, point);
                    max = math.max(max, point);
                }
                if (IsAngleBetween(math.PI, startAngle, endAngle))
                {
                    float2 point = origin + new float2(-radius, 0);
                    min = math.min(min, point);
                    max = math.max(max, point);
                }
                if (IsAngleBetween(-math.PI * 0.5f, startAngle, endAngle))
                {
                    float2 point = origin + new float2(0, -radius);
                    min = math.min(min, point);
                    max = math.max(max, point);
                }
            }

            return new Bounds2D((min + max) * 0.5f, max - min);
        }

        private bool IsAngleBetween(float angle, float startAngle, float endAngle)
        {
            angle = NormalizeAngle(angle);
            startAngle = NormalizeAngle(startAngle);
            endAngle = NormalizeAngle(endAngle);

            if (startAngle <= endAngle)
            {
                return angle >= startAngle && angle <= endAngle;
            }
            else
            {
                return angle >= startAngle || angle <= endAngle;
            }
        }

        /// <summary>
        /// 将角度标准化到 [-π, π] 范围内。
        /// </summary>
        /// <param name="angle">需要标准化的角度。</param>
        /// <returns>标准化后的角度。</returns>
        private float NormalizeAngle(float angle)
        {
            while (angle < -math.PI)
                angle += 2f * math.PI;
            while (angle > math.PI)
                angle -= 2f * math.PI;
            return angle;
        }
        
        /// <summary>
        /// 扇形搜索
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="direction"></param>
        /// <param name="angle"></param>
        /// <param name="radius"></param>
        /// <param name="resultList"></param>
        public void SectorQuery(float2 origin, float2 direction, float angle, float radius, NativeList<T> resultList)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            // 计算扇形的包围盒
            var aabb = CalculateSectorAABB(origin, direction, angle, radius);

            // 将 AABB 限制在世界范围内
            aabb.Clamp(_data->WorldBounds);

            // 计算 AABB 覆盖的网格单元索引范围
            CalculateStartEndIterationInternal(_data, aabb, out var start, out var end);

            // 用于跟踪已处理的 itemID，避免重复
            var processedItemIDs = new NativeParallelHashSet<int>(32, Allocator.Temp);

            var hashPosition = new int2(0);

            // 预计算一些值
            float cosHalfAngle = math.cos(angle * 0.5f);
            float radiusSquared = radius * radius;

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    uint cellHash = Hash(hashPosition);

                    // 获取该网格单元中的所有 itemID
                    if (_buckets.TryGetFirstValue(cellHash, out int itemID, out var iterator))
                    {
                        do
                        {
                            // 检查是否已经处理过该 itemID
                            if (!processedItemIDs.Contains(itemID))
                            {
                                processedItemIDs.Add(itemID);

                                // 获取对应的 T 对象
                                if (_itemIDToItem.TryGetValue(itemID, out var item))
                                {
                                    // 获取对象的中心点
                                    float2 itemCenter = item.GetCenter();

                                    // 计算对象与 origin 之间的向量
                                    float2 toItem = itemCenter - origin;

                                    // 计算距离的平方
                                    float distanceSquared = math.lengthsq(toItem);

                                    // 检查距离是否在半径范围内
                                    if (distanceSquared <= radiusSquared)
                                    {
                                        // 归一化 toItem
                                        float2 toItemDir = math.normalizesafe(toItem);

                                        // 计算方向向量与 toItemDir 的点乘
                                        float dot = math.dot(direction, toItemDir);

                                        // 检查角度是否在扇形范围内
                                        if (dot >= cosHalfAngle)
                                        {
                                            resultList.Add(item);
                                        }
                                    }
                                }
                            }
                        } while (_buckets.TryGetNextValue(out itemID, ref iterator));
                    }
                }
            }
            
            // 释放临时的 HashSet
            processedItemIDs.Dispose();
        }
        
        /// <summary>
        /// 圆形搜索
        /// </summary>
        /// <param name="circleCenter"></param>
        /// <param name="radius"></param>
        /// <param name="resultList"></param>
        public void CircleQuery(float2 circleCenter, float radius, NativeList<T> resultList)
        {
            Assert.IsTrue(resultList.IsCreated);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            Bounds2D queryBounds = new Bounds2D(circleCenter, new float2(radius * 2f));
            queryBounds.Clamp(_data->WorldBounds);
            CalculateStartEndIterationInternal(_data, queryBounds, out int2 start, out int2 end);
            var hashMapUnic = new NativeHashSet<int>(64, Allocator.Temp);
            var hashPosition = new int2(0);
            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;
                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;
                    uint hash = Hash(hashPosition);
                    if (_buckets.TryGetFirstValue(hash, out var itemID, out var it))
                    {
                        do
                        {
                            hashMapUnic.Add(itemID);
                        } while (_buckets.TryGetNextValue(out itemID, ref it));
                    }
                }
            }
            ExtractItemsWithinCircleFromHashMap(hashMapUnic, circleCenter, radius, resultList);
            hashMapUnic.Dispose();
        }

        private void ExtractItemsWithinCircleFromHashMap(NativeHashSet<int> hashMapUnic, float2 sphereCenter, float radius, NativeList<T> resultList)
        {
            var itemIDs = hashMapUnic.ToNativeArray(Allocator.Temp);
            for (int i = 0; i < itemIDs.Length; ++i)
            {
                int itemID = itemIDs[i];
                if (_itemIDToBounds.TryGetValue(itemID, out var itemBounds))
                {
                    if (BoundsIntersectsCircle(itemBounds, sphereCenter, radius))
                    {
                        if (_itemIDToItem.TryGetValue(itemID, out var item))
                        {
                            resultList.Add(item);
                        }
                    }
                }
            }
            itemIDs.Dispose();
        }
        
        private bool BoundsIntersectsCircle(Bounds2D bounds, float2 sphereCenter, float radius)
        {
            float sqDistance = 0f;
            float2 min = bounds.Min;
            float2 max = bounds.Max;
            for (int i = 0; i < 2; i++)
            {
                float v = sphereCenter[i];
                if (v < min[i])
                {
                    sqDistance += (min[i] - v) * (min[i] - v);
                }
                else if (v > max[i])
                {
                    sqDistance += (v - max[i]) * (v - max[i]);
                }
            }
            return sqDistance <= radius * radius;
        }
        
        #endregion
        
        
        /// <inheritdoc />
        public void GetIndexiesVoxel(T item, NativeList<int2> results)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            var bounds = new Bounds2D(item.GetCenter(), item.GetSize());

            bounds.Clamp(_data -> WorldBounds);

            CalculateStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int2(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    results.Add(hashPosition);
                }
            }
        }
        

        /// <summary>
        /// 获取体素的位置
        /// </summary>
        /// <param name="index"></param>
        /// <param name="center"></param>
        /// <returns></returns>
        public float2 GetPositionVoxel(int2 index, bool center)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            var pos = index * _data -> CellSize + _data -> WorldBoundsMin;

            if (center)
                pos += _data -> CellSize * 0.5F;

            return pos;
        }
        
        /// <summary>
        /// 根据坐标获取体素索引
        /// </summary>
        /// <param name="position"></param>
        /// <returns></returns>
        public int2 GetIndexVoxel(float2 position)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            position -= _data -> WorldBoundsMin;

            position /= _data -> CellSize;

            return position.FloorToInt();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SpatialHash2DData
    {
        public Bounds2D WorldBounds;    //24
        public float2 WorldBoundsMin; //12

        public Bounds2D RayCastBound; //24
        public float2 CellSize;     //12

        public float2 RayOrigin;    //12
        public float2 RayDirection; //12
        public int2   CellCount;    //12

        public int  Counter; //4
        public bool HasHit;  //1
    }
    
    public interface ISpatialHashing2DItem<T> : IEquatable<T>
    {
        [Pure]
        float2 GetCenter();
        
        [Pure]
        float2 GetSize();
        
        int SpatialHashingIndex { get; set; }
    }
}