using System;
using System.Diagnostics.Contracts;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;

namespace HMH.ECS.SpatialHashing
{
    /// <summary>
    /// Spatial hashing logic. Have to be assign by ref !
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public unsafe struct SpatialHash<T> : IDisposable, IRay where T : unmanaged, ISpatialHashingItem<T>
    {
        public SpatialHash(Bounds worldBounds, float3 cellSize, Allocator label)
            : this(worldBounds, cellSize, worldBounds.GetCellCount(cellSize).Mul() * 3, label)
        { }

        public SpatialHash(Bounds worldBounds, float3 cellSize, int initialSize, Allocator allocator)
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
            _data                   = (SpatialHashData*)UnsafeUtility.MallocTracked(sizeof(SpatialHashData), UnsafeUtility.AlignOf<SpatialHashData>(), allocator, 0);
            _data -> WorldBounds    = worldBounds;
            _data -> WorldBoundsMin = worldBounds.Min;
            _data -> CellSize       = cellSize;
            _data -> CellCount      = worldBounds.GetCellCount(cellSize);
            _data -> RayCastBound   = new Bounds();
            _data -> HasHit         = false;
            _data -> Counter        = 0;
            _data -> RayOrigin      = float3.zero;
            _data -> RayDirection   = float3.zero;

            _buckets            = new NativeParallelMultiHashMap<uint, int>(initialSize, allocator);
            _itemIDToBounds     = new NativeParallelHashMap<int, Bounds>(initialSize >> 1, allocator);
            _itemIDToItem       = new NativeParallelHashMap<int, T>(initialSize >> 1, allocator);
            _helpMoveHashMapOld = new NativeParallelHashSet<int3>(128, allocator);
            _helpMoveHashMapNew = new NativeParallelHashSet<int3>(128, allocator);

            _voxelRay    = new VoxelRay<SpatialHash<T>>();
            _rayHitValue = 0;
        }

        /// <inheritdoc />
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

            var bounds = new Bounds(item.GetCenter(), item.GetSize());

            bounds.Clamp(_data -> WorldBounds);

            // TODO Maintien free id to replace hashmap by array
            var itemID = ++_data -> Counter;

            item.SpatialHashingIndex = itemID;
            _itemIDToBounds.TryAdd(itemID, bounds);
            _itemIDToItem.TryAdd(itemID, item);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        AddInternal(hashPosition, itemID);
                    }
                }
            }
        }

        public void AddFast(ref T item)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

            var itemID = item.SpatialHashingIndex;
            var bounds = new Bounds(item.GetCenter(), item.GetSize());

            bounds.Clamp(_data -> WorldBounds);

            _itemIDToBounds[itemID] = bounds;
            _itemIDToItem[itemID]   = item;

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        AddInternal(hashPosition, itemID);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddInternal(int3 position, int itemID)
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

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        RemoveInternal(hashPosition, itemID);
                    }
                }
            }
        }

        /// <summary>
        /// Remove method used for move or scale an item;
        /// </summary>
        /// <param name="itemID"></param>
        public void RemoveFast(int itemID)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var success = _itemIDToBounds.TryGetValue(itemID, out var bounds);

            Assert.IsTrue(success);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        RemoveInternal(hashPosition, itemID);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void RemoveInternal(int3 voxelPosition, int itemID)
        {
            _buckets.Remove(Hash(voxelPosition), itemID);
        }

        public void Move(T item)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
            var itemID  = item.SpatialHashingIndex;
            var success = _itemIDToBounds.TryGetValue(itemID, out var oldBounds);
            Assert.IsTrue(success);

            var newBounds = new Bounds(item.GetCenter(), item.GetSize());
            newBounds.Clamp(_data -> WorldBounds);
            _itemIDToBounds.Remove(itemID);
            _itemIDToBounds.TryAdd(itemID, newBounds);
            _itemIDToItem.Remove(itemID);
            _itemIDToItem.TryAdd(itemID, item);

            _helpMoveHashMapOld.Clear();
            SetVoxelIndexForBounds(_data, oldBounds, _helpMoveHashMapOld);
            _helpMoveHashMapNew.Clear();
            SetVoxelIndexForBounds(_data, newBounds, _helpMoveHashMapNew);

            foreach (var oldVoxelPosition in _helpMoveHashMapOld)
            {
                if (_helpMoveHashMapNew.Contains(oldVoxelPosition) == false)
                {
                    RemoveInternal(oldVoxelPosition, itemID);
                }
            }

            foreach (var newVoxelPosition in _helpMoveHashMapNew)
            {
                if (_helpMoveHashMapOld.Contains(newVoxelPosition) == false)
                {
                    AddInternal(newVoxelPosition, itemID);
                }
            }
        }

        public void Resize(T item)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

            Move(item);
        }

        public void Clear()
        {
            _itemIDToBounds.Clear();
            _buckets.Clear();
            _itemIDToBounds.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void SetVoxelIndexForBounds(SpatialHashData* data, Bounds bounds, NativeParallelHashSet<int3> collection)
        {
            CalculStartEndIterationInternal(data, bounds, out var start, out var end);

            var position = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                position.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    position.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        position.z = z;

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
        }

        public void CalculStartEndIteration(Bounds bounds, out int3 start, out int3 end)
        {
            CalculStartEndIterationInternal(_data, bounds, out start, out end);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CalculStartEndIterationInternal(SpatialHashData* data, Bounds bounds, out int3 start, out int3 end)
        {
            start = ((bounds.Min - data -> WorldBoundsMin) / data -> CellSize).FloorToInt();
            end   = ((bounds.Max - data -> WorldBoundsMin) / data -> CellSize).CeilToInt();
        }

        public T GetObject(int index)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            if (_itemIDToItem.TryGetValue(index, out var value))
                return value;

            Assert.IsTrue(false);

            return default;
        }

        public void PrepareFreePlace(int count)
        {
            if (_buckets.Capacity - _buckets.Count() < count)
                _buckets.Capacity = math.ceilpow2(_buckets.Count() + count);

            if (_itemIDToBounds.Capacity - _itemIDToBounds.Count() < count)
                _itemIDToBounds.Capacity = math.ceilpow2(_itemIDToBounds.Count() + count);

            if (_itemIDToItem.Capacity - _itemIDToItem.Count() < count)
                _itemIDToItem.Capacity = math.ceilpow2(_itemIDToItem.Count() + count);
        }

        #endregion

        #region Query

        public void Query(int3 chunkIndex, NativeList<T> resultList)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            var hashMapUnic = new NativeParallelHashMap<int, byte>(64, Allocator.Temp);
            var hash        = Hash(chunkIndex);

            if (_buckets.TryGetFirstValue(hash, out var item, out var it) == false)
                return;

            do
            {
                if (hashMapUnic.TryAdd(item, 0))
                    resultList.Add(_itemIDToItem[item]);
            } while (_buckets.TryGetNextValue(out item, ref it));
        }

        /// <summary>
        /// Query system to find object in <paramref name="bounds"/>.
        /// </summary>
        public void Query(Bounds bounds, NativeList<T> resultList)
        {
            Assert.IsTrue(resultList.IsCreated);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            bounds.Clamp(_data -> WorldBounds);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashMapUnic  = new NativeHashSet<int>(64, Allocator.Temp);
            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        var hash = Hash(hashPosition);

                        if (_buckets.TryGetFirstValue(hash, out var item, out var it))
                        {
                            do
                                hashMapUnic.Add(item);
                            while (_buckets.TryGetNextValue(out item, ref it));
                        }
                    }
                }
            }

            ExtractValueFromHashMap(hashMapUnic, bounds, resultList);
        }

        public void Query(Bounds bounds, NativeList<int3> voxelIndexes)
        {
            Assert.IsTrue(voxelIndexes.IsCreated);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            bounds.Clamp(_data -> WorldBounds);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        voxelIndexes.Add(hashPosition);
                    }
                }
            }
        }

        public void Query(Bounds obbBounds, quaternion rotation, NativeList<T> resultList)
        {
            Assert.IsTrue(resultList.IsCreated);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            var bounds = TransformBounds(in obbBounds, in rotation);
            bounds.Clamp(_data -> WorldBounds);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashMapUnic = new NativeHashSet<int>(64, Allocator.Temp);

            var hashPosition = new int3(0F);

            //add offset for simplify logic and allowed pruning
            obbBounds.Size += _data -> CellSize * 1F;

            var inverseRotation = math.inverse(rotation);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        var pos = GetPositionVoxel(hashPosition, true);

                        if (obbBounds.RayCastOBBFast(pos - new float3(_data -> CellSize.x * 0.5F, 0F, 0F), Right, inverseRotation, _data -> CellSize.x) ||
                            obbBounds.RayCastOBBFast(pos - new float3(0F, _data -> CellSize.y * 0.5F, 0F), Up, inverseRotation, _data -> CellSize.y) ||
                            obbBounds.RayCastOBBFast(pos - new float3(0F, 0F, _data -> CellSize.z * 0.5F), Forward, inverseRotation, _data -> CellSize.z))
                        {
                            var hash = Hash(hashPosition);

                            if (_buckets.TryGetFirstValue(hash, out var item, out var it))
                            {
                                do
                                    hashMapUnic.Add(item);
                                while (_buckets.TryGetNextValue(out item, ref it));
                            }
                        }
                    }
                }
            }

            ExtractValueFromHashMap(hashMapUnic, bounds, resultList);
        }

        public void Query(Bounds obbBounds, quaternion rotation, NativeList<int3> voxelIndexes)
        {
            Assert.IsTrue(voxelIndexes.IsCreated);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            var bounds = TransformBounds(in obbBounds, in rotation);
            bounds.Clamp(_data -> WorldBounds);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            //add offset for simplify logic and allowed pruning
            obbBounds.Size += _data -> CellSize * 1F;

            var inverseRotation = math.inverse(rotation);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        var pos = GetPositionVoxel(hashPosition, true);

                        if (obbBounds.RayCastOBBFast(pos - new float3(_data -> CellSize.x * 0.5F, 0F, 0F), Right, inverseRotation, _data -> CellSize.x) ||
                            obbBounds.RayCastOBBFast(pos - new float3(0F, _data -> CellSize.y * 0.5F, 0F), Up, inverseRotation, _data -> CellSize.y) ||
                            obbBounds.RayCastOBBFast(pos - new float3(0F, 0F, _data -> CellSize.z * 0.5F), Forward, inverseRotation, _data -> CellSize.z))
                            voxelIndexes.Add(hashPosition);
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Bounds TransformBounds(in Bounds boundTarget, in quaternion rotation)
        {
            var b = new Bounds(boundTarget.Center, math.abs(math.mul(rotation, boundTarget.Size)));
            return b;
        }

        private void ExtractValueFromHashMap(NativeHashSet<int> hashMapUnic, Bounds bounds, NativeList<T> resultList)
        {
            using var iterator = hashMapUnic.GetEnumerator();
            while (iterator.MoveNext())
            {
                var itemID = iterator.Current;

                _itemIDToBounds.TryGetValue(itemID, out var b);

                if (bounds.Intersects(b) == false)
                    continue;

                resultList.Add(_itemIDToItem[itemID]);
            }
        }
        public int QueryCount(int3 chunkIndex)
        {
            var hash = Hash(chunkIndex);

            int counter = 0;

            if (_buckets.TryGetFirstValue(hash, out _, out var it))
            {
                ++counter;

                while (_buckets.TryGetNextValue(out _, ref it))
                    ++counter;
            }

            return counter;
        }

        #endregion
        
        #region Raycast
        
        public bool Raycast(Ray ray)
        {
            return Raycast(ray.origin, ray.direction);
        }
        
        public bool Raycast(Ray ray, float maxDistance)
        {
            return Raycast(ray.origin, ray.direction, maxDistance);
        }
        
        public bool Raycast(float3 origin, float3 direction)
        {
            return Raycast(origin, direction, float.PositiveInfinity);
        }
        
        public bool Raycast(float3 origin, float3 direction, float maxDistance)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            float magnitude = math.length(direction);
            if(magnitude <= 1.401298464324817E-45)
                return false;
            float3 normalizedDirection = direction / magnitude;
            _data -> HasHit = false;
            _data -> RayOrigin    = origin;
            _data -> RayDirection = normalizedDirection;
            _voxelRay.RayCast(ref this, _data -> RayOrigin, _data -> RayDirection, maxDistance);
            return _data -> HasHit;
        }

        public bool Raycast(Ray ray, out T item)
        {
            return Raycast(ray.origin, ray.direction, out item, float.PositiveInfinity);
        }
        
        public bool Raycast(Ray ray, out T item, float maxDistance)
        {
            return Raycast(ray.origin, ray.direction, out item, maxDistance);
        }
        
        public bool Raycast(float3 origin, float3 direction, out T item)
        {
            return Raycast(origin, direction, out item, float.PositiveInfinity);
        }
        
        public bool Raycast(float3 origin, float3 direction, out T item, float maxDistance)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            float magnitude = math.length(direction);
            if (magnitude <= 1.401298464324817E-45)
            {
                item = default;
                return false;
            }
            float3 normalizedDirection = direction / magnitude;
            _data -> HasHit = false;
            _data -> RayOrigin    = origin;
            _data -> RayDirection = direction;
            _voxelRay.RayCast(ref this, _data -> RayOrigin, _data -> RayDirection, maxDistance);
            if (_data->HasHit == false)
            {
                item = default;
                return false;
            }
            item = _itemIDToItem[_rayHitValue];
            return true;
        }

        #endregion

        #region Linecast

        public bool Linecast(float3 start, float3 end)
        {
            float3 direction = end - start;
            return Raycast(start, direction, math.length(direction));
        }
        
        public bool Linecast(float3 start, float3 end, out T item)
        {
            float3 direction = end - start;
            return Raycast(start, direction, out item, math.length(direction));
        }
        
        #endregion

        #region Boxcast

        public bool BoxCast(float3 center, float3 halfExtents, float3 direction)
        {
            return BoxCast(center, halfExtents, direction, Quaternion.identity, float.PositiveInfinity);
        }
        
        public bool BoxCast(float3 center, float3 halfExtents, float3 direction, Quaternion orientation)
        {
            return BoxCast(center, halfExtents, direction, orientation, float.PositiveInfinity);
        }
        
        public bool BoxCast(float3 center, float3 halfExtents, float3 direction, Quaternion orientation, float maxDistance)
        {
            //TODO:
            return false;
            
        }


        public bool BoxCast(float3 center, float3 halfExtents, float3 direction, out T hit, Quaternion orientation, float maxDistance)
        {
            //TODO:
            hit = default;
            return false;
        }

        public bool BoxCast(float3 center, float3 halfExtents, float3 direction, out T hit, Quaternion orientation)
        {
            return BoxCast(center, halfExtents, direction, out hit, orientation, float.PositiveInfinity);
        }

        public bool BoxCast(float3 center, float3 halfExtents, float3 direction, out T hit)
        {
            return BoxCast(center, halfExtents, direction, out hit, Quaternion.identity);
        }

        #endregion

        #region Capsulecast

        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, float maxDistance)
        {
            //TODO:
            return false;
        }
        
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction)
        {
            return CapsuleCast(point1, point2, radius, direction, float.PositiveInfinity);
        }
        
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, out T hit, float maxDistance)
        {
            //TODO:
            hit = default;
            return false;
        }
        
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, out T hit)
        {
            return CapsuleCast(point1, point2, radius, direction, out hit, float.PositiveInfinity);
        }
        
        #endregion
        
        #region Spherecast
        
        private bool SphereAABBIntersect(float3 sphereCenter, float sphereRadius, Bounds aabb, out float distance)
        {
            // 计算 AABB 到球体中心的最近点
            float3 closestPoint = math.clamp(sphereCenter, aabb.Min, aabb.Max);

            // 计算距离平方
            float distSq = math.lengthsq(sphereCenter - closestPoint);

            // 比较距离平方和半径平方
            bool intersects = distSq <= (sphereRadius * sphereRadius);

            distance = intersects ? math.sqrt(distSq) : float.PositiveInfinity;

            return intersects;
        }

        public bool SphereCastAll(float3 origin, float3 direction, float radius, NativeList<T> hitInfo, float maxDistance = float.PositiveInfinity)
        {
            bool hasHit = false;
            // 归一化方向
            direction = math.normalize(direction);
            // 初始化 Voxel Raycasting
            VoxelRayIterator voxelRay = new VoxelRayIterator(origin, direction, _data->CellSize, _data->WorldBoundsMin, _data->CellCount);
            while (voxelRay.MoveNext(maxDistance))
            {
                int3 currentVoxel = voxelRay.Current;
                // 检查体素是否在世界范围内
                if (math.any(currentVoxel < 0) || math.any(currentVoxel >= _data->CellCount))
                    continue;
                uint hash = Hash(currentVoxel);
                float3 voxelCenterPos = GetPositionVoxel(currentVoxel, true);
                // 获取该体素中的对象
                if (_buckets.TryGetFirstValue(hash, out var itemID, out var it))
                {
                    do
                    {
                        // 获取对象的包围盒
                        if (_itemIDToBounds.TryGetValue(itemID, out var bounds))
                        {
                            // 检查球体是否与包围盒相交
                            if (SphereAABBSweep(origin, radius, direction, maxDistance, bounds, out float hitDistance))
                            {
                                hasHit = true;
                                // 获取对象实例
                                if (_itemIDToItem.TryGetValue(itemID, out var item))
                                {
                                    hitInfo.Add(item);
                                }
                            }
                        }
                    } while (_buckets.TryGetNextValue(out itemID, ref it));
                }
            }
            return hasHit;
        }
        
        public bool SphereCast(float3 origin, float3 direction, float radius, out T hitInfo, float maxDistance = float.PositiveInfinity)
        {
            hitInfo = default;
            bool hasHit = false;
            float closestHitDistance = maxDistance;
            // 归一化方向
            direction = math.normalize(direction);
            // 初始化 Voxel Raycasting
            VoxelRayIterator voxelRay = new VoxelRayIterator(origin, direction, _data->CellSize, _data->WorldBoundsMin, _data->CellCount);
            while (voxelRay.MoveNext(maxDistance))
            {
                int3 currentVoxel = voxelRay.Current;
                // 检查体素是否在世界范围内
                if (math.any(currentVoxel < 0) || math.any(currentVoxel >= _data->CellCount))
                    continue;
                uint hash = Hash(currentVoxel);
                float3 voxelCenterPos = GetPositionVoxel(currentVoxel, true);
                // 获取该体素中的对象
                if (_buckets.TryGetFirstValue(hash, out var itemID, out var it))
                {
                    do
                    {
                        // 获取对象的包围盒
                        if (_itemIDToBounds.TryGetValue(itemID, out var bounds))
                        {
                            // 检查球体是否与包围盒相交
                            if (SphereAABBSweep(origin, radius, direction, closestHitDistance, bounds, out float hitDistance))
                            {
                                if (hitDistance < closestHitDistance)
                                {
                                    closestHitDistance = hitDistance;
                                    hasHit = true;
                                    // 获取对象实例
                                    if (_itemIDToItem.TryGetValue(itemID, out var item))
                                    {
                                        hitInfo = item;
                                    }
                                }
                            }
                        }
                    } while (_buckets.TryGetNextValue(out itemID, ref it));
                }
            }
            return hasHit;
        }
        
        private bool SphereAABBSweep(float3 sphereOrigin, float sphereRadius, float3 direction, float maxDistance, Bounds bounds, out float hitDistance)
        {
            hitDistance = 0f;
            direction = math.normalize(direction);

            // 扩展 AABB，使其考虑球体的半径
            Bounds expandedBounds = bounds;
            expandedBounds.Expand(sphereRadius * 2f); // 扩展大小为球体直径
            
            // 2. 执行射线与扩展后的 AABB 的相交测试
            float3 enterPoint;
            bool hasIntersection = expandedBounds.GetEnterPositionAABB(sphereOrigin, direction, maxDistance, out enterPoint);

            if (hasIntersection)
            {
                // 3. 计算碰撞距离
                hitDistance = math.length(enterPoint - sphereOrigin);

                // 判断碰撞距离是否在最大距离内
                if (hitDistance <= maxDistance)
                {
                    return true;
                }
            }

            return false;
        }


        public bool SphereCast(Vector3 origin, float radius, Vector3 direction, float maxDistance)
        {
            //TODO:
            return false;
        }
        public bool SphereCast(Vector3 origin, float radius, Vector3 direction)
        {
            return SphereCast(origin, radius, direction, float.PositiveInfinity);
        }
        
        public bool SphereCast(Ray ray, float radius, float maxDistance)
        {
            return SphereCast(ray.origin, radius, ray.direction, maxDistance);
        }

        public bool SphereCast(Ray ray, float radius)
        {
            return SphereCast(ray, radius, float.PositiveInfinity);
        }
        
        
        public bool SphereCast(Vector3 origin, float radius, Vector3 direction, out T hitInfo, float maxDistance)
        {
            //TODO:
            hitInfo = default;
            return false;
        }
        
        public bool SphereCast(Vector3 origin, float radius, Vector3 direction, out T hitInfo)
        {
            return SphereCast(origin, radius, direction, out hitInfo, float.PositiveInfinity);
        }
        
        public bool SphereCast(Ray ray, float radius, out T hitInfo, float maxDistance)
        {
            return SphereCast(ray.origin, radius, ray.direction, out hitInfo, maxDistance);
        }

        public bool SphereCast(Ray ray, float radius, out T hitInfo)
        {
            return SphereCast(ray, radius, out hitInfo, float.PositiveInfinity);
        }
        
        #endregion
        
        #region Hashing

        public static uint Hash(int3 cellIndex)
        {
            return math.hash(cellIndex);
        }

        private int GetCellCountFromSize(float3 size)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif

            var deltaSize = size / _data -> CellSize;

            return (int)math.ceil(deltaSize.x) * (int)math.ceil(deltaSize.y) * (int)math.ceil(deltaSize.z);
        }

        public float3 GetPositionVoxel(int3 index, bool center)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            var pos = index * _data -> CellSize + _data -> WorldBoundsMin;

            if (center)
                pos += _data -> CellSize * 0.5F;

            return pos;
        }

        public int3 GetIndexVoxel(float3 position)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            position -= _data -> WorldBoundsMin;

            position /= _data -> CellSize;

            return position.FloorToInt();
        }

        #endregion

        #region Implementation of IRay

        /// <inheritdoc />
        public bool OnTraversingVoxel(int3 voxelIndex)
        {
            if (math.any(voxelIndex > _data -> CellCount)) //if voxel still in world
                return true;

            var hash = Hash(voxelIndex);

            if (_buckets.TryGetFirstValue(hash, out var itemID, out var it))
                do
                {
                    _itemIDToBounds.TryGetValue(itemID, out var b);

                    if (b.GetEnterPositionAABB(_data -> RayOrigin, _data -> RayDirection, 1 << 25, out _) == false)
                        continue;

                    _data -> HasHit = true;
                    _rayHitValue    = itemID;

                    return true;
                } while (_buckets.TryGetNextValue(out itemID, ref it));

            return false;
        }

        /// <inheritdoc />
        public void GetIndexiesVoxel(T item, NativeList<int3> results)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            var bounds = new Bounds(item.GetCenter(), item.GetSize());

            bounds.Clamp(_data -> WorldBounds);

            CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

            var hashPosition = new int3(0F);

            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;

                    for (int z = start.z; z < end.z; ++z)
                    {
                        hashPosition.z = z;

                        results.Add(hashPosition);
                    }
                }
            }
        }

        #endregion

        public Concurrent ToConcurrent()
        {
            return new Concurrent
            {
                _data           = _data,
                _itemIDToBounds = _itemIDToBounds.AsParallelWriter(),
                _itemIDToItem   = _itemIDToItem.AsParallelWriter(),
                _buckets        = _buckets.AsParallelWriter(),

#if ENABLE_UNITY_COLLECTIONS_CHECKS
                m_Safety = m_Safety
#endif
            };
        }

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
        private SpatialHashData* _data;
        private NativeParallelMultiHashMap<uint, int> _buckets;        //4
        private NativeParallelHashMap<int, Bounds>    _itemIDToBounds; //4
        private NativeParallelHashMap<int, T>         _itemIDToItem;   //4

        private NativeParallelHashSet<int3>      _helpMoveHashMapOld;
        private NativeParallelHashSet<int3>      _helpMoveHashMapNew;
        private VoxelRay<SpatialHash<T>> _voxelRay;
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

        public float3 CellSize
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> CellSize;
            }
        }

        public Bounds WorldBounds
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> WorldBounds;
            }
        }
        
        public float3 WorldBoundsMin
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
                return _data -> WorldBoundsMin;
            }
        }

        public int3 CellCount
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
        public Bounds DebugRayCastBounds => _data -> RayCastBound;
        public VoxelRay<SpatialHash<T>> DebugVoxelRay => _voxelRay;
#endif

        #endregion

        public struct Concurrent
        {
            public bool TryAdd(ref T item)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

                var bounds = new Bounds(item.GetCenter(), item.GetSize());

                bounds.Clamp(_data -> WorldBounds);

                var itemID = Interlocked.Increment(ref _data -> Counter);
                item.SpatialHashingIndex = itemID;

                if (_itemIDToBounds.TryAdd(itemID, bounds) == false || _itemIDToItem.TryAdd(itemID, item) == false)
                    return false;

                CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

                var hashPosition = new int3(0F);

                for (int x = start.x; x < end.x; ++x)
                {
                    hashPosition.x = x;

                    for (int y = start.y; y < end.y; ++y)
                    {
                        hashPosition.y = y;

                        for (int z = start.z; z < end.z; ++z)
                        {
                            hashPosition.z = z;

                            var hash = Hash(hashPosition);
                            _buckets.Add(hash, itemID);
                        }
                    }
                }

                return true;
            }

            /// <summary>
            /// Add fast after a remove for moving or scaling item
            /// <para>DOESN'T WORK YET WAIT UNITY OVERRIDE VALUE IN HASHMAP</para>
            /// </summary>
            /// <param name="item"></param>
            /// <returns></returns>
            public void AddFast(in T item)
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif

                var itemID = item.SpatialHashingIndex;
                var bounds = new Bounds(item.GetCenter(), item.GetSize());

                bounds.Clamp(_data -> WorldBounds);

                //TODO Replace with Override
                if (_itemIDToBounds.TryAdd(itemID, bounds) == false || _itemIDToItem.TryAdd(itemID, item) == false)
                    return;

                CalculStartEndIterationInternal(_data, bounds, out var start, out var end);

                var hashPosition = new int3(0F);

                for (int x = start.x; x < end.x; ++x)
                {
                    hashPosition.x = x;

                    for (int y = start.y; y < end.y; ++y)
                    {
                        hashPosition.y = y;

                        for (int z = start.z; z < end.z; ++z)
                        {
                            hashPosition.z = z;

                            var hash = Hash(hashPosition);
                            _buckets.Add(hash, itemID);
                        }
                    }
                }
            }

            #region Variables

            [NativeDisableUnsafePtrRestriction]
            internal SpatialHashData* _data;

            internal NativeParallelMultiHashMap<uint, int>.ParallelWriter _buckets;        //4
            internal NativeParallelHashMap<int, Bounds>.ParallelWriter    _itemIDToBounds; //4
            internal NativeParallelHashMap<int, T>.ParallelWriter         _itemIDToItem;   //4

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            internal AtomicSafetyHandle m_Safety;
#endif

            [NativeSetThreadIndex]
            #pragma warning disable 649
            internal int _threadIndex;
            #pragma warning restore 649

            #endregion
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SpatialHashData
    {
        public Bounds WorldBounds;    //24
        public float3 WorldBoundsMin; //12

        public Bounds RayCastBound; //24
        public float3 CellSize;     //12

        public float3 RayOrigin;    //12
        public float3 RayDirection; //12
        public int3   CellCount;    //12

        public int  Counter; //4
        public bool HasHit;  //1
    }

    public interface ISpatialHashingItem<T> : IEquatable<T>
    {
        [Pure]
        float3 GetCenter();

        [Pure]
        float3 GetSize();

        int SpatialHashingIndex { get; set; }
    }
}