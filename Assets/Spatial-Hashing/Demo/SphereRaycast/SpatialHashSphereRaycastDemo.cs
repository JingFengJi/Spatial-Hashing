using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using Random = UnityEngine.Random;

namespace HMH.ECS.SpatialHashing.Debug
{
    public class SpatialHashSphereRaycastDemo : MonoBehaviour, IRay
    {
        [SerializeField, Range(0F, 1F)] private float _refreshTime = 0.2F;
        [SerializeField] private Bounds _worldBounds;
        [SerializeField] private int _spawnCount;
        [SerializeField] private Mesh instanceMesh;
        [SerializeField] private Material instanceMaterial;
        [SerializeField] private bool _Move;
        [SerializeField] private bool _DrawSpatialHashBoundGizmos;

        [SerializeField] private bool _DrawLinkGizmos;
        [SerializeField] private bool _DrawSpatialHashCells;
        [SerializeField] private Transform _startRay;
        [SerializeField] private Transform _endRay;
        [SerializeField] private float _SphereRadius = 1F;
        
        private NativeList<ItemTest> _listItem;
        private SpatialHash<ItemTest> _spatialHashing;
        private float _timeLastRefresh = -99F;
        private Dictionary<int3, List<ItemTest>> _links = new Dictionary<int3, List<ItemTest>>();
        private List<int3> _voxelTraversed = new List<int3>();
        private VoxelRay<HashMapVisualDebug> _voxelRay = new VoxelRay<HashMapVisualDebug>();
        
        private int subMeshIndex = 0;

        private int cachedInstanceCount = -1;
        private int cachedSubMeshIndex = -1;
        private ComputeBuffer positionBuffer;
        private ComputeBuffer argsBuffer;
        private uint[] args = new uint[5] { 0, 0, 0, 0, 0 };

        private NativeArray<float4> _positionBuffer;
        
        private ItemTest m_RayacstResult = new ItemTest();
        
        void OnDisable()
        {
            if (positionBuffer != null)
            {
                positionBuffer.Release();
            }

            positionBuffer = null;

            if (argsBuffer != null)
            {
                argsBuffer.Release();
            }

            argsBuffer = null;
            
            _listItem.Clear();
            _listItem.Dispose();
            _positionBuffer.Dispose();
            _spatialHashing.Dispose();
            _SphereCastAllResult.Clear();
            _SphereCastAllResult.Dispose();
        }
        
        private void Start()
        {
            Random.InitState(123456789);
            argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            _positionBuffer = new NativeArray<float4>(_spawnCount, Allocator.Persistent);
            _listItem = new NativeList<ItemTest>(_spawnCount, Allocator.Persistent);
            var copy = new Bounds(_worldBounds.Center, _worldBounds.Size);
            _spatialHashing = new SpatialHash<ItemTest>(copy, new float3(10F), Allocator.Persistent);
            _SphereCastAllResult = new NativeList<ItemTest>(64, Allocator.Persistent);
            
            if (instanceMesh != null)
            {
                subMeshIndex = Mathf.Clamp(subMeshIndex, 0, instanceMesh.subMeshCount - 1);
            }
            
            positionBuffer = new ComputeBuffer(_spawnCount, 16);

            for (int i = 0; i < _spawnCount; i++)
            {
                float size = 1;
                Vector3 randomPos = Random.insideUnitSphere * 100;
                _positionBuffer[i] = new float4(randomPos, size);
                var item = new ItemTest()
                {
                    ID = i, Position = new float3(_positionBuffer[i].x, _positionBuffer[i].y, _positionBuffer[i].z),
                    Size = size
                };
                _spatialHashing.Add(ref item);
                _listItem.Add(item);
            }

            positionBuffer.SetData(_positionBuffer);
            instanceMaterial.SetBuffer("positionBuffer", positionBuffer);
            
            if (instanceMesh != null)
            {
                args[0] = (uint)instanceMesh.GetIndexCount(subMeshIndex);
                args[1] = (uint)_spawnCount;
                args[2] = (uint)instanceMesh.GetIndexStart(subMeshIndex);
                args[3] = (uint)instanceMesh.GetBaseVertex(subMeshIndex);
            }
            else
            {
                args[0] = args[1] = args[2] = args[3] = 0;
            }

            argsBuffer.SetData(args);

            cachedInstanceCount = _spawnCount;
            cachedSubMeshIndex = subMeshIndex;
        }

        [BurstCompile]
        public struct MoveItemTestJob : IJob
        {
            public void Execute()
            {
                for (int i = 0; i < ItemList.Length; i++)
                    SpatialHash.Move(ItemList[i]);
            }

            public NativeList<ItemTest> ItemList;
            public SpatialHash<ItemTest> SpatialHash;
        }

        [BurstCompile]
        public struct AddItemTestJob : IJobParallelFor
        {
            public void Execute(int index)
            {
                var item = ItemList[index];
                SpatialHash.TryAdd(ref item);
                ItemList[index] = item;
            }

            [NativeDisableParallelForRestriction] public NativeList<ItemTest> ItemList;
            public SpatialHash<ItemTest>.Concurrent SpatialHash;
        }

        [BurstCompile]
        public struct RemoveItemTestJob : IJob
        {
            public void Execute()
            {
                for (int i = 0; i < ItemList.Length; i++)
                {
                    SpatialHash.Remove(ItemList[i].SpatialHashingIndex);
                }
            }

            public NativeList<ItemTest> ItemList;
            public SpatialHash<ItemTest> SpatialHash;
        }

        [BurstCompile]
        public struct UpdatePosJob : IJobParallelFor
        {
            [NativeDisableContainerSafetyRestriction]
            public NativeList<ItemTest> ItemList;

            public NativeArray<float4> PositionBuffer;
            [ReadOnly] public float Time;

            public void Execute(int index)
            {
                ref float4 pos = ref PositionBuffer.GetElementAsRef(index);
                pos += new float4(math.sin(Time), 0, math.cos(Time), 0);
                ref ItemTest item = ref ItemList.ElementAt(index);
                item.Position = new float3(pos.x, pos.y, pos.z);
            }
        }

        private void Update()
        {
            Profiler.BeginSample("UpdatePosJob");
            if (_Move)
            {
                var updatePosJob = new UpdatePosJob()
                {
                    PositionBuffer = _positionBuffer,
                    Time = Time.realtimeSinceStartup,
                    ItemList = _listItem
                };
                updatePosJob.Schedule(_positionBuffer.Length, 64).Complete();
                
                positionBuffer.SetData(_positionBuffer);
                instanceMaterial.SetBuffer("positionBuffer", positionBuffer);
            }
            Profiler.EndSample();
            
            Graphics.DrawMeshInstancedIndirect(instanceMesh, subMeshIndex, instanceMaterial, new UnityEngine.Bounds(Vector3.zero, new Vector3(100.0f, 100.0f, 100.0f)), argsBuffer);
            
            Profiler.BeginSample("SpatialHash_Remove_Add");
            //new MoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule().Complete();
            // int length = _listItem.Length;
            // var inputDep = new JobHandle();
            // inputDep= new RemoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule(inputDep);
            // inputDep = new AddItemTestJob() { SpatialHash = _spatialHashing.ToConcurrent(), ItemList = _listItem }.Schedule(length, 64, inputDep);
            // inputDep.Complete();
            Profiler.EndSample();
            
            Profiler.BeginSample("RayCast");
            
            // m_RayacstResult = new ItemTest();
            // if (_spatialHashing.SphereCast(_startRay.position, _endRay.position - _startRay.position,_SphereRadius, out m_RayacstResult, (_endRay.position - _startRay.position).magnitude))
            // {
            //     //UnityEngine.Debug.Log("Hit");   
            // }
            // else
            // {
            //     //UnityEngine.Debug.Log("No Hit");
            // }
            
            _SphereCastAllResult.Clear();
            if (_spatialHashing.SphereCastAll(_startRay.position, _endRay.position - _startRay.position,_SphereRadius, _SphereCastAllResult, (_endRay.position - _startRay.position).magnitude))
            {
                //UnityEngine.Debug.Log("Hit");   
            }
            else
            {
                //UnityEngine.Debug.Log("No Hit");
            }
            
            Profiler.EndSample();
        }

        public Mesh cylinderMesh;

        private NativeList<ItemTest> _SphereCastAllResult;
        
        private void OnDrawGizmos()
        {
            if (Application.isPlaying == false)
                return;
            
            if (_DrawSpatialHashBoundGizmos)
            {
                Gizmos.color = new Color(0F, 1F, 0F, 0.1F);
                Gizmos.DrawCube(_spatialHashing.WorldBounds.Center, _spatialHashing.WorldBounds.Size);
            }
            if (_DrawLinkGizmos)
            {
                if (Time.realtimeSinceStartup - _timeLastRefresh > _refreshTime)
                    RefreshLinks();
                
                foreach (var l in _links)
                {
                    DrawCell(l.Key);
            
                    foreach (var item in l.Value)
                        DrawLink(l.Key, item);
                }
            }

            if (_DrawSpatialHashCells)
            {
                float3 cellSize = _spatialHashing.CellSize;
                float3 worldBoundsMin = _spatialHashing.WorldBoundsMin;
                int3 cellCount = _spatialHashing.CellCount;
                for (int i = 0; i < cellCount.x; i++)
                {
                    for (int j = 0; j < cellCount.y; j++)
                    {
                        for (int k = 0; k < cellCount.z; k++)
                        {
                            int3 cellIndex = new int3(i, j, k);
                            float3 cellCenter = worldBoundsMin + (cellIndex + new float3(0.5f,0.5f,0.5f)) * cellSize;
                        
                            // 绘制格子的边界框
                            Gizmos.DrawWireCube(cellCenter, cellSize);
                        }
                    }
                }
            }
            
            // NativeList<ItemTest> items = new NativeList<ItemTest>(Allocator.Temp);
            // _spatialHashing.Query(new int3(10,10,10), items);
            // for (int i = 0; i < items.Length; i++)
            // {
            //     Gizmos.color = new Color(1, 0, 0, 0.2f);
            //     Gizmos.DrawWireCube(items[i].Position, new Vector3(items[i].Size,items[i].Size,items[i].Size) );
            // }
            // items.Dispose();
            
            Gizmos.color = new Color(1, 1, 0, 0.2f);
            
            Vector3 position = (_startRay.position + _endRay.position) / 2f; // 圆柱体的中心位置
            Vector3 direction = _endRay.position - _startRay.position;
            float height = direction.magnitude; // 圆柱体的高度
            Quaternion rotation = Quaternion.FromToRotation(Vector3.up, direction.normalized);
            Vector3 scale = new Vector3(_SphereRadius * 2, height / 2f, _SphereRadius * 2); // 注意 Y 轴缩放为高度的一半
            Matrix4x4 oldMatrix = Gizmos.matrix;
            Gizmos.matrix = Matrix4x4.TRS(position, rotation, scale);
            Gizmos.DrawMesh(cylinderMesh);
            Gizmos.matrix = oldMatrix;
            
            Gizmos.color = Color.blue;
            Gizmos.DrawCube(m_RayacstResult.GetCenter(), m_RayacstResult.GetSize());

            for (int i = 0; i < _SphereCastAllResult.Length; i++)
            {
                Gizmos.DrawCube(_SphereCastAllResult[i].GetCenter(), _SphereCastAllResult[i].GetSize());
            }
        }

        private void RefreshLinks()
        {
            _timeLastRefresh = Time.realtimeSinceStartup;
            _links.Clear();

            var unic = new HashSet<ItemTest>();
            var values = _spatialHashing.DebugBuckets.GetValueArray(Allocator.TempJob);

            foreach (var itemTest in values)
                unic.Add(_spatialHashing.DebugIDToItem[itemTest]);
            values.Dispose();

            var list = new NativeList<int3>(Allocator.Temp);

            foreach (var item in unic)
            {
                list.Clear();
                _spatialHashing.GetIndexiesVoxel(item, list);

                for (int i = 0; i < list.Length; i++)
                {
                    var index = list[i];

                    if (_links.TryGetValue(index, out var l) == false)
                    {
                        l = new List<ItemTest>();
                        _links.Add(index, l);
                    }

                    l.Add(item);
                }
            }

            list.Dispose();
        }

        private void DrawCell(int3 index)
        {
            var position = _spatialHashing.GetPositionVoxel(index, true);
            Gizmos.color = new Color(0F, 1F, 0F, 0.3F);
            Gizmos.DrawCube(position, _spatialHashing.CellSize);
            Gizmos.color = Color.black;
            Gizmos.DrawWireCube(position, _spatialHashing.CellSize);
        }

        private void DrawLink(int3 cellIndex, ItemTest target)
        {
            var position = _spatialHashing.GetPositionVoxel(cellIndex, true);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(position, target.Position);
        }

        #region Implementation of IRay<ItemTest>
        
        public bool OnTraversingVoxel(int3 voxelIndex)
        {
            _voxelTraversed.Add(voxelIndex);

            return false;
        }
        
        public int3 GetIndexVoxel(float3 position)
        {
            return _spatialHashing.GetIndexVoxel(position);
        }
        
        public float3 GetPositionVoxel(int3 index, bool center)
        {
            return _spatialHashing.GetPositionVoxel(index, center);
        }
        
        public float3 CellSize
        {
            get { return _spatialHashing.CellSize; }
        }

        #endregion
  }
}