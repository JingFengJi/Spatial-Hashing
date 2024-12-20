﻿using System;
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
using SpatialHashing.Utils;
using SpatialHashing.Utils;

namespace SpatialHashing.Demo
{
    public class HashMapVisualDebug : MonoBehaviour, IRay
    {
        public Mesh instanceMesh;
        public Material instanceMaterial;
        public int subMeshIndex = 0;

        private int cachedInstanceCount = -1;
        private int cachedSubMeshIndex = -1;
        private ComputeBuffer positionBuffer;
        private ComputeBuffer argsBuffer;
        private uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
        
        private NativeArray<float4> _positionBuffer;

        void OnDisable()
        {
            if (positionBuffer != null)
                positionBuffer.Release();
            positionBuffer = null;

            if (argsBuffer != null)
                argsBuffer.Release();
            argsBuffer = null;
        }

        private FpsCounter m_FpsCounter;
        
        private void Start()
        {
            
            m_FpsCounter = new FpsCounter(0.5f);
            Random.InitState(123456789);
            argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            _positionBuffer = new NativeArray<float4>(_spawnCount, Allocator.Persistent);
            var copy = new Bounds(_worldBounds.Center, _worldBounds.Size * 4F);
            _spatialHashing = new SpatialHash<ItemTest>(copy, new float3(10F), Allocator.Persistent);

            // Ensure submesh index is in range
            if (instanceMesh != null)
                subMeshIndex = Mathf.Clamp(subMeshIndex, 0, instanceMesh.subMeshCount - 1);

            // Positions
            if (positionBuffer != null)
                positionBuffer.Release();
            positionBuffer = new ComputeBuffer(_spawnCount, 16);
            
            for (int i = 0; i < _spawnCount; i++)
            {
                float angle = Random.Range(0.0f, Mathf.PI * 2.0f);
                float distance = Random.Range(20.0f, 100.0f);
                float height = Random.Range(-20.0f, 20.0f);
                float size = Random.Range(1f, 5f);
                
                _positionBuffer[i] = new float4(Mathf.Sin(angle) * distance, height, Mathf.Cos(angle) * distance, size);
                var item = new ItemTest()
                    { ID = i, Position = new float3(_positionBuffer[i].x, _positionBuffer[i].y, _positionBuffer[i].z), Size = size};
                _spatialHashing.Add(ref item);
                _listItem.Add(item);
            }

            positionBuffer.SetData(_positionBuffer);
            instanceMaterial.SetBuffer("positionBuffer", positionBuffer);

            // Indirect args
            if (instanceMesh != null) {
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

            /// <inheritdoc />
            public void Execute()
            {
                for (int i = 0; i < ItemList.Length; i++)
                    SpatialHash.Move(ItemList[i]);
            }

            public NativeList<ItemTest>  ItemList;
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
            
            [NativeDisableParallelForRestriction]
            public NativeList<ItemTest>  ItemList;
            public SpatialHash<ItemTest>.Concurrent SpatialHash;
        }
        [BurstCompile]
        public struct RemoveItemTestJob : IJob
        {
            public void Execute()
            {
                for (int i = 0; i < ItemList.Length; i++)
                    SpatialHash.Remove(ItemList[i].SpatialHashingIndex);
            }

            public NativeList<ItemTest>  ItemList;
            public SpatialHash<ItemTest> SpatialHash;
        }

        [BurstCompile]
        public struct UpdatePosJob : IJobParallelFor
        {
            [NativeDisableContainerSafetyRestriction] public NativeList<ItemTest> ItemList;
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

        private void OnGUI()
        {
            Matrix4x4 cachedMatrix = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(2, 2, 1f));
            string title = $"FPS: {m_FpsCounter.CurrentFps:F2}";
            if (GUILayout.Button(title, GUILayout.Width(150f), GUILayout.Height(100f)))
            {
                
            }
            GUI.matrix = cachedMatrix;
        }

        private void Update()
        {
            m_FpsCounter.Update(Time.deltaTime, Time.unscaledDeltaTime);
            
            var UpdatePosJob = new UpdatePosJob()
            {
                PositionBuffer = _positionBuffer,
                Time = Time.realtimeSinceStartup,
                ItemList = _listItem
            };
            UpdatePosJob.Schedule(_positionBuffer.Length, 64).Complete();
            positionBuffer.SetData(_positionBuffer);
            instanceMaterial.SetBuffer("positionBuffer", positionBuffer);
            
            Graphics.DrawMeshInstancedIndirect(instanceMesh, subMeshIndex, instanceMaterial, new UnityEngine.Bounds(Vector3.zero, new Vector3(100.0f, 100.0f, 100.0f)), argsBuffer);
        
            //World.DefaultGameObjectInjectionWorld.EntityManager.CompleteAllTrackedJobs();

            //var itemList = new NativeList<ItemTest>(_spawnCount, Allocator.TempJob);

            // Profiler.BeginSample("SpatialHasing 1");
            // for (var i = 0; i < _listItem.Count; i++)
            // {
            //     if (math.any(_listItem[i].Position != _positionBuffer[i].xyz))
            //     {
            //         var item = _listItem[i];
            //         itemList.Add(item);
            //     }
            // }
            // Profiler.EndSample();
            
            // Profiler.BeginSample("SpatialHasing 2");
            new MoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule().Complete();
            // int length = _listItem.Length;
            // var inputDep = new JobHandle();
            // inputDep= new RemoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule(inputDep);
            // inputDep = new AddItemTestJob() { SpatialHash = _spatialHashing.ToConcurrent(), ItemList = _listItem }.Schedule(length, 64, inputDep);
            // inputDep.Complete();
            // Profiler.EndSample();
            
            // Profiler.BeginSample("SpatialHasing 3");
            // int delta = 0;
            // for (var i = 0; i < _listItem.Count; i++)
            // {
            //     if (math.any(_listItem[i].Position != (float3)_positionBuffer[i].xyz))
            //     {
            //         var item =itemList[delta++];
            //         item.Position = _positionBuffer[i].xyz;
            //         _listItem[i]  = item;
            //     }
            // }
            // Profiler.EndSample();
            // itemList.Dispose();
            
            // m_Results.Clear();
            // Profiler.BeginSample("Query");
            // _spatialHashing.Query(m_QuerryBound, m_Results);
            // Profiler.EndSample();
            
            // Profiler.BeginSample("RayCast");
            // var startRayPosition = _startRay.position;
            // var startCellBound   = new Bounds(_spatialHashing.GetPositionVoxel(_spatialHashing.GetIndexVoxel(startRayPosition), true), _spatialHashing.CellSize);
            //
            // // Gizmos.color = Color.yellow;
            // // Gizmos.DrawLine(startRayPosition, _endRay.position);
            //
            // //ItemTest hit = new ItemTest();
            // var      ray = new Ray(startRayPosition, _endRay.position - startRayPosition);
            //
            // if (_spatialHashing.Raycast(ray, out m_RayacstResult, (_endRay.position - _startRay.position).magnitude))
            // {
            //     // Gizmos.color = Color.blue;
            //     // Gizmos.DrawCube(hit.GetCenter(), hit.GetSize());
            // }
            //
            // Profiler.EndSample();
        }
        
        [SerializeField]
        private Bounds m_QuerryBound = new Bounds(5.5F, 100F);
        private NativeList<ItemTest> m_Results = new NativeList<ItemTest>(32, Allocator.Persistent);

        private ItemTest m_RayacstResult = new ItemTest();
        
        private void OnDrawGizmos()
        {
            if (Application.isPlaying == false)
                return;

            // for (int i = 0; i < _listItem.Length; i++)
            // {
            //     Gizmos.color = new Color(0F, 1F, 0F, 0.3F);
            //     Gizmos.DrawCube(_listItem[i].GetCenter(), _listItem[i].GetSize());
            // }

            if (_useQuery)
            {
                Gizmos.color = new Color(1F, 0F, 0F, 0.3F);
                Gizmos.DrawCube(m_QuerryBound.Center, m_QuerryBound.Size);
            
                for (int i = 0; i < m_Results.Length; i++)
                {
                    Gizmos.color = new Color(0F, 1F, 0F, 0.6F); 
                    Gizmos.DrawCube(m_Results[i].GetCenter(), m_Results[i].GetSize());
                }
            }
            
            if (Time.realtimeSinceStartup - _timeLastRefresh > _refreshTime)
                RefreshLinks();
            
            foreach (var l in _links)
            {
                DrawCell(l.Key);
            
                foreach (var item in l.Value)
                    DrawLink(l.Key, item);
            }

            if (_useRaycast)
            {
                var startRayPosition = _startRay.position;
                var startCellBound   = new Bounds(_spatialHashing.GetPositionVoxel(_spatialHashing.GetIndexVoxel(startRayPosition), true), _spatialHashing.CellSize);

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(startRayPosition, _endRay.position);

                //ItemTest hit = new ItemTest();
                //var      ray = new Ray(startRayPosition, _endRay.position - startRayPosition);

                //if (_spatialHashing.RayCast(ray, ref hit, (_endRay.position - _startRay.position).magnitude))
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawCube(m_RayacstResult.GetCenter(), m_RayacstResult.GetSize());
                }

                // _voxelTraversed.Clear();
                //
                // var me = this;
                // _voxelRay.RayCast(ref me, ray.origin, ray.direction, (_endRay.position - _startRay.position).magnitude);
                // Gizmos.color = new Color(0.88F, 0.6F, 0.1F, 0.4F);
                //
                // foreach (var voxel in _voxelTraversed)
                // {
                //     var position = _spatialHashing.GetPositionVoxel(voxel, true);
                //     Gizmos.DrawCube(position, _spatialHashing.CellSize);
                // }
                //
                // var rayOffsetted = new Ray(ray.origin - (Vector3)(ray.direction * _spatialHashing.CellSize), ray.direction);
                // startCellBound.GetEnterPositionAABB(rayOffsetted, 1 << 25, out var enterPoint);
                // Gizmos.color = Color.white;
                // Gizmos.DrawCube(enterPoint, Vector3.one);
                //
                // startCellBound.GetExitPosition(rayOffsetted, 1 << 25, out var exitPoint);
                // Gizmos.color = Color.white;
                // Gizmos.DrawCube(exitPoint, Vector3.one);

            }
        }

        private void RefreshLinks()
        {
            _timeLastRefresh = Time.realtimeSinceStartup;
            _links.Clear();

            var unic   = new HashSet<ItemTest>();
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

        /// <inheritdoc />
        public bool OnTraversingVoxel(int3 voxelIndex)
        {
            _voxelTraversed.Add(voxelIndex);

            return false;
        }

        /// <inheritdoc />
        public int3 GetIndexVoxel(float3 position)
        {
            return _spatialHashing.GetIndexVoxel(position);
        }

        /// <inheritdoc />
        public float3 GetPositionVoxel(int3 index, bool center)
        {
            return _spatialHashing.GetPositionVoxel(index, center);
        }

        /// <inheritdoc />
        public float3 CellSize { get { return _spatialHashing.CellSize; } }

        #endregion

        #region Variables

        [SerializeField, Range(0F, 1F)]
        private float _refreshTime = 0.2F;
        [SerializeField]
        private Bounds _worldBounds;
        [SerializeField]
        private int _spawnCount;
        [SerializeField]
        private bool _useRaycast;
        [SerializeField] private bool _useQuery;
        [SerializeField]
        private Transform _startRay;
        [SerializeField]
        private Transform _endRay;

        private NativeList<ItemTest>                   _listItem           = new NativeList<ItemTest>(0, Allocator.Persistent);
        //private List<GameObject>                 _listItemGameobject = new List<GameObject>();
        private SpatialHash<ItemTest>            _spatialHashing;
        private float                            _timeLastRefresh = -99F;
        private Dictionary<int3, List<ItemTest>> _links           = new Dictionary<int3, List<ItemTest>>();
        private List<int3>                       _voxelTraversed  = new List<int3>();
        private VoxelRay<HashMapVisualDebug>     _voxelRay        = new VoxelRay<HashMapVisualDebug>();

        #endregion

        public struct ItemTest : ISpatialHashingItem<ItemTest>, IComponentData
        {
            public int    ID;
            public float3 Position;
            public float Size;
            
            #region Implementation of IEquatable<ItemTest>

            /// <inheritdoc />
            public bool Equals(ItemTest other)
            {
                return ID == other.ID;
            }

            /// <inheritdoc />
            public override int GetHashCode()
            {
                return ID;
            }

            #region Overrides of ValueType

            /// <inheritdoc />
            public override string ToString()
            {
                return "Item " + ID;
            }

            #endregion

            #endregion

            #region Implementation of ISpatialHashingItem<ItemTest>

            /// <inheritdoc />
            public float3 GetCenter()
            {
                return Position;
            }

            /// <inheritdoc />
            public float3 GetSize()
            {
                return new float3(Size);
            }

            /// <inheritdoc />
            public int SpatialHashingIndex { get; set; }

            #endregion
        }
    }
}