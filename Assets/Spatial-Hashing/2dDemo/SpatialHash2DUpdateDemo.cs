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
    public class SpatialHash2DUpdateDemo : MonoBehaviour, IRay2D
    {
        [SerializeField, Range(0F, 1F)] private float _refreshTime = 0.2F;
        [SerializeField] private Bounds2D _worldBounds;
        [SerializeField] private int _spawnCount;
        [SerializeField] private Mesh instanceMesh;
        [SerializeField] private Material instanceMaterial;
        [SerializeField] private bool _Move;
        [SerializeField] private bool _DrawLinkGizmos;
        [SerializeField] private bool _DrawQueryResultGizmos;
        
        private NativeList<ItemTest2D> _listItem;
        private SpatialHash2D<ItemTest2D> _spatialHashing;
        private float _timeLastRefresh = -99F;
        private Dictionary<int2, List<ItemTest2D>> _links = new Dictionary<int2, List<ItemTest2D>>();
        private List<int2> _voxelTraversed = new List<int2>();
        private VoxelRay<HashMapVisualDebug> _voxelRay = new VoxelRay<HashMapVisualDebug>();
        
        private int subMeshIndex = 0;

        private int cachedInstanceCount = -1;
        private int cachedSubMeshIndex = -1;
        private ComputeBuffer positionBuffer;
        private ComputeBuffer argsBuffer;
        private uint[] args = new uint[5] { 0, 0, 0, 0, 0 };

        private NativeArray<float4> _positionBuffer;

        
        [SerializeField] private Transform m_CircleQueryTransform;
        [SerializeField] private float m_CircleQueryRadius = 10.0f;
        private NativeList<ItemTest2D> m_QueryResults;
        
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
            m_QueryResults.Clear();
            m_QueryResults.Dispose();
        }


        private void Start()
        {
            Random.InitState(123456789);
            argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            _positionBuffer = new NativeArray<float4>(_spawnCount, Allocator.Persistent);
            _listItem = new NativeList<ItemTest2D>(_spawnCount, Allocator.Persistent);
            m_QueryResults = new NativeList<ItemTest2D>(32, Allocator.Persistent);
            var copy = new Bounds2D(_worldBounds.Center, _worldBounds.Size);
            _spatialHashing = new SpatialHash2D<ItemTest2D>(copy, new float2(10F), Allocator.Persistent);
            
            if (instanceMesh != null)
            {
                subMeshIndex = Mathf.Clamp(subMeshIndex, 0, instanceMesh.subMeshCount - 1);
            }
            
            positionBuffer = new ComputeBuffer(_spawnCount, 16);

            for (int i = 0; i < _spawnCount; i++)
            {
                float size = 1;

                float2 position = Random.insideUnitCircle * 100;
                _positionBuffer[i] = new float4(position.x, position.y,0, size);
                var item = new ItemTest2D()
                {
                    ID = i, Position = new float2(_positionBuffer[i].x, _positionBuffer[i].y),
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

            public NativeList<ItemTest2D> ItemList;
            public SpatialHash2D<ItemTest2D> SpatialHash;
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
            public NativeList<ItemTest2D> ItemList;

            public NativeArray<float4> PositionBuffer;
            [ReadOnly] public float Time;

            public void Execute(int index)
            {
                ref float4 pos = ref PositionBuffer.GetElementAsRef(index);
                pos += new float4(math.sin(Time), math.cos(Time), 0, 0);
                ref ItemTest2D item = ref ItemList.ElementAt(index);
                item.Position = new float2(pos.x, pos.y);
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
            
            Profiler.BeginSample("SpatialHash_Move");
            //new MoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule().Complete();
            Profiler.EndSample();
            
            Profiler.BeginSample("SpatialHash_Remove_Add");
            // int length = _listItem.Length;
            // var inputDep = new JobHandle();
            // inputDep= new RemoveItemTestJob() { SpatialHash = _spatialHashing, ItemList = _listItem }.Schedule(inputDep);
            // inputDep = new AddItemTestJob() { SpatialHash = _spatialHashing.ToConcurrent(), ItemList = _listItem }.Schedule(length, 64, inputDep);
            // inputDep.Complete();
            Profiler.EndSample();
            
            
            m_QueryResults.Clear();
            Profiler.BeginSample("Query");
            //_spatialHashing.CircleQuery(new float2(m_CircleQueryTransform.position.x, m_CircleQueryTransform.position.y), m_CircleQueryRadius, m_QueryResults);
            
            _spatialHashing.SectorQuery(new Vector2(m_SectorOrigin.position.x, m_SectorOrigin.position.y), m_SectorDirection, m_SectorAngle, m_SectorRadius, m_QueryResults);
            Profiler.EndSample();
        }

        private void OnDrawGizmos()
        {
            if (Application.isPlaying == false)
                return;
            
            DrawSquare(new Vector3(_spatialHashing.WorldBounds.Center.x, _spatialHashing.WorldBounds.Center.y, 0), _spatialHashing.WorldBounds.Size.x);
            
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
            
            if (_DrawQueryResultGizmos)
            {
                Gizmos.color = new Color(1F, 0F, 0F, 0.3F);
                
                DrawCircle(m_CircleQueryTransform.position, m_CircleQueryRadius, 360, Vector3.forward);
                
                for (int i = 0; i < m_QueryResults.Length; i++)
                {
                    Gizmos.color = new Color(0F, 1F, 0F, 1F); 
                    
                    DrawSquare(new Vector3(m_QueryResults[i].Position.x, m_QueryResults[i].Position.y,0), m_QueryResults[i].Size);
                }
                
                Vector2 originVec2 = new Vector2(m_SectorOrigin.position.x, m_SectorOrigin.position.y);
                // 调用绘制扇形的函数
                DrawSectorGizmos(originVec2, m_SectorDirection.normalized, m_SectorAngle, m_SectorRadius, Color.yellow);

                Gizmos.color = Color.green;
                var aabb = _spatialHashing.CalculateSectorAABB(originVec2, m_SectorDirection.normalized, m_SectorAngle, m_SectorRadius);
                DrawSquare(new Vector3(aabb.Center.x, aabb.Center.y, 0), aabb.Size.x * 2);
            }
        }

        [SerializeField]
        private Transform m_SectorOrigin;
        [SerializeField]
        private Vector2 m_SectorDirection = new Vector2(1, 0);
        [SerializeField]
        private float m_SectorAngle = 90f;
        [SerializeField]
        private float m_SectorRadius = 10f;
        
        private void DrawSectorGizmos(Vector2 origin, Vector2 direction, float angle, float radius, Color color)
        {
            Gizmos.color = color;

            int segmentCount = 30; // 扇形边缘的分段数，值越大越圆滑
            float angleStep = angle / segmentCount;
            float halfAngle = angle / 2f;

            // 计算起始角度
            float startAngle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg - halfAngle;

            Vector3 prevPoint = origin;

            for (int i = 0; i <= segmentCount; i++)
            {
                float currentAngle = startAngle + angleStep * i;
                float rad = currentAngle * Mathf.Deg2Rad;

                Vector3 point = new Vector3(
                    origin.x + Mathf.Cos(rad) * radius,
                    origin.y + Mathf.Sin(rad) * radius,
                    0f);

                if (i > 0)
                {
                    // 绘制边缘线
                    Gizmos.DrawLine(prevPoint, point);
                }

                // 从原点绘制线到边缘
                Gizmos.DrawLine(origin, point);

                prevPoint = point;
            }
        }
        
        private void RefreshLinks()
        {
            _timeLastRefresh = Time.realtimeSinceStartup;
            _links.Clear();

            var unic = new HashSet<ItemTest2D>();
            var values = _spatialHashing.DebugBuckets.GetValueArray(Allocator.TempJob);

            foreach (var itemTest in values)
                unic.Add(_spatialHashing.DebugIDToItem[itemTest]);
            values.Dispose();

            var list = new NativeList<int2>(Allocator.Temp);

            foreach (var item in unic)
            {
                list.Clear();
                _spatialHashing.GetIndexiesVoxel(item, list);

                for (int i = 0; i < list.Length; i++)
                {
                    var index = list[i];

                    if (_links.TryGetValue(index, out var l) == false)
                    {
                        l = new List<ItemTest2D>();
                        _links.Add(index, l);
                    }

                    l.Add(item);
                }
            }

            list.Dispose();
        }

        private void DrawCell(int2 index)
        {
            var position = _spatialHashing.GetPositionVoxel(index, true);
            Gizmos.color = new Color(0F, 1F, 0F, 0.3F);
            DrawSquare(new Vector3(position.x, position.y,0), _spatialHashing.CellSize.x);
        }
        
        private void DrawCircle(Vector3 center, float radius, int segments, Vector3 normal)
        {
            // 确保法线向量被归一化
            normal = normal.normalized;

            // 选择一个任意的向量作为起始方向（需与法线不平行）
            Vector3 refDir = Vector3.up;

            // 如果法线与 refDir 平行，则选择另一个向量
            if (Vector3.Dot(normal, refDir) > 0.99f)
            {
                refDir = Vector3.right;
            }

            // 计算圆所在平面的基准向量
            Vector3 side = Vector3.Cross(normal, refDir).normalized;
            Vector3 up = Vector3.Cross(side, normal).normalized;

            // 设置 Gizmos 的颜色（可选）
            Gizmos.color = Color.green;

            // 计算每段的角度增量
            float angleDelta = 360f / segments;
            Quaternion rotation = Quaternion.AngleAxis(angleDelta, normal);

            // 计算第一个点的位置
            Vector3 point = center + side * radius;

            // 保存前一个点的位置
            Vector3 prevPoint = point;

            // 绘制圆
            for (int i = 1; i <= segments; i++)
            {
                // 旋转到下一个点
                side = rotation * side;
                point = center + side * radius;

                // 绘制线段
                Gizmos.DrawLine(prevPoint, point);

                // 更新前一个点
                prevPoint = point;
            }
        }
        
        private void DrawSquare(Vector3 center, float sideLength)
        {
            // 计算边长的一半
            float halfSide = sideLength / 2f;

            // 计算四个顶点的坐标（以 XY 平面为例，Z 轴保持不变）
            Vector3 topLeft = new Vector3(center.x - halfSide, center.y + halfSide, center.z);
            Vector3 topRight = new Vector3(center.x + halfSide, center.y + halfSide, center.z);
            Vector3 bottomRight = new Vector3(center.x + halfSide, center.y - halfSide, center.z);
            Vector3 bottomLeft = new Vector3(center.x - halfSide, center.y - halfSide, center.z);

            // 设置 Gizmos 的颜色（可选）
            Gizmos.color = Color.red;

            // 绘制四条边，连接四个顶点
            Gizmos.DrawLine(topLeft, topRight);       // 上边
            Gizmos.DrawLine(topRight, bottomRight);   // 右边
            Gizmos.DrawLine(bottomRight, bottomLeft); // 下边
            Gizmos.DrawLine(bottomLeft, topLeft);     // 左边
        }
        
        private void DrawLink(int2 cellIndex, ItemTest2D target)
        {
            var position = _spatialHashing.GetPositionVoxel(cellIndex, true);
            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3(position.x, position.y, 0), new Vector3(target.Position.x, target.Position.y, 0));
        }

        #region Implementation of IRay<ItemTest>
        
        public bool OnTraversingVoxel(int2 voxelIndex)
        {
            _voxelTraversed.Add(voxelIndex);

            return false;
        }
        
        public int2 GetIndexVoxel(float2 position)
        {
            return _spatialHashing.GetIndexVoxel(position);
        }
        
        public float2 GetPositionVoxel(int2 index, bool center)
        {
            return _spatialHashing.GetPositionVoxel(index, center);
        }
        
        public float2 CellSize
        {
            get { return _spatialHashing.CellSize; }
        }

        #endregion
  }
}