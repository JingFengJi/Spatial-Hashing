using System;
using System.Runtime.CompilerServices;
using SpatialHashing.Utils;
using Unity.Mathematics;
using UnityEngine;

namespace SpatialHashing
{
    [Serializable]
    public struct Bounds2D : IEquatable<Bounds2D>
    {
        public Bounds2D(float2 center, float2 size)
        {
            _center  = center;
            _extents = size * 0.5f;
        }
        
        public void SetMinMax(float2 min, float2 max)
        {
            Extents = (max - min) * 0.5f;
            Center  = min + Extents;
        }
        
        public void Encapsulate(float2 point)
        {
            SetMinMax(math.min(Min, point), math.max(Max, point));
        }

        public void Encapsulate(Bounds2D bounds)
        {
            Encapsulate(bounds.Center - bounds.Extents);
            Encapsulate(bounds.Center + bounds.Extents);
        }

        public void Clamp(Bounds2D bounds)
        {
            var bMin = bounds.Min;
            var bMax = bounds.Max;
            SetMinMax(math.clamp(Min, bMin, bMax), math.clamp(Max, bMin, bMax));
        }

        public void Expand(float amount)
        {
            amount  *= 0.5f;
            Extents += new float2(amount, amount);
        }

        public void Expand(float2 amount)
        {
            Extents += amount * 0.5f;
        }

        public bool Intersects(Bounds2D bounds)
        {
            return math.all(Min <= bounds.Max) && math.all(Max >= bounds.Min);
        }

        public int2 GetCellCount(float2 cellSize)
        {
            var min = Min;
            var max = Max;

            var diff = max - min;
            diff /= cellSize;

            return diff.CeilToInt();
        }

        /// <summary>
        /// Find the intersection of a line from v0 to v1 and an axis-aligned bounding box http://www.youtube.com/watch?v=USjbg5QXk3g
        /// <see cref="https://github.com/BSVino/MathForGameDevelopers/blob/line-box-intersection/math/collision.cpp"/>
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="direction"></param>
        /// <param name="length"></param>
        /// <param name="enterPoint"></param>
        /// <returns></returns>
        // 
        public bool GetEnterPositionAABB(float3 origin, float3 direction, float length)
        {
            var start = origin + direction * length;

            float low  = 0F;
            float high = 1F;

            return ClipLine(0, origin, start, ref low, ref high) && ClipLine(1, origin, start, ref low, ref high) && ClipLine(2, origin, start, ref low, ref high);
        }

        /// <summary>
        /// Find the intersection of a line from v0 to v1 and an axis-aligned bounding box http://www.youtube.com/watch?v=USjbg5QXk3g
        /// <see cref="https://github.com/BSVino/MathForGameDevelopers/blob/line-box-intersection/math/collision.cpp"/>
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="direction"></param>
        /// <param name="length"></param>
        /// <param name="enterPoint"></param>
        /// <returns></returns>

        // 
        public bool GetEnterPositionAABB(float3 origin, float3 direction, float length, out float3 enterPoint)
        {
            enterPoint = new float3();
            var start = origin + direction * length;

            float low  = 0F;
            float high = 1F;

            if (ClipLine(0, origin, start, ref low, ref high) == false || ClipLine(1, origin, start, ref low, ref high) == false || ClipLine(2, origin, start, ref low, ref high) == false)
                return false;

            // The formula for I: http://youtu.be/USjbg5QXk3g?t=6m24s
            var b = start - origin;
            enterPoint = origin + b * low;

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
       private bool ClipLine(int d, float3 v0, float3 v1,ref float low,ref float high)
        {
            // f_low and f_high are the results from all clipping so far. We'll write our results back out to those parameters.

            // f_dim_low and f_dim_high are the results we're calculating for this current dimension.
            // Find the point of intersection in this dimension only as a fraction of the total vector http://youtu.be/USjbg5QXk3g?t=3m12s
            var dimensionLow = (Min[d] - v0[d])/(v1[d] - v0[d]);
            var dimensionHigh = (Max[d] - v0[d])/(v1[d] - v0[d]);

            // Make sure low is less than high
            if (dimensionHigh < dimensionLow)
            {
                var tmp = dimensionHigh;
                dimensionHigh = dimensionLow;
                dimensionLow = tmp;
            }

            // If this dimension's high is less than the low we got then we definitely missed. http://youtu.be/USjbg5QXk3g?t=7m16s
            if (dimensionHigh < low)
                return false;

            // Likewise if the low is less than the high.
            if (dimensionLow > high)
                return false;

            // Add the clip from this dimension to the previous results http://youtu.be/USjbg5QXk3g?t=5m32s
            low  = math.max(dimensionLow, low);
            high = math.min(dimensionHigh, high);

            if (low > high)
                return false;

            return true;
        }

        public override string ToString()
        {
            return $"Center: {_center}, Extents: {_extents}";
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(_center, _extents);
        }

        public static bool operator ==(Bounds2D lhs, Bounds2D rhs)
        {
            return math.all(lhs.Center == rhs.Center) & math.all(lhs.Extents == rhs.Extents);
        }

        public static bool operator !=(Bounds2D lhs, Bounds2D rhs)
        {
            return !(lhs == rhs);
        }
        
        
        #region Variables

        [SerializeField]
        private float2 _center;
        [SerializeField]
        private float2 _extents;

        #endregion

        #region Properties

        public float2 Center { get { return _center; } set { _center = value; } }

        public float2 Size { get { return _extents * 2f; } set { _extents = value * 0.5f; } }

        public float2 Extents { get { return _extents; } set { _extents = value; } }

        public float2 Min { get { return Center - Extents; } set { SetMinMax(value, Max); } }

        public float2 Max { get { return Center + Extents; } set { SetMinMax(Min, value); } }

        #endregion

        public bool Equals(Bounds2D other)
        {
            return _center.Equals(other._center) && _extents.Equals(other._extents);
        }
    }
}