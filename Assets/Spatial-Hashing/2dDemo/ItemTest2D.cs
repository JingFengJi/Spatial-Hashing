using Unity.Entities;
using Unity.Mathematics;

namespace SpatialHashing.Demo
{
    
    public struct ItemTest2D : ISpatialHashing2DItem<ItemTest2D>, IComponentData
    {
        public int ID;
        public float2 Position;
        public float Size;
        
        #region Implementation of IEquatable<ItemTest>

        /// <inheritdoc />
        public bool Equals(ItemTest2D other)
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
        public float2 GetCenter()
        {
            return Position;
        }

        /// <inheritdoc />
        public float2 GetSize()
        {
            return new float2(Size);
        }

        /// <inheritdoc />
        public int SpatialHashingIndex { get; set; }

        #endregion
    }

}