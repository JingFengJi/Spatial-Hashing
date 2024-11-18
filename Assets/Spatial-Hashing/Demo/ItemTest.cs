using Unity.Entities;
using Unity.Mathematics;

namespace SpatialHashing.Demo
{
    
    public struct ItemTest : ISpatialHashingItem<ItemTest>, IComponentData
    {
        public int ID;
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