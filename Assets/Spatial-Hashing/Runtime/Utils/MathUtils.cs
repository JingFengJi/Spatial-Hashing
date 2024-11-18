using Unity.Mathematics;

namespace SpatialHashing.Utils
{
    public static class MathUtils
    {
        public static Bounds2D CalculateSectorAABB(float2 origin, float2 direction, float angle, float radius)
        {
            float2 dir = math.normalize(direction);

            // 计算扇形的起始和结束角度（以弧度为单位）
            float theta0 = math.degrees(math.atan2(dir.y, dir.x));
            float halfAngle = angle * 0.5f;
            float theta1 = theta0 - halfAngle;
            float theta2 = theta0 + halfAngle;

            // 确保角度在 [0, 2π) 范围内
            theta1 = NormalizeAngle(theta1);
            theta2 = NormalizeAngle(theta2);

            // 初始化最小和最大 x, y 值为原点坐标
            float minX = origin.x;
            float maxX = origin.x;
            float minY = origin.y;
            float maxY = origin.y;

            // 计算扇形弧的两个端点，并更新最小和最大值
            UpdateMinMaxWithAngle(origin, radius, theta1, ref minX, ref maxX, ref minY, ref maxY);
            UpdateMinMaxWithAngle(origin, radius, theta2, ref minX, ref maxX, ref minY, ref maxY);

            // 检查并处理关键角度（0, π/2, π, 3π/2）
            CheckAndUpdateCriticalAngle(origin, radius, theta1, theta2, 0, ref minX, ref maxX, ref minY, ref maxY);
            CheckAndUpdateCriticalAngle(origin, radius, theta1, theta2, 90, ref minX, ref maxX, ref minY, ref maxY);
            CheckAndUpdateCriticalAngle(origin, radius, theta1, theta2, 180, ref minX, ref maxX, ref minY, ref maxY);
            CheckAndUpdateCriticalAngle(origin, radius, theta1, theta2, 270, ref minX, ref maxX, ref minY, ref maxY);

            // 计算中心和范围
            float2 center = new float2((minX + maxX) * 0.5f, (minY + maxY) * 0.5f);
            float2 extents = new float2(maxX - minX, maxY - minY);

            return new Bounds2D(center, extents);
        }
        
        
        private static float NormalizeAngle(float angle)
        {
            float twoPi = 360;
            angle %= twoPi;
            if (angle < 0f)
                angle += twoPi;
            return angle;
        }

        private static void UpdateMinMaxWithAngle(float2 origin, float radius, float angle, ref float minX, ref float maxX, ref float minY, ref float maxY)
        {
            float radians = math.radians(angle);
            float x = origin.x + math.cos(radians) * radius;
            float y = origin.y + math.sin(radians) * radius;

            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }

        private static void CheckAndUpdateCriticalAngle(float2 origin, float radius, float startAngle, float endAngle, float criticalAngle, ref float minX, ref float maxX, ref float minY, ref float maxY)
        {
            if (IsAngleInSector(criticalAngle, startAngle, endAngle))
            {
                UpdateMinMaxWithAngle(origin, radius, criticalAngle, ref minX, ref maxX, ref minY, ref maxY);
            }
        }

        private static bool IsAngleInSector(float angle, float startAngle, float endAngle)
        {
            // 处理角度环绕的情况
            if (startAngle > endAngle)
            {
                return angle >= startAngle || angle <= endAngle;
            }
            else
            {
                return angle >= startAngle && angle <= endAngle;
            }
        }

        // 判断扇形的圆弧是否与 Bounds 相交
        private static bool SectorArcIntersectsBounds(Bounds2D bounds, float2 sectorOrigin, float theta1, float theta2,
            float sectorRadius)
        {
            // 检查 Bounds 的边是否与扇形的圆弧相交
            // 由于扇形的圆弧是曲线，这里简化处理，取多个点进行采样检测
            const int sampleCount = 8; // 采样点数量，可以根据需要调整
            float angleStep = (theta2 - theta1) / sampleCount;
            for (int i = 0; i <= sampleCount; i++)
            {
                float angle = theta1 + angleStep * i;
                angle = NormalizeAngle(angle);
                float2 pointOnArc = sectorOrigin + new float2(math.cos(math.radians(angle)), math.sin(math.radians(angle))) * sectorRadius;
                if (PointInBounds(pointOnArc, bounds))
                {
                    return true;
                }
            }
            return false;
        }

        // 判断点是否在 Bounds 内
        private static bool PointInBounds(float2 point, Bounds2D bounds)
        {
            float2 min = bounds.Center - bounds.Extents;
            float2 max = bounds.Center + bounds.Extents;
            return (point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y);
        }
        
        
        public static bool BoundsIntersectsSector(Bounds2D bounds, float2 sectorOrigin, float2 sectorDirection, float sectorAngle, float sectorRadius)
        {
            // 将扇形方向归一化
            float2 dir = math.normalize(sectorDirection);

            // 计算扇形的起始和结束角度（以弧度为单位）
            float theta0 = math.degrees(math.atan2(dir.y, dir.x));
            float halfAngle = sectorAngle * 0.5f;
            float theta1 = theta0 - halfAngle;
            float theta2 = theta0 + halfAngle;

            // 获取 Bounds 的四个顶点
            float2 min = bounds.Center - bounds.Extents;
            float2 max = bounds.Center + bounds.Extents;

            float2 corner1 = new float2(min.x, min.y);
            float2 corner2 = new float2(min.x, max.y);
            float2 corner3 = new float2(max.x, max.y);
            float2 corner4 = new float2(max.x, min.y);
            

            // 检查 Bounds 的顶点是否在扇形内
            if (PointInSector(corner1, sectorOrigin, dir, sectorAngle, sectorRadius) ||
                PointInSector(corner2, sectorOrigin, dir, sectorAngle, sectorRadius) ||
                PointInSector(corner3, sectorOrigin, dir, sectorAngle, sectorRadius) ||
                PointInSector(corner4, sectorOrigin, dir, sectorAngle, sectorRadius))
            {
                return true;
            }
            
            // 检查扇形的圆弧是否与 Bounds 相交
            if (SectorArcIntersectsBounds(bounds, sectorOrigin, theta1, theta2, sectorRadius))
            {
                return true;
            }

            // 检查扇形的两条边是否与 Bounds 相交
            float2 edge1End = sectorOrigin + new float2(math.cos(math.radians(theta1)), math.sin(math.radians(theta1))) * sectorRadius;
            float2 edge2End = sectorOrigin + new float2(math.cos(math.radians(theta2)), math.sin(math.radians(theta2))) * sectorRadius;

            if (LineIntersectsBounds(bounds, sectorOrigin, edge1End) || LineIntersectsBounds(bounds, sectorOrigin, edge2End))
            {
                return true;
            }

            // 检查 Bounds 的边是否包含扇形的中心
            if (PointInBounds(sectorOrigin, bounds))
            {
                return true;
            }

            return false;
        }
        
        private static bool PointInSector(float2 point, float2 sectorOrigin, float2 sectorDir, float sectorAngle, float sectorRadius)
        {
            float2 dirToPoint = point - sectorOrigin;
            float distance = math.length(dirToPoint);

            if (distance > sectorRadius)
                return false;

            dirToPoint = math.normalize(dirToPoint);

            float dot = math.dot(sectorDir, dirToPoint);
            float angleToPoint = math.degrees(math.acos(dot));

            return angleToPoint <= sectorAngle * 0.5f;
        }
        
        

        private static bool LineIntersectsBounds(Bounds2D bounds, float2 p1, float2 p2)
        {
            float2 min = bounds.Center - bounds.Extents;
            float2 max = bounds.Center + bounds.Extents;

            // 检查线段是否与 Bounds 的四条边相交
            if (LineIntersectsLine(p1, p2, new float2(min.x, min.y), new float2(min.x, max.y)) ||
                LineIntersectsLine(p1, p2, new float2(min.x, max.y), new float2(max.x, max.y)) ||
                LineIntersectsLine(p1, p2, new float2(max.x, max.y), new float2(max.x, min.y)) ||
                LineIntersectsLine(p1, p2, new float2(max.x, min.y), new float2(min.x, min.y)))
            {
                return true;
            }

            return false;
        }

        // 判断两条线段是否相交
        private static bool LineIntersectsLine(float2 p1, float2 p2, float2 q1, float2 q2)
        {
            float2 r = p2 - p1;
            float2 s = q2 - q1;
            float denom = r.x * s.y - r.y * s.x;
            if (denom == 0)
            {
                return false; // 平行或共线
            }
            float u = ((q1.x - p1.x) * r.y - (q1.y - p1.y) * r.x) / denom;
            float t = ((q1.x - p1.x) * s.y - (q1.y - p1.y) * s.x) / denom;
            return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
        }
        
        public static bool BoundsIntersectsCircle(Bounds2D bounds, float2 sphereCenter, float radius)
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
        
        public static bool BoundsIntersectsSphere(Bounds bounds, float3 sphereCenter, float radius)
        {
            float sqDistance = 0f;
            float3 min = bounds.Min;
            float3 max = bounds.Max;
            for (int i = 0; i < 3; i++)
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
    }
}