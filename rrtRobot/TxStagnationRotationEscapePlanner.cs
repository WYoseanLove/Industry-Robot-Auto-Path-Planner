using System;
using System.Collections;
using System.Collections.Generic;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using static System.Net.Mime.MediaTypeNames;

namespace rrtRobot
{
    /// <summary>
    /// 旋转轴枚举。
    /// 表示绕 TCP 自身局部坐标轴 X / Y / Z 旋转。
    /// </summary>
    public enum EscapeRotationAxis
    {
        X = 0,
        Y = 1,
        Z = 2
    }

    /// <summary>
    /// 旋转扫描方向定义。
    /// 例如 Z-、Z+、X-、X+、Y-、Y+。
    /// </summary>
    public class EscapeRotationDirection
    {
        public string Name { get; set; }

        public EscapeRotationAxis Axis { get; set; }

        public int Sign { get; set; }
    }

    /// <summary>
    /// 密闭空间逃逸结果。
    /// </summary>
    public class StagnationRotationEscapeResult
    {
        /// <summary>
        /// 是否成功生成逃离点位。
        /// </summary>
        public bool Success { get; set; }

        /// <summary>
        /// 是否需要退回普通随机点。
        /// </summary>
        public bool UseFallbackRandom { get; set; }

        /// <summary>
        /// 结果说明。
        /// </summary>
        public string Reason { get; set; }

        /// <summary>
        /// 当前用于扫描的锚点关节。
        /// </summary>
        public joint AnchorJoint { get; set; }

        /// <summary>
        /// 当前用于扫描的锚点位姿。
        /// </summary>
        public point AnchorPoint { get; set; }

        /// <summary>
        /// 旋转扫描得到的碰撞点。
        /// 只保留成对存在的方向点：
        /// Z-、Z+、X-、X+、Y-、Y+ 中，只有某轴的正负两个方向都存在时才加入。
        /// 
        /// 例如：
        /// 1. 仅有 Z-、Z+，则只保存这 2 个点；
        /// 2. 有 Z-、Z+、X-、X+，则保存这 4 个点；
        /// 3. 若只有 Z-、Z+、X+，则 X+ 会被舍弃，只保留 Z-、Z+。
        /// </summary>
        public List<TxVector> CollisionPoints { get; set; } = new List<TxVector>();

        /// <summary>
        /// 由成对碰撞点求得的逃离中点。
        /// 姿态保持与 AnchorPoint 一致。
        /// </summary>
        public point EscapePoint { get; set; }

        /// <summary>
        /// 逃离中点对应的最优逆解。
        /// </summary>
        public joint EscapeJoint { get; set; }

        /// <summary>
        /// 在 EscapeJoint 附近 ±5deg 扰动得到的 rand 点。
        /// </summary>
        public joint RandJoint { get; set; }

        /// <summary>
        /// 当逆解失败时返回的普通随机点。
        /// </summary>
        public joint FallbackRandomJoint { get; set; }

        /// <summary>
        /// 预留字段。
        /// 当前版本中不在本类内部做多次重新锚定。
        /// </summary>
        public int ReanchorCount { get; set; }
    }
    /// <summary>
    /// 关节限位盒，用于限制出口吸引点不要超出机器人关节软限位。
    /// </summary>
    public class JointLimitBox
    {
        public double[] Lower { get; private set; }
        public double[] Upper { get; private set; }

        public JointLimitBox(
            double j1L, double j2L, double j3L, double j4L, double j5L, double j6L,
            double j1U, double j2U, double j3U, double j4U, double j5U, double j6U)
        {
            Lower = new double[] { j1L, j2L, j3L, j4L, j5L, j6L };
            Upper = new double[] { j1U, j2U, j3U, j4U, j5U, j6U };
        }

        public double[] Clamp(double[] q)
        {
            double[] r = new double[6];

            for (int i = 0; i < 6; i++)
            {
                r[i] = Math.Max(Lower[i], Math.Min(Upper[i], q[i]));
            }

            return r;
        }
    }

    /// <summary>
    /// 密闭空间旋转扫描逃逸规划器。
    /// 
    /// 逻辑：
    /// 1. 以当前锚点为中心，保持 XYZ 不变；
    /// 2. 分别绕 TCP 自身坐标轴的 Z-、Z+、X-、X+、Y-、Y+ 按固定角度步长旋转；
    /// 3. 每个方向都从原始锚点重新开始扫描，不做姿态累积；
    /// 4. 在每个方向上找到第一个碰撞姿态，并用 Collision_CheckPoint 得到碰撞点；
    /// 5. 仅保留成对存在的方向点；
    /// 6. 用保留下来的碰撞点求中点，得到逃离点位；
    /// 7. 对逃离点位求逆解，并用 ChooseBestInverseSolution 选择最优解；
    /// 8. 在 EscapeJoint 附近做 ±5deg 扰动，得到 rand 点。
    /// 
    /// 注意：
    /// 本类只负责“计算一次逃离点位”以及“基于已有 EscapeJoint 生成 rand”。
    /// 是否需要重新计算 escapePoint，由外部通过 NeedRefreshEscapePoint 判断。
    /// </summary>
    public class TxStagnationRotationEscapePlanner
    {
        private readonly Random random = new Random();

        /// <summary>
        /// 旋转角步长，单位：度。
        /// 默认每 0.1 度扫描一次。
        /// </summary>
        public double RotationStepDeg { get; set; } = 0.1;

        /// <summary>
        /// 单方向最大旋转角度，单位：度。
        /// </summary>
        public double MaxRotationDeg { get; set; } = 180.0;

        /// <summary>
        /// 逃离姿态附近生成 rand 点的扰动范围，单位：度。
        /// </summary>
        public double RandJitterDeg { get; set; } = 15;

        /// <summary>
        /// 碰撞检测使用的 Clearance。
        /// </summary>
        public double CollisionClearance { get; set; } = 5.0;

        /// <summary>
        /// 关节限位盒。
        /// </summary>
        public JointLimitBox JointLimits { get; private set; }

        /// <summary>
        /// 普通随机点生成时的焊钳开度离散份数。
        /// </summary>
        public int GunOpenSplit { get; set; } = 30;

        public TxStagnationRotationEscapePlanner(JointLimitBox jointLimits)
        {
            JointLimits = jointLimits;
        }

        /// <summary>
        /// 更新关节限位。
        /// </summary>
        public void UpdateJointLimits(JointLimitBox jointLimits)
        {
            JointLimits = jointLimits;
        }

        /// <summary>
        /// 计算一次新的逃离点位。
        /// 
        /// 注意：
        /// 这里不负责判断“是否需要重新计算”，
        /// 也不负责在内部多次重复重算 escapePoint。
        /// 外部若需要复用 escapePoint，应缓存结果并通过 NeedRefreshEscapePoint 决定何时重算。
        /// </summary>
        public StagnationRotationEscapeResult GenerateEscapeRand(
            Control control,
            joint rootJoint,
            List<Node3D_joint> treeNodes,
            double stepSize)
        {
            StagnationRotationEscapeResult result = new StagnationRotationEscapeResult();
            result.Success = false;
            result.UseFallbackRandom = true;
            result.Reason = "Escape planning not started.";
            result.FallbackRandomJoint = CreateFallbackRandomJoint(treeNodes[0].loc);

            if (treeNodes == null || treeNodes.Count == 0)
            {
                result.Reason = "Tree nodes is empty. Use fallback random.";
                return result;
            }

            point currentAnchorPoint = ConvertJointToPoint(control, rootJoint);

            result.AnchorJoint = rootJoint;
            result.AnchorPoint = currentAnchorPoint;
            result.ReanchorCount = 0;
            result.CollisionPoints.Clear();

            List<TxVector> collisionPoints;
            if (!TryGetSixCollisionPoints(
                control,
                currentAnchorPoint,
                rootJoint,
                out collisionPoints))
            {
                result.Reason = "Failed to get paired collision points by rotational scan. Use fallback random.";
                return result;
            }

            result.CollisionPoints.AddRange(collisionPoints);

            point escapePoint = BuildEscapeMidPoint(currentAnchorPoint, collisionPoints);
            result.EscapePoint = escapePoint;

            joint escapeJoint;
            if (!TrySolveBestValidJoint(control, escapePoint, rootJoint, out escapeJoint))
            {
                result.Reason = "Inverse solution failed for escape midpoint. Use fallback random.";
                return result;
            }

            result.EscapeJoint = escapeJoint;

            joint randJoint;
            if (!TryGenerateRandJoint(control, escapeJoint, out randJoint))
            {
                randJoint = escapeJoint;
            }

            result.Success = true;
            result.UseFallbackRandom = false;
            result.RandJoint = randJoint;
            result.Reason = "Escape point generated successfully.";
            return result;
        }
        /// <summary>
        /// 判断当前缓存的逃离点位是否需要退出 escape mode。
        /// 
        /// 新规则：
        /// 1. 根节点使用本次 escape 生成时记录的 AnchorPoint；
        /// 2. 逃逸方向向量为 EscapePoint - AnchorPoint；
        /// 3. 只取当前树节点列表中的最后一个节点；
        /// 4. 计算“最后一个节点点位 - 根节点点位”的 3D 向量；
        /// 5. 将该向量投影到“EscapePoint - AnchorPoint”方向上；
        /// 6. 如果投影长度大于等于 EscapePoint 到根节点的长度，
        ///    则说明该节点已经沿逃逸方向越过了 EscapePoint，返回 true；
        /// 7. 否则返回 false。
        /// 
        /// 注意：
        /// refreshDistanceMm 参数为兼容现有调用保留，当前逻辑中不再使用。
        /// </summary>
        public bool NeedRefreshEscapePoint(
            Control control,
            StagnationRotationEscapeResult escapeResult,
            List<Node3D_joint> treeNodes)
        {
            if (escapeResult == null || !escapeResult.Success)
            {
                return true;
            }

            if (treeNodes == null || treeNodes.Count == 0)
            {
                return true;
            }

            point rootPoint = escapeResult.AnchorPoint;
            point escapePoint = escapeResult.EscapePoint;

            double vx = escapePoint.x - rootPoint.x;
            double vy = escapePoint.y - rootPoint.y;
            double vz = escapePoint.z - rootPoint.z;

            double escapeLength = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (escapeLength < 1e-6)
            {
                return true;
            }

            double dx = vx / escapeLength;
            double dy = vy / escapeLength;
            double dz = vz / escapeLength;

            Node3D_joint lastNode = treeNodes[treeNodes.Count - 1];
            point nodePoint = ConvertJointToPoint(control, lastNode.loc);

            double nx = nodePoint.x - rootPoint.x;
            double ny = nodePoint.y - rootPoint.y;
            double nz = nodePoint.z - rootPoint.z;

            double projectionLength =
                nx * dx +
                ny * dy +
                nz * dz;

            return projectionLength >= escapeLength;
        }
        /// <summary>
        /// 基于已有 EscapeJoint，再生成一个新的 ±5deg rand 点。
        /// 不重新计算 escapePoint。
        /// </summary>
        public bool TryGenerateRandFromEscapeJoint(
            Control control,
            joint escapeJoint,
            out joint randJoint)
        {
            return TryGenerateRandJoint(control, escapeJoint, out randJoint);
        }

        /// <summary>
        /// 按方向扫描碰撞点，并只保留成对存在的方向点。
        /// 
        /// 规则：
        /// 1. Z-、Z+ 必须同时存在，才加入结果；
        /// 2. X-、X+ 必须同时存在，才加入结果；
        /// 3. Y-、Y+ 必须同时存在，才加入结果；
        /// 4. 只要至少存在一对，就返回 true；
        /// 5. 若没有任何成对方向，则返回 false。
        /// 
        /// 返回顺序固定按轴排列：
        /// Z-、Z+、X-、X+、Y-、Y+。
        /// 但只包含成对成功的点。
        /// </summary>
        private bool TryGetSixCollisionPoints(
            Control control,
            point anchorPoint,
            joint referenceJoint,
            out List<TxVector> collisionPoints)
        {
            collisionPoints = new List<TxVector>();

            TxVector zMinusPoint;
            TxVector zPlusPoint;
            TxVector xMinusPoint;
            TxVector xPlusPoint;
            TxVector yMinusPoint;
            TxVector yPlusPoint;

            ApplyJoint(control, referenceJoint);
            bool hasZMinus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "Z-",
                    Axis = EscapeRotationAxis.Z,
                    Sign = -1
                },
                out zMinusPoint);

            ApplyJoint(control, referenceJoint);
            bool hasZPlus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "Z+",
                    Axis = EscapeRotationAxis.Z,
                    Sign = 1
                },
                out zPlusPoint);

            ApplyJoint(control, referenceJoint);
            bool hasXMinus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "X-",
                    Axis = EscapeRotationAxis.X,
                    Sign = -1
                },
                out xMinusPoint);

            ApplyJoint(control, referenceJoint);
            bool hasXPlus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "X+",
                    Axis = EscapeRotationAxis.X,
                    Sign = 1
                },
                out xPlusPoint);

            ApplyJoint(control, referenceJoint);
            bool hasYMinus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "Y-",
                    Axis = EscapeRotationAxis.Y,
                    Sign = -1
                },
                out yMinusPoint);

            ApplyJoint(control, referenceJoint);
            bool hasYPlus = TryFindFirstCollisionPointByRotation(
                control,
                anchorPoint,
                referenceJoint,
                new EscapeRotationDirection
                {
                    Name = "Y+",
                    Axis = EscapeRotationAxis.Y,
                    Sign = 1
                },
                out yPlusPoint);

            if (hasZMinus && hasZPlus)
            {
                collisionPoints.Add(zMinusPoint);
                collisionPoints.Add(zPlusPoint);
            }

            if (hasXMinus && hasXPlus)
            {
                collisionPoints.Add(xMinusPoint);
                collisionPoints.Add(xPlusPoint);
            }

            if (hasYMinus && hasYPlus)
            {
                collisionPoints.Add(yMinusPoint);
                collisionPoints.Add(yPlusPoint);
            }

            return collisionPoints.Count >= 2;
        }

        /// <summary>
        /// 在一个旋转方向上，从原始锚点开始，每 RotationStepDeg 扫描一次，
        /// 找到第一个发生碰撞的位置，并通过 Collision_CheckPoint 获取碰撞点。
        /// 
        /// 重要：
        /// 1. XYZ 始终保持为原始锚点的 XYZ；
        /// 2. 姿态旋转是绕 TCP 自身局部坐标轴；
        /// 3. 当前方向内的每一个角度，都是“相对原始锚点姿态”的角度；
        /// 4. 不是基于上一步姿态继续累加。
        /// </summary>
        private bool TryFindFirstCollisionPointByRotation(
            Control control,
            point anchorPoint,
            joint referenceJoint,
            EscapeRotationDirection direction,
            out TxVector firstCollisionPoint)
        {
            firstCollisionPoint = new TxVector(0, 0, 0);

            for (double angleDeg = RotationStepDeg; angleDeg <= MaxRotationDeg; angleDeg += RotationStepDeg)
            {
                double angleRad = direction.Sign * angleDeg * Math.PI / 180.0;

                point rotatedPoint = RotatePointAroundLocalAxis(anchorPoint, direction.Axis, angleRad);

                joint rotatedJoint;
                if (!TryGetClosestInverseJoint(control, rotatedPoint, referenceJoint, out rotatedJoint))
                {
                    continue;
                }

                ApplyJoint(control, rotatedJoint);

                TxVector collisionPoint = TxRobotAPIClass.Collision_CheckPoint(
                    control,
                    TxrrtRobotPathPlannerForm.cd,
                    TxrrtRobotPathPlannerForm.queryParams,
                    TxrrtRobotPathPlannerForm.root,
                    TxrrtRobotPathPlannerForm.collisionSrc,
                    TxrrtRobotPathPlannerForm.collisionTar,
                    CollisionClearance);

                if (!IsZeroVector(collisionPoint))
                {
                    firstCollisionPoint = collisionPoint;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 由所有成对保留下来的碰撞点求中点。
        /// 点数可能是 2、4 或 6。
        /// 姿态保持为当前锚点姿态。
        /// </summary>
        private point BuildEscapeMidPoint(point anchorPoint, List<TxVector> collisionPoints)
        {
            double x = 0;
            double y = 0;
            double z = 0;

            for (int i = 0; i < collisionPoints.Count; i++)
            {
                x += collisionPoints[i].X;
                y += collisionPoints[i].Y;
                z += collisionPoints[i].Z;
            }

            x /= collisionPoints.Count;
            y /= collisionPoints.Count;
            z /= collisionPoints.Count;

            return new point(
                x,
                y,
                z,
                anchorPoint.rx,
                anchorPoint.ry,
                anchorPoint.rz,
                anchorPoint.Gun_Open);
        }

        /// <summary>
        /// 对 escapePoint 做逆解。
        /// 
        /// 规则：
        /// 1. 目标点本身无逆解，直接返回 false；
        /// 2. 目标点有逆解且单点无碰撞，直接返回 true；
        /// 3. 目标点有逆解但单点碰撞失败，则沿 root -> targetPoint 的 3D 方向，
        ///    以 5mm 步长继续远离 root 搜索；
        /// 4. 返回第一个无干涉点；
        /// 5. 如果向外搜索过程中先出现无逆解，且之前一直未找到无干涉点，则返回 false。
        /// </summary>
        private bool TrySolveBestValidJoint(
            Control control,
            point targetPoint,
            joint referenceJoint,
            out joint bestJoint)
        {
            bestJoint = new joint();

            ArrayList solutions = null;

            try
            {
                solutions = TxRobotAPIClass.robotInverseCal(
                    control,
                    TxrrtRobotPathPlannerForm.robot,
                    targetPoint);

                // 目标点本身无逆解，直接失败
                if (solutions == null || solutions.Count == 0)
                {
                    return false;
                }

                using (TxPoseData prePoseData = new TxPoseData())
                {
                    ArrayList preJointValues = new ArrayList(referenceJoint.ToArray());
                    prePoseData.JointValues = preJointValues;

                    int bestIndex =
                        TxRobotPathOptimizePtp.ChooseBestInverseSolution(
                            ref solutions,
                            prePoseData);

                    if (bestIndex < 0 || bestIndex >= solutions.Count)
                    {
                        return false;
                    }

                    TxPoseData pose = solutions[bestIndex] as TxPoseData;
                    if (pose == null || pose.JointValues == null || pose.JointValues.Count < 6)
                    {
                        return false;
                    }

                    joint candidate = new joint(
                        Convert.ToDouble(pose.JointValues[0]),
                        Convert.ToDouble(pose.JointValues[1]),
                        Convert.ToDouble(pose.JointValues[2]),
                        Convert.ToDouble(pose.JointValues[3]),
                        Convert.ToDouble(pose.JointValues[4]),
                        Convert.ToDouble(pose.JointValues[5]),
                        targetPoint.Gun_Open);

                    if (!IsInsideJointLimits(candidate))
                    {
                        return false;
                    }

                    joint test = candidate;
                    if (TxRobotRRTConnectJoint.collisioncheckforSingleJoint(control, ref test))
                    {
                        bestJoint = test;
                        return true;
                    }
                }
            }
            finally
            {
                if (solutions != null)
                {
                    TxRobotAPIClass.DisposeTxposureData(solutions);
                }
            }

            // 只有“目标点有逆解但发生单点干涉”时才进入这里
            return TrySolveBestValidJointByMovingAwayFromRoot(
                control,
                targetPoint,
                referenceJoint,
                out bestJoint);
        }
        /// <summary>
        /// 当目标点已有逆解但单点碰撞失败时，
        /// 沿 root -> targetPoint 的 3D 方向，以 5mm 步长继续远离 root，
        /// 直到找到第一个无碰撞点。
        /// 
        /// 规则：
        /// 1. 姿态保持 targetPoint 的姿态不变，只沿 3D 方向平移 XYZ；
        /// 2. 每一步重新求逆解；
        /// 3. 如果某一步无逆解，且之前一直未找到无碰撞点，则直接返回 false；
        /// 4. 返回第一个无碰撞点。
        /// </summary>
        private bool TrySolveBestValidJointByMovingAwayFromRoot(
            Control control,
            point targetPoint,
            joint referenceJoint,
            out joint bestJoint)
        {
            bestJoint = new joint();

            point rootPoint = ConvertJointToPoint(control, referenceJoint);

            double vx = targetPoint.x - rootPoint.x;
            double vy = targetPoint.y - rootPoint.y;
            double vz = targetPoint.z - rootPoint.z;

            double baseLength = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            if (baseLength < 1e-6)
            {
                return false;
            }

            double dx = vx / baseLength;
            double dy = vy / baseLength;
            double dz = vz / baseLength;

            const double moveStepMm = 5.0;
            const int maxMoveStepCount = 200;

            for (int stepIndex = 1; stepIndex <= maxMoveStepCount; stepIndex++)
            {
                double moveDistance = moveStepMm * stepIndex;

                point movedPoint = new point(
                    targetPoint.x + dx * moveDistance,
                    targetPoint.y + dy * moveDistance,
                    targetPoint.z + dz * moveDistance,
                    targetPoint.rx,
                    targetPoint.ry,
                    targetPoint.rz,
                    targetPoint.Gun_Open);

                ArrayList movedSolutions = null;

                try
                {
                    movedSolutions = TxRobotAPIClass.robotInverseCal(
                        control,
                        TxrrtRobotPathPlannerForm.robot,
                        movedPoint);

                    // 一旦向外移动到无逆解，且之前仍未找到无干涉点，则失败
                    if (movedSolutions == null || movedSolutions.Count == 0) continue;


                    using (TxPoseData prePoseData = new TxPoseData())
                    {
                        ArrayList preJointValues = new ArrayList(referenceJoint.ToArray());
                        prePoseData.JointValues = preJointValues;

                        int bestIndex =
                            TxRobotPathOptimizePtp.ChooseBestInverseSolution(
                                ref movedSolutions,
                                prePoseData);

                        if (bestIndex < 0 || bestIndex >= movedSolutions.Count)
                        {
                            return false;
                        }

                        TxPoseData pose = movedSolutions[bestIndex] as TxPoseData;
                        if (pose == null || pose.JointValues == null || pose.JointValues.Count < 6)
                        {
                            return false;
                        }

                        joint candidate = new joint(
                            Convert.ToDouble(pose.JointValues[0]),
                            Convert.ToDouble(pose.JointValues[1]),
                            Convert.ToDouble(pose.JointValues[2]),
                            Convert.ToDouble(pose.JointValues[3]),
                            Convert.ToDouble(pose.JointValues[4]),
                            Convert.ToDouble(pose.JointValues[5]),
                            movedPoint.Gun_Open);

                        if (!IsInsideJointLimits(candidate))
                        {
                            return false;
                        }

                        joint test = candidate;
                        if (TxRobotRRTConnectJoint.collisioncheckforSingleJoint(control, ref test))
                        {
                            bestJoint = test;
                            return true;
                        }
                    }
                }
                finally
                {
                    if (movedSolutions != null)
                    {
                        TxRobotAPIClass.DisposeTxposureData(movedSolutions);
                    }
                }
            }

            return false;
        }
        /// <summary>
        /// 用于旋转扫描过程中的逆解选择。
        /// 
        /// 扫描阶段不要求该姿态无碰撞，因为目的正是找到第一个碰撞姿态。
        /// 这里只根据 referenceJoint 选择一个最接近的最优逆解，用于更新机器人姿态后取碰撞点。
        /// </summary>
        private bool TryGetClosestInverseJoint(
            Control control,
            point targetPoint,
            joint referenceJoint,
            out joint closestJoint)
        {
            closestJoint = new joint();

            ArrayList solutions = null;

            try
            {
                solutions = TxRobotAPIClass.robotInverseCal(
                    control,
                    TxrrtRobotPathPlannerForm.robot,
                    targetPoint);

                if (solutions == null || solutions.Count == 0)
                {
                    return false;
                }

                using (TxPoseData prePoseData = new TxPoseData())
                {
                    ArrayList preJointValues = new ArrayList(referenceJoint.ToArray());
                    prePoseData.JointValues = preJointValues;

                    int bestIndex =
                        TxRobotPathOptimizePtp.ChooseBestInverseSolution(
                            ref solutions,
                            prePoseData);

                    if (bestIndex < 0 || bestIndex >= solutions.Count)
                    {
                        return false;
                    }

                    TxPoseData pose = solutions[bestIndex] as TxPoseData;
                    if (pose == null || pose.JointValues == null || pose.JointValues.Count < 6)
                    {
                        return false;
                    }

                    joint candidate = new joint(
                        Convert.ToDouble(pose.JointValues[0]),
                        Convert.ToDouble(pose.JointValues[1]),
                        Convert.ToDouble(pose.JointValues[2]),
                        Convert.ToDouble(pose.JointValues[3]),
                        Convert.ToDouble(pose.JointValues[4]),
                        Convert.ToDouble(pose.JointValues[5]),
                        targetPoint.Gun_Open);

                    if (!IsInsideJointLimits(candidate))
                    {
                        return false;
                    }

                    closestJoint = candidate;
                    return true;
                }
            }
            finally
            {
                if (solutions != null)
                {
                    TxRobotAPIClass.DisposeTxposureData(solutions);
                }
            }
        }

        /// <summary>
        /// 在 escapeJoint 附近做 ±5deg 小范围扰动，生成 rand 点。
        /// 优先返回第一个单点有效的点。
        /// </summary>
        private bool TryGenerateRandJoint(
            Control control,
            joint escapeJoint,
            out joint randJoint)
        {
            randJoint = escapeJoint;

            double jitterRad = RandJitterDeg * Math.PI / 180.0;

            joint candidate = new joint(
                   escapeJoint.j1 + GetRandomDouble(-jitterRad, jitterRad),
                   escapeJoint.j2 + GetRandomDouble(-jitterRad , jitterRad ),
                   escapeJoint.j3 + GetRandomDouble(-jitterRad , jitterRad ),
                   escapeJoint.j4 + GetRandomDouble(-jitterRad, jitterRad),
                   escapeJoint.j5 + GetRandomDouble(-jitterRad, jitterRad),
                   escapeJoint.j6 + GetRandomDouble(-jitterRad, jitterRad),
                   escapeJoint.Sever_Gun);

            randJoint = ClampJoint(candidate);

            return true;

        }

        /// <summary>
        /// 当 escape 点逆解失败时，返回普通随机点。
        /// 范围与当前工程中 rand_node 的生成逻辑保持一致。
        /// </summary>
        public joint CreateFallbackRandomJoint(joint referenceJoint)
        {
            double gunOpen = referenceJoint.Sever_Gun;

            if (TxrrtRobotPathPlannerForm.ToolJointOpening > 0 && GunOpenSplit > 0)
            {
                gunOpen = TxrrtRobotPathPlannerForm.ToolJointOpening -
                          random.Next(0, GunOpenSplit) *
                          (TxrrtRobotPathPlannerForm.ToolJointOpening / GunOpenSplit);
            }

            double j1 = GetRandomDoubleWithLimit(referenceJoint.j1 - Math.PI / 2, referenceJoint.j1 + Math.PI / 2, 0);
            double j2 = GetRandomDoubleWithLimit(referenceJoint.j2 - Math.PI / 2, referenceJoint.j2 + Math.PI / 2, 1);
            double j3 = GetRandomDoubleWithLimit(referenceJoint.j3 - Math.PI / 2, referenceJoint.j3 + Math.PI / 2, 2);
            double j4 = GetRandomDoubleWithLimit(referenceJoint.j4 - Math.PI, referenceJoint.j4 + Math.PI, 3);
            double j5 = GetRandomDoubleWithLimit(referenceJoint.j5 - Math.PI / 2, referenceJoint.j5 + Math.PI / 2, 4);
            double j6 = GetRandomDoubleWithLimit(referenceJoint.j6 - Math.PI, referenceJoint.j6 + Math.PI, 5);

            return new joint(j1, j2, j3, j4, j5, j6, gunOpen);
        }

        /// <summary>
        /// 把关节姿态设置到机器人上。
        /// 仅用于扫描过程中的姿态更新。
        /// </summary>
        private void ApplyJoint(Control control, joint j)
        {
            using (TxPoseData robotPosture = new TxPoseData())
            {
                ArrayList arr = new ArrayList(j.ToArray());
                robotPosture.JointValues = arr;

                ArrayList sols = new ArrayList();
                sols.Add(robotPosture);

                TxRobotAPIClass.TxRobotPostureGenerate(
                    control,
                    TxrrtRobotPathPlannerForm.robot,
                    TxrrtRobotPathPlannerForm.robServerGun,
                    sols,
                    j.Sever_Gun);
            }
        }

        /// <summary>
        /// 通过正运动学把 joint 转成 point。
        /// </summary>
        private point ConvertJointToPoint(Control control, joint j)
        {
            using (TxPoseData robotPosture = new TxPoseData())
            {
                ArrayList arr = new ArrayList(j.ToArray());
                robotPosture.JointValues = arr;

                ArrayList sols = new ArrayList();
                sols.Add(robotPosture);

                TxRobotAPIClass.TxRobotPostureGenerate(
                    control,
                    TxrrtRobotPathPlannerForm.robot,
                    TxrrtRobotPathPlannerForm.robServerGun,
                    sols,
                    j.Sever_Gun);

                point p = new point(
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.Translation.X,
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.Translation.Y,
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.Translation.Z,
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    TxrrtRobotPathPlannerForm.robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                    j.Sever_Gun);

                return p;
            }
        }

        /// <summary>
        /// 围绕 TCP 局部坐标轴旋转。
        /// 
        /// 注意：
        /// 1. XYZ 保持不变；
        /// 2. 姿态做局部旋转；
        /// 3. 采用 RPY_XYZ，与当前工程保持一致；
        /// 4. 绕自身坐标轴旋转应使用：Rnew = Rcurrent * Rdelta。
        /// </summary>
        private point RotatePointAroundLocalAxis(point source, EscapeRotationAxis axis, double deltaAngleRad)
        {
            double[,] currentR = BuildRotationMatrixFromRPY(source.rx, source.ry, source.rz);
            double[,] deltaR = BuildAxisRotationMatrix(axis, deltaAngleRad);

            double[,] newR = MultiplyMatrix(currentR, deltaR);

            double rx;
            double ry;
            double rz;
            ConvertMatrixToRPY(newR, out rx, out ry, out rz);

            return new point(
                source.x,
                source.y,
                source.z,
                rx,
                ry,
                rz,
                source.Gun_Open);
        }

        /// <summary>
        /// 根据 RPY_XYZ 构造旋转矩阵。
        /// 对应 R = Rz * Ry * Rx。
        /// </summary>
        private double[,] BuildRotationMatrixFromRPY(double rx, double ry, double rz)
        {
            double cx = Math.Cos(rx);
            double sx = Math.Sin(rx);
            double cy = Math.Cos(ry);
            double sy = Math.Sin(ry);
            double cz = Math.Cos(rz);
            double sz = Math.Sin(rz);

            double[,] r = new double[3, 3];

            r[0, 0] = cz * cy;
            r[0, 1] = cz * sy * sx - sz * cx;
            r[0, 2] = cz * sy * cx + sz * sx;

            r[1, 0] = sz * cy;
            r[1, 1] = sz * sy * sx + cz * cx;
            r[1, 2] = sz * sy * cx - cz * sx;

            r[2, 0] = -sy;
            r[2, 1] = cy * sx;
            r[2, 2] = cy * cx;

            return r;
        }

        /// <summary>
        /// 根据单轴旋转角度构造增量旋转矩阵。
        /// </summary>
        private double[,] BuildAxisRotationMatrix(EscapeRotationAxis axis, double angle)
        {
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);

            double[,] r = new double[3, 3];

            if (axis == EscapeRotationAxis.X)
            {
                r[0, 0] = 1; r[0, 1] = 0; r[0, 2] = 0;
                r[1, 0] = 0; r[1, 1] = c; r[1, 2] = -s;
                r[2, 0] = 0; r[2, 1] = s; r[2, 2] = c;
            }
            else if (axis == EscapeRotationAxis.Y)
            {
                r[0, 0] = c; r[0, 1] = 0; r[0, 2] = s;
                r[1, 0] = 0; r[1, 1] = 1; r[1, 2] = 0;
                r[2, 0] = -s; r[2, 1] = 0; r[2, 2] = c;
            }
            else
            {
                r[0, 0] = c; r[0, 1] = -s; r[0, 2] = 0;
                r[1, 0] = s; r[1, 1] = c; r[1, 2] = 0;
                r[2, 0] = 0; r[2, 1] = 0; r[2, 2] = 1;
            }

            return r;
        }

        /// <summary>
        /// 3x3 矩阵乘法。
        /// </summary>
        private double[,] MultiplyMatrix(double[,] a, double[,] b)
        {
            double[,] r = new double[3, 3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    r[i, j] =
                        a[i, 0] * b[0, j] +
                        a[i, 1] * b[1, j] +
                        a[i, 2] * b[2, j];
                }
            }

            return r;
        }

        /// <summary>
        /// 旋转矩阵转回 RPY_XYZ。
        /// 与 BuildRotationMatrixFromRPY 保持一致。
        /// </summary>
        private void ConvertMatrixToRPY(double[,] r, out double rx, out double ry, out double rz)
        {
            double value = -r[2, 0];

            if (value > 1.0) value = 1.0;
            if (value < -1.0) value = -1.0;

            ry = Math.Asin(value);
            double cy = Math.Cos(ry);

            if (Math.Abs(cy) > 1e-8)
            {
                rx = Math.Atan2(r[2, 1], r[2, 2]);
                rz = Math.Atan2(r[1, 0], r[0, 0]);
            }
            else
            {
                rx = 0.0;
                rz = Math.Atan2(-r[0, 1], r[1, 1]);
            }
        }

        /// <summary>
        /// 判断碰撞点是否为零向量。
        /// Collision_CheckPoint 在未碰撞时返回 (0,0,0)。
        /// </summary>
        private bool IsZeroVector(TxVector v)
        {
            if (v == null)
            {
                return true;
            }

            const double EPS = 1e-8;
            return Math.Abs(v.X) < EPS && Math.Abs(v.Y) < EPS && Math.Abs(v.Z) < EPS;
        }

        /// <summary>
        /// 把 joint 限制在软限位内。
        /// </summary>
        private joint ClampJoint(joint q)
        {
            if (JointLimits == null)
            {
                return q;
            }

            double[] clamped = JointLimits.Clamp(q.ToArray());

            return new joint(
                clamped[0],
                clamped[1],
                clamped[2],
                clamped[3],
                clamped[4],
                clamped[5],
                q.Sever_Gun);
        }

        /// <summary>
        /// 判断 joint 是否在软限位范围内。
        /// </summary>
        private bool IsInsideJointLimits(joint q)
        {
            if (JointLimits == null)
            {
                return true;
            }

            double[] arr = q.ToArray();

            for (int i = 0; i < 6; i++)
            {
                if (arr[i] < JointLimits.Lower[i] || arr[i] > JointLimits.Upper[i])
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// 在给定范围内生成随机数。
        /// </summary>
        private double GetRandomDouble(double minValue, double maxValue)
        {
            return random.NextDouble() * (maxValue - minValue) + minValue;
        }

        /// <summary>
        /// 在关节限位内生成随机值。
        /// </summary>
        private double GetRandomDoubleWithLimit(double minValue, double maxValue, int jointIndex)
        {
            if (JointLimits != null)
            {
                if (minValue < JointLimits.Lower[jointIndex]) minValue = JointLimits.Lower[jointIndex];
                if (maxValue > JointLimits.Upper[jointIndex]) maxValue = JointLimits.Upper[jointIndex];
            }

            return GetRandomDouble(minValue, maxValue);
        }
    }
}