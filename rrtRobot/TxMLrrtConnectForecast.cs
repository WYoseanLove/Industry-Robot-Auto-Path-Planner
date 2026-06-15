using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace rrtRobot
{
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
    /// 历史逃逸训练样本。
    /// Features: 当前局部环境特征。
    /// Label: 历史成功路径中的逃逸方向。
    /// </summary>
    public class OutletTrainingSample
    {
        public double[] Features { get; set; }
        public double[] Label { get; set; }
    }

    /// <summary>
    /// 出口吸引点预测结果。
    /// RandLikeAttractor 是最重要的输出：
    /// 它的作用等同于 rand_node.loc，用于决定哪个成功节点被 Nearest_Node 选中扩展。
    /// </summary>
    public class OutletAttractorResult
    {
        public bool Success { get; set; }

        /// <summary>
        /// 像 rand_node.loc 一样使用的出口吸引点。
        /// </summary>
        public joint RandLikeAttractor { get; set; }

        /// <summary>
        /// 被认为最靠近出口方向的边界成功节点。
        /// </summary>
        public joint ExitBoundaryJoint { get; set; }

        /// <summary>
        /// 边界节点在 currentTree 中的索引。
        /// 这个索引可以作为调试使用，也可以直接拿来扩展。
        /// </summary>
        public int BoundaryNodeIndex { get; set; }

        /// <summary>
        /// 从边界节点指向出口吸引点的方向。
        /// </summary>
        public double[] ExitDirection { get; set; }

        public double Score { get; set; }

        public double Confidence { get; set; }

        public string Reason { get; set; }

        /// <summary>
        /// 当前候选所属的方向槽位。
        /// </summary>
        public int DirectionSlotId { get; set; }
    }

    internal class OutletBoundaryCandidate
    {
        public int NodeIndex { get; set; }
        public joint BoundaryJoint { get; set; }
        public double[] DirectionFromRoot { get; set; }
        public double DistanceFromRoot { get; set; }

        /// <summary>
        /// 方向槽位 ID。边界重算后重新分配。
        /// </summary>
        public int DirectionSlotId { get; set; }
    }

    internal class OutletDirectionStats
    {
        public int DirectionSlotId { get; set; }

        public int SelectCount { get; set; }

        public int CycleCount { get; set; }

        public double LastScore { get; set; }

        public double AvgScore { get; set; }

        public double[] RepresentativeDirection { get; set; }

        /// <summary>
        /// 上一个周期该方向边界点到 root 的 joint distance。
        /// </summary>
        public double PreviousBoundaryDistance { get; set; }

        /// <summary>
        /// 当前周期该方向边界点到 root 的 joint distance。
        /// </summary>
        public double CurrentBoundaryDistance { get; set; }

        /// <summary>
        /// 本周期相对上一周期的扩展长度。
        /// </summary>
        public double LastExpansionGain { get; set; }

        /// <summary>
        /// 扩展长度的历史平均值。
        /// </summary>
        public double AvgExpansionGain { get; set; }

        public int RefreshCount { get; set; }
    }

    internal class OutletBoundaryRefreshState
    {
        public int CallCountSinceRefresh { get; set; }

        public List<OutletBoundaryCandidate> CachedCandidates { get; set; } =
            new List<OutletBoundaryCandidate>();

        public Dictionary<int, OutletDirectionStats> DirectionStats { get; set; } =
            new Dictionary<int, OutletDirectionStats>();
    }

    /// <summary>
    /// 机器学习 + 启发式的出口吸引点生成器。
    /// 
    /// 核心作用：
    /// 1. 从当前成功树中找边界节点。
    /// 2. 判断哪个边界方向更像出口。
    /// 3. 生成一个位于出口附近的吸引点 RandLikeAttractor。
    /// 4. 这个吸引点可以直接替代 rand_node.loc，用于选择扩展节点。
    /// </summary>
    public class EscapeOutletAttractorLearner
    {
        /// <summary>
        /// 最近障碍点数量。
        /// </summary>
        public int KObs { get; set; } = 10;

        /// <summary>
        /// 最近成功点数量。
        /// </summary>
        public int KSuccess { get; set; } = 10;

        /// <summary>
        /// KNN 使用的历史样本数量。
        /// </summary>
        public int KExperience { get; set; } = 25;

        /// <summary>
        /// 至少多少个历史样本后才启用 KNN 经验学习。
        /// </summary>
        public int MinLearningSamples { get; set; } = 30;

        /// <summary>
        /// 出口吸引点距离边界节点的距离，单位是 stepSize 的倍数。
        /// 建议 4 到 8。
        /// </summary>
        public double AttractorDistanceSteps { get; set; } = 6.0;

        /// <summary>
        /// 角度聚类阈值，单位为度。
        /// 用于找各个方向上的边界节点。
        /// </summary>
        public double BoundaryClusterAngleDegree { get; set; } = 10.0;

        /// <summary>
        /// 关节角归一化尺度。
        /// </summary>
        public double AngleScale { get; set; } = Math.PI;

        /// <summary>
        /// 障碍点排斥影响半径，单位是 stepSize 的倍数。
        /// </summary>
        public double ObstacleInfluenceSteps { get; set; } = 4.0;

        /// <summary>
        /// 边界点重算周期。每经过这么多次预测调用，重新提取一次边界。
        /// </summary>
        public int BoundaryRefreshInterval { get; set; } = 100;

        /// <summary>
        /// 历史经验库。
        /// </summary>
        private readonly List<OutletTrainingSample> memory = new List<OutletTrainingSample>();

        private readonly OutletBoundaryRefreshState startBoundaryState =
            new OutletBoundaryRefreshState();

        private readonly OutletBoundaryRefreshState endBoundaryState =
            new OutletBoundaryRefreshState();

        private static readonly Random outletRandom = new Random();

        public int SampleCount
        {
            get { return memory.Count; }
        }

        public void ClearMemory()
        {
            memory.Clear();

            startBoundaryState.CallCountSinceRefresh = 0;
            startBoundaryState.CachedCandidates.Clear();
            startBoundaryState.DirectionStats.Clear();

            endBoundaryState.CallCountSinceRefresh = 0;
            endBoundaryState.CachedCandidates.Clear();
            endBoundaryState.DirectionStats.Clear();
        }

        /// <summary>
        /// 主函数：预测一个像 rand_node.loc 一样使用的出口吸引点。
        /// 
        /// currentTree:
        ///     当前正在扩展的树，例如 start_nodes 或 end_nodes。
        /// qGoal:
        ///     当前树想靠近的目标。start树传 p_end，end树传 p_start。
        /// qOtherTreeReference:
        ///     对侧树的参考点。可以传对侧树根节点，也可以传对侧树中离当前树最近的点。
        /// obsPoints:
        ///     碰撞失败点列表，即你的 obs。
        /// stepSize:
        ///     当前树步长。
        /// isStartTree:
        ///     当前是否为起始树。
        /// limits:
        ///     关节限位。
        public OutletAttractorResult PredictOutletAttractor(
                 List<Node3D_joint> currentTree,
                 joint qGoal,
                 joint qOtherTreeReference,
                 IList<joint> obsPoints,
                 double stepSize,
                 bool isStartTree,
                 JointLimitBox limits = null)
        {
            if (currentTree == null || currentTree.Count == 0)
            {
                return new OutletAttractorResult
                {
                    Success = false,
                    Reason = "Current tree is empty."
                };
            }

            if (obsPoints == null)
            {
                obsPoints = new List<joint>();
            }

            joint root = currentTree[0].loc;

            List<joint> successPoints = currentTree
                .Where(n => n != null)
                .Select(n => n.loc)
                .ToList();

            OutletBoundaryRefreshState boundaryState =
                isStartTree ? startBoundaryState : endBoundaryState;

            List<OutletBoundaryCandidate> boundaryCandidates =
                GetOrRefreshBoundaryCandidates(
                    currentTree,
                    root,
                    boundaryState);

            if (boundaryCandidates.Count == 0)
            {
                return FallbackAttractorFromRoot(
                    root,
                    qGoal,
                    qOtherTreeReference,
                    successPoints,
                    obsPoints,
                    stepSize,
                    isStartTree,
                    limits,
                    "No boundary candidates found. Fallback from root.");
            }

            // ============================================================
            // 修改点：
            // 原来只保存 bestResult；
            // 现在保存所有成功生成的候选吸引点，然后按 Score 排序，
            // 最后从前三名中随机返回一个。
            // ============================================================
            List<OutletAttractorResult> validResults =
                new List<OutletAttractorResult>();

            foreach (OutletBoundaryCandidate candidate in boundaryCandidates)
            {
                OutletAttractorResult result = BuildAndScoreAttractorFromBoundary(
                    candidate,
                    root,
                    qGoal,
                    qOtherTreeReference,
                    successPoints,
                    obsPoints,
                    stepSize,
                    isStartTree,
                    limits,
                    boundaryState);

                if (result != null)
                {
                    validResults.Add(result);
                }
            }

            if (validResults.Count == 0)
            {
                return FallbackAttractorFromRoot(
                    root,
                    qGoal,
                    qOtherTreeReference,
                    successPoints,
                    obsPoints,
                    stepSize,
                    isStartTree,
                    limits,
                    "All boundary candidates failed. Fallback from root.");
            }

            // 按分数从高到低排序
            List<OutletAttractorResult> sortedResults = validResults
                .OrderByDescending(r => r.Score)
                .ToList();

            // 取前三名。如果不足三个，就取实际数量。
            int topCount = Math.Min(3, sortedResults.Count);

            List<OutletAttractorResult> topResults = sortedResults
                .Take(topCount)
                .ToList();

            // 从前三名中随机选一个
            int selectedIndex = outletRandom.Next(0, topResults.Count);

            OutletAttractorResult selectedResult = topResults[selectedIndex];

            selectedResult.Success = true;
            selectedResult.Reason =
                "Outlet attractor randomly selected from top " +
                topCount.ToString() +
                " scored boundary candidates. Rank=" +
                (selectedIndex + 1).ToString() +
                ", Score=" +
                selectedResult.Score.ToString("F4");

            UpdateDirectionStats(
                boundaryState,
                selectedResult.DirectionSlotId,
                selectedResult.Score);

            return selectedResult;
        }

        /// <summary>
        /// 从最终成功路径中增加学习样本。
        /// 推荐在规划成功后调用。
        /// </summary>
        public int AddExperienceFromFinalPath(
            List<joint> finalPath,
            IList<joint> obsPoints,
            joint goal,
            bool isStartTree,
            int lookAhead = 5)
        {
            if (finalPath == null || finalPath.Count < 2)
            {
                return 0;
            }

            if (obsPoints == null)
            {
                obsPoints = new List<joint>();
            }

            lookAhead = Math.Max(1, lookAhead);

            int added = 0;
            List<joint> successPoints = new List<joint>(finalPath);

            for (int i = 0; i < finalPath.Count - 1; i++)
            {
                int targetIndex = Math.Min(i + lookAhead, finalPath.Count - 1);

                joint qCurrent = finalPath[i];
                joint qFuture = finalPath[targetIndex];

                double[] label = NormalizeSafe(Subtract(qFuture, qCurrent));

                if (Norm(label) < 1e-9)
                {
                    continue;
                }

                double[] features = BuildFeatures(
                    qCurrent,
                    goal,
                    successPoints,
                    obsPoints,
                    isStartTree);

                memory.Add(new OutletTrainingSample
                {
                    Features = features,
                    Label = label
                });

                added++;
            }

            return added;
        }

        /// <summary>
        /// 从整棵树中增加经验。
        /// 这个也可以用，但优先级低于最终路径。
        /// </summary>
        public int AddExperienceFromTree(
            List<Node3D_joint> tree,
            IList<joint> obsPoints,
            joint goal,
            bool isStartTree)
        {
            if (tree == null || tree.Count < 2)
            {
                return 0;
            }

            if (obsPoints == null)
            {
                obsPoints = new List<joint>();
            }

            List<joint> successPoints = tree
                .Where(n => n != null)
                .Select(n => n.loc)
                .ToList();

            int added = 0;

            for (int i = 1; i < tree.Count; i++)
            {
                Node3D_joint node = tree[i];

                if (node == null || node.parent == null)
                {
                    continue;
                }

                joint parent = node.parent.loc;
                joint child = node.loc;

                double[] label = NormalizeSafe(Subtract(child, parent));

                if (Norm(label) < 1e-9)
                {
                    continue;
                }

                double[] features = BuildFeatures(
                    parent,
                    goal,
                    successPoints,
                    obsPoints,
                    isStartTree);

                memory.Add(new OutletTrainingSample
                {
                    Features = features,
                    Label = label
                });

                added++;
            }

            return added;
        }

        public void SaveMemoryToCsv(string filePath)
        {
            using (StreamWriter sw = new StreamWriter(filePath, false))
            {
                foreach (OutletTrainingSample sample in memory)
                {
                    List<string> values = new List<string>();

                    values.AddRange(sample.Features.Select(v => v.ToString("G17", CultureInfo.InvariantCulture)));
                    values.AddRange(sample.Label.Select(v => v.ToString("G17", CultureInfo.InvariantCulture)));

                    sw.WriteLine(string.Join(",", values));
                }
            }
        }

        public int LoadMemoryFromCsv(string filePath)
        {
            if (!File.Exists(filePath))
            {
                return 0;
            }

            int loaded = 0;

            foreach (string line in File.ReadLines(filePath))
            {
                if (string.IsNullOrWhiteSpace(line))
                {
                    continue;
                }

                string[] parts = line.Split(',');

                if (parts.Length <= 6)
                {
                    continue;
                }

                double[] values = new double[parts.Length];

                bool ok = true;

                for (int i = 0; i < parts.Length; i++)
                {
                    if (!double.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]))
                    {
                        ok = false;
                        break;
                    }
                }

                if (!ok)
                {
                    continue;
                }

                int featureLength = values.Length - 6;

                double[] features = new double[featureLength];
                double[] label = new double[6];

                Array.Copy(values, 0, features, 0, featureLength);
                Array.Copy(values, featureLength, label, 0, 6);

                label = NormalizeSafe(label);

                if (Norm(label) < 1e-9)
                {
                    continue;
                }

                memory.Add(new OutletTrainingSample
                {
                    Features = features,
                    Label = label
                });

                loaded++;
            }

            return loaded;
        }

        private List<OutletBoundaryCandidate> GetOrRefreshBoundaryCandidates(
            List<Node3D_joint> tree,
            joint root,
            OutletBoundaryRefreshState state)
        {
            state.CallCountSinceRefresh++;

            bool needRefresh =
                state.CachedCandidates == null ||
                state.CachedCandidates.Count == 0 ||
                state.CallCountSinceRefresh >= BoundaryRefreshInterval;

            if (needRefresh)
            {
                List<OutletBoundaryCandidate> refreshed =
                    FindBoundaryCandidatesByAngularClustering(tree, root);

                AssignDirectionSlotsAndUpdateStats(state, refreshed);

                state.CachedCandidates = refreshed;
                state.CallCountSinceRefresh = 0;
            }

            return state.CachedCandidates;
        }

        private void AssignDirectionSlotsAndUpdateStats(
            OutletBoundaryRefreshState state,
            List<OutletBoundaryCandidate> refreshedCandidates)
        {
            Dictionary<int, OutletDirectionStats> oldStats = state.DirectionStats;
            Dictionary<int, OutletDirectionStats> newStats =
                new Dictionary<int, OutletDirectionStats>();
            HashSet<int> usedOldSlots = new HashSet<int>();

            int nextSlotId = oldStats.Count == 0 ? 0 : oldStats.Keys.Max() + 1;
            double cosThreshold = Math.Cos(BoundaryClusterAngleDegree * Math.PI / 180.0);

            foreach (OutletBoundaryCandidate candidate in refreshedCandidates)
            {
                int matchedSlotId = -1;
                double bestDot = double.MinValue;

                foreach (KeyValuePair<int, OutletDirectionStats> pair in oldStats)
                {
                    if (usedOldSlots.Contains(pair.Key))
                    {
                        continue;
                    }

                    if (pair.Value.RepresentativeDirection == null)
                    {
                        continue;
                    }

                    double dot = Dot(candidate.DirectionFromRoot, pair.Value.RepresentativeDirection);

                    if (dot > bestDot)
                    {
                        bestDot = dot;
                        matchedSlotId = pair.Key;
                    }
                }

                bool matched = matchedSlotId >= 0 && bestDot >= cosThreshold;

                if (!matched)
                {
                    matchedSlotId = nextSlotId;
                    nextSlotId++;
                }

                candidate.DirectionSlotId = matchedSlotId;

                OutletDirectionStats oldSlotStats = null;
                oldStats.TryGetValue(matchedSlotId, out oldSlotStats);

                double previousBoundaryDistance =
                    oldSlotStats == null
                        ? candidate.DistanceFromRoot
                        : oldSlotStats.CurrentBoundaryDistance;

                double currentBoundaryDistance = candidate.DistanceFromRoot;
                double lastExpansionGain = currentBoundaryDistance - previousBoundaryDistance;

                int refreshCount = oldSlotStats == null ? 1 : oldSlotStats.RefreshCount + 1;

                double avgExpansionGain =
                    oldSlotStats == null
                        ? lastExpansionGain
                        : ((oldSlotStats.AvgExpansionGain * oldSlotStats.RefreshCount) + lastExpansionGain) / refreshCount;

                OutletDirectionStats newSlotStats = new OutletDirectionStats
                {
                    DirectionSlotId = matchedSlotId,
                    SelectCount = oldSlotStats == null ? 0 : oldSlotStats.SelectCount,
                    CycleCount = oldSlotStats == null ? 0 : oldSlotStats.CycleCount,
                    LastScore = oldSlotStats == null ? 0.0 : oldSlotStats.LastScore,
                    AvgScore = oldSlotStats == null ? 0.0 : oldSlotStats.AvgScore,
                    RepresentativeDirection = candidate.DirectionFromRoot,
                    PreviousBoundaryDistance = previousBoundaryDistance,
                    CurrentBoundaryDistance = currentBoundaryDistance,
                    LastExpansionGain = lastExpansionGain,
                    AvgExpansionGain = avgExpansionGain,
                    RefreshCount = refreshCount
                };

                newStats[matchedSlotId] = newSlotStats;
                usedOldSlots.Add(matchedSlotId);
            }

            state.DirectionStats = newStats;
        }

        private void UpdateDirectionStats(
            OutletBoundaryRefreshState state,
            int directionSlotId,
            double score)
        {
            if (directionSlotId < 0)
            {
                return;
            }

            OutletDirectionStats stats;
            if (!state.DirectionStats.TryGetValue(directionSlotId, out stats))
            {
                stats = new OutletDirectionStats
                {
                    DirectionSlotId = directionSlotId
                };
                state.DirectionStats[directionSlotId] = stats;
            }

            stats.SelectCount++;
            stats.CycleCount++;
            stats.LastScore = score;

            if (stats.CycleCount == 1)
            {
                stats.AvgScore = score;
            }
            else
            {
                stats.AvgScore =
                    ((stats.AvgScore * (stats.CycleCount - 1)) + score) /
                    stats.CycleCount;
            }
        }

        private List<OutletBoundaryCandidate> FindBoundaryCandidatesByAngularClustering(
            List<Node3D_joint> tree,
            joint root)
        {
            List<OutletBoundaryCandidate> candidates = new List<OutletBoundaryCandidate>();

            if (tree == null || tree.Count <= 1)
            {
                return candidates;
            }

            double angleRad = BoundaryClusterAngleDegree * Math.PI / 180.0;
            double cosThreshold = Math.Cos(angleRad);

            foreach (var pair in tree.Select((node, index) => new { Node = node, Index = index }))
            {
                if (pair.Node == null)
                {
                    continue;
                }

                double[] delta = Subtract(pair.Node.loc, root);
                double distance = Norm(delta);

                if (distance < 1e-9)
                {
                    continue;
                }

                double[] direction = NormalizeSafe(delta);

                bool merged = false;

                for (int i = 0; i < candidates.Count; i++)
                {
                    double dot = Dot(direction, candidates[i].DirectionFromRoot);

                    if (dot >= cosThreshold)
                    {
                        // 同一个角度簇中，保留距离 root 最远的节点，作为该方向边界。
                        if (distance > candidates[i].DistanceFromRoot)
                        {
                            candidates[i] = new OutletBoundaryCandidate
                            {
                                NodeIndex = pair.Index,
                                BoundaryJoint = pair.Node.loc,
                                DirectionFromRoot = direction,
                                DistanceFromRoot = distance,
                                DirectionSlotId = candidates[i].DirectionSlotId
                            };
                        }

                        merged = true;
                        break;
                    }
                }

                if (!merged)
                {
                    candidates.Add(new OutletBoundaryCandidate
                    {
                        NodeIndex = pair.Index,
                        BoundaryJoint = pair.Node.loc,
                        DirectionFromRoot = direction,
                        DistanceFromRoot = distance,
                        DirectionSlotId = candidates.Count
                    });
                }
            }

            return candidates;
        }

        private OutletAttractorResult BuildAndScoreAttractorFromBoundary(
            OutletBoundaryCandidate candidate,
            joint root,
            joint qGoal,
            joint qOtherTreeReference,
            List<joint> successPoints,
            IList<joint> obsPoints,
            double stepSize,
            bool isStartTree,
    JointLimitBox limits,
    OutletBoundaryRefreshState boundaryState)
        {
            joint qBoundary = candidate.BoundaryJoint;

            double[] dOutward = candidate.DirectionFromRoot;
            double[] dGoal = NormalizeSafe(Subtract(qGoal, qBoundary));
            double[] dOther = NormalizeSafe(Subtract(qOtherTreeReference, qBoundary));
            double[] dRepulse = ComputeObstacleRepulsiveDirection(qBoundary, obsPoints, stepSize);

            double learningConfidence;
            double[] dLearn = PredictLearningDirection(
                qBoundary,
                qGoal,
                successPoints,
                obsPoints,
                isStartTree,
                out learningConfidence);

            bool hasLearning = dLearn != null && Norm(dLearn) > 1e-9;

            OutletDirectionStats directionStats = null;
            if (boundaryState != null)
            {
                boundaryState.DirectionStats.TryGetValue(candidate.DirectionSlotId, out directionStats);
            }

            double lastExpansionGain =
                directionStats == null ? 0.0 : directionStats.LastExpansionGain;

            double avgExpansionGain =
                directionStats == null ? 0.0 : directionStats.AvgExpansionGain;

            double[] dExit = new double[6];

            for (int i = 0; i < 6; i++)
            {
                dExit[i] =
                    0.5 * dOutward[i] +
                    0.10 * dGoal[i] +
                    0.35 * dOther[i] +
                    0.40 * dRepulse[i] +
                    (hasLearning ? 0.45 * dLearn[i] : 0.0);
            }

            dExit = NormalizeSafe(dExit);

            if (Norm(dExit) < 1e-9)
            {
                return null;
            }

            double attractorDistance = Math.Max(stepSize, AttractorDistanceSteps * stepSize);

            double[] q = qBoundary.ToArray();

            for (int i = 0; i < 6; i++)
            {
                q[i] += attractorDistance * dExit[i];
            }

            if (limits != null)
            {
                q = limits.Clamp(q);
            }

            joint qAttractor = new joint(
                q[0], q[1], q[2],
                q[3], q[4], q[5],
                qBoundary.Sever_Gun);

            double score = ScoreAttractor(
                root,
                qBoundary,
                qAttractor,
                qGoal,
                qOtherTreeReference,
                obsPoints,
                candidate.DistanceFromRoot,
                learningConfidence,
                lastExpansionGain,
                avgExpansionGain);
            double[] q_checkneared = qBoundary.ToArray();

            for (int i = 0; i < 6; i++)
            {
                q_checkneared[i] += stepSize * dExit[i];
            }

            if (limits != null)
            {
                q_checkneared = limits.Clamp(q_checkneared);
            }

            joint qAttractor_neared = new joint(
                q_checkneared[0], q_checkneared[1], q_checkneared[2],
                q_checkneared[3], q_checkneared[4], q_checkneared[5],
                qBoundary.Sever_Gun);

            int nearbySuccessCount = 0;

            if (successPoints != null && successPoints.Count > 0)
            {
                foreach (joint successPoint in successPoints)
                {
                    if (Dist(qAttractor_neared, successPoint) <= stepSize)
                    {
                        nearbySuccessCount++;
                    }
                }
            }

            if (nearbySuccessCount > 5)
            {
                score *= 0.1;
            }

            return new OutletAttractorResult
            {
                Success = true,
                RandLikeAttractor = qAttractor,
                ExitBoundaryJoint = qBoundary,
                BoundaryNodeIndex = candidate.NodeIndex,
                ExitDirection = dExit,
                Score = score,
                Confidence = learningConfidence,
                Reason = "Boundary candidate scored.",
                DirectionSlotId = candidate.DirectionSlotId
            };
        }

        private double ScoreAttractor(
            joint root,
            joint qBoundary,
            joint qAttractor,
            joint qGoal,
            joint qOtherTreeReference,
            IList<joint> obsPoints,
            double boundaryDistanceFromRoot,
            double learningConfidence,
            double lastExpansionGain,
            double avgExpansionGain)
        {
            double clearance = EstimateClearance(qAttractor, obsPoints);

            double goalProgress =
                Dist(qBoundary, qGoal) - Dist(qAttractor, qGoal);

            double otherTreeProgress =
                Dist(qBoundary, qOtherTreeReference) - Dist(qAttractor, qOtherTreeReference);

            double obstacleDensityPenalty =
                CountNearbyObstacles(qAttractor, obsPoints, AngleScale / 8.0);

            double boundaryFactor = boundaryDistanceFromRoot / AngleScale;

            // 扩展更快的评价准则：
            // 在下一次重算边界时，看该方向簇新的边界点相比上一周期边界点，
            // 距 root 的 joint distance 增加了多少。
            double expansionGainScore = Math.Max(0.0, lastExpansionGain) / AngleScale;
            double avgExpansionGainScore = Math.Max(0.0, avgExpansionGain) / AngleScale;

            double score =
                2.00 * clearance / AngleScale +
                1.20 * goalProgress / AngleScale +
                1.00 * otherTreeProgress / AngleScale +
                0.60 * boundaryFactor +
                1.50 * learningConfidence +
                1.20 * expansionGainScore +
                0.80 * avgExpansionGainScore -
                0.30 * obstacleDensityPenalty;

            return score;
        }

        private OutletAttractorResult FallbackAttractorFromRoot(
            joint root,
            joint qGoal,
            joint qOtherTreeReference,
            List<joint> successPoints,
            IList<joint> obsPoints,
            double stepSize,
            bool isStartTree,
            JointLimitBox limits,
            string reason)
        {
            double learningConfidence;
            double[] dLearn = PredictLearningDirection(
                root,
                qGoal,
                successPoints,
                obsPoints,
                isStartTree,
                out learningConfidence);

            double[] dGoal = NormalizeSafe(Subtract(qGoal, root));
            double[] dOther = NormalizeSafe(Subtract(qOtherTreeReference, root));
            double[] dRepulse = ComputeObstacleRepulsiveDirection(root, obsPoints, stepSize);

            double[] d = new double[6];

            for (int i = 0; i < 6; i++)
            {
                d[i] =
                    0.40 * dGoal[i] +
                    0.40 * dOther[i] +
                    0.30 * dRepulse[i] +
                    (dLearn == null ? 0.0 : 0.50 * dLearn[i]);
            }

            d = NormalizeSafe(d);

            if (Norm(d) < 1e-9)
            {
                d = new double[] { 1, 0, 0, 0, 0, 0 };
            }

            double distance = Math.Max(stepSize, AttractorDistanceSteps * stepSize);

            double[] q = root.ToArray();

            for (int i = 0; i < 6; i++)
            {
                q[i] += distance * d[i];
            }

            if (limits != null)
            {
                q = limits.Clamp(q);
            }

            joint qAttractor = new joint(
                q[0], q[1], q[2],
                q[3], q[4], q[5],
                root.Sever_Gun);

            return new OutletAttractorResult
            {
                Success = true,
                RandLikeAttractor = qAttractor,
                ExitBoundaryJoint = root,
                BoundaryNodeIndex = 0,
                ExitDirection = d,
                Score = 0.0,
                Confidence = learningConfidence,
                Reason = reason,
                DirectionSlotId = -1
            };
        }

        private double[] PredictLearningDirection(
            joint qCurrent,
            joint qGoal,
            List<joint> successPoints,
            IList<joint> obsPoints,
            bool isStartTree,
            out double confidence)
        {
            confidence = 0.0;

            if (memory.Count < MinLearningSamples)
            {
                return null;
            }

            double[] features = BuildFeatures(
                qCurrent,
                qGoal,
                successPoints,
                obsPoints,
                isStartTree);

            List<Tuple<double, OutletTrainingSample>> nearest = memory
                .Select(s => new Tuple<double, OutletTrainingSample>(
                    FeatureDistanceSquared(features, s.Features),
                    s))
                .OrderBy(t => t.Item1)
                .Take(KExperience)
                .ToList();

            if (nearest.Count == 0)
            {
                return null;
            }

            double[] d = new double[6];
            double totalWeight = 0.0;
            double totalDistance = 0.0;

            foreach (var item in nearest)
            {
                double distance = Math.Sqrt(item.Item1);
                double weight = 1.0 / (distance + 1e-6);

                for (int i = 0; i < 6; i++)
                {
                    d[i] += weight * item.Item2.Label[i];
                }

                totalWeight += weight;
                totalDistance += distance;
            }

            if (totalWeight < 1e-12)
            {
                return null;
            }

            for (int i = 0; i < 6; i++)
            {
                d[i] /= totalWeight;
            }

            d = NormalizeSafe(d);

            double avgDistance = totalDistance / nearest.Count;
            confidence = 1.0 / (1.0 + avgDistance);

            return d;
        }

        private double[] BuildFeatures(
            joint qCurrent,
            joint qGoal,
            List<joint> successPoints,
            IList<joint> obsPoints,
            bool isStartTree)
        {
            List<double> features = new List<double>();

            double[] q = qCurrent.ToArray();

            for (int i = 0; i < 6; i++)
            {
                features.Add(q[i] / AngleScale);
            }

            double[] goalDelta = Subtract(qGoal, qCurrent);

            for (int i = 0; i < 6; i++)
            {
                features.Add(goalDelta[i] / AngleScale);
            }

            List<joint> nearestObs = obsPoints
                .OrderBy(o => Dist(qCurrent, o))
                .Take(KObs)
                .ToList();

            for (int i = 0; i < KObs; i++)
            {
                if (i < nearestObs.Count)
                {
                    double[] delta = Subtract(nearestObs[i], qCurrent);
                    double d = Dist(qCurrent, nearestObs[i]);

                    for (int j = 0; j < 6; j++)
                    {
                        features.Add(delta[j] / AngleScale);
                    }

                    features.Add(d / AngleScale);
                }
                else
                {
                    for (int j = 0; j < 7; j++)
                    {
                        features.Add(0.0);
                    }
                }
            }

            List<joint> nearestSuccess = successPoints
                .Where(s => Dist(qCurrent, s) > 1e-9)
                .OrderBy(s => Dist(qCurrent, s))
                .Take(KSuccess)
                .ToList();

            for (int i = 0; i < KSuccess; i++)
            {
                if (i < nearestSuccess.Count)
                {
                    double[] delta = Subtract(nearestSuccess[i], qCurrent);
                    double d = Dist(qCurrent, nearestSuccess[i]);

                    for (int j = 0; j < 6; j++)
                    {
                        features.Add(delta[j] / AngleScale);
                    }

                    features.Add(d / AngleScale);
                }
                else
                {
                    for (int j = 0; j < 7; j++)
                    {
                        features.Add(0.0);
                    }
                }
            }

            features.Add(Dist(qCurrent, qGoal) / AngleScale);

            double minObs = obsPoints.Count > 0
                ? obsPoints.Min(o => Dist(qCurrent, o))
                : AngleScale;

            features.Add(minObs / AngleScale);

            features.Add(isStartTree ? 1.0 : 0.0);

            return features.ToArray();
        }

        private double[] ComputeObstacleRepulsiveDirection(
            joint q,
            IList<joint> obsPoints,
            double stepSize)
        {
            double[] d = new double[6];

            if (obsPoints == null || obsPoints.Count == 0)
            {
                return d;
            }

            double radius = Math.Max(stepSize, ObstacleInfluenceSteps * stepSize);

            List<joint> nearObs = obsPoints
                .OrderBy(o => Dist(q, o))
                .Take(KObs)
                .ToList();

            foreach (joint o in nearObs)
            {
                double dist = Dist(q, o);

                if (dist < 1e-9)
                {
                    continue;
                }

                if (dist > radius)
                {
                    continue;
                }

                double[] away = NormalizeSafe(Subtract(q, o));

                double weight = (radius - dist) / radius;
                weight = weight * weight / (dist + 1e-6);

                for (int i = 0; i < 6; i++)
                {
                    d[i] += weight * away[i];
                }
            }

            return NormalizeSafe(d);
        }

        private double EstimateClearance(joint q, IList<joint> obsPoints)
        {
            if (obsPoints == null || obsPoints.Count == 0)
            {
                return AngleScale;
            }

            return obsPoints.Min(o => Dist(q, o));
        }

        private int CountNearbyObstacles(joint q, IList<joint> obsPoints, double radius)
        {
            if (obsPoints == null || obsPoints.Count == 0)
            {
                return 0;
            }

            int count = 0;

            foreach (joint o in obsPoints)
            {
                if (Dist(q, o) < radius)
                {
                    count++;
                }
            }

            return count;
        }

        private double FeatureDistanceSquared(double[] a, double[] b)
        {
            int n = Math.Min(a.Length, b.Length);
            double sum = 0.0;

            for (int i = 0; i < n; i++)
            {
                double d = a[i] - b[i];
                sum += d * d;
            }

            int diff = Math.Abs(a.Length - b.Length);

            if (diff > 0)
            {
                sum += diff;
            }

            return sum;
        }

        private static double[] Subtract(joint a, joint b)
        {
            return new double[]
            {
                a.j1 - b.j1,
                a.j2 - b.j2,
                a.j3 - b.j3,
                a.j4 - b.j4,
                a.j5 - b.j5,
                a.j6 - b.j6
            };
        }

        private static double Dist(joint a, joint b)
        {
            double[] da = a.ToArray();
            double[] db = b.ToArray();

            double sum = 0.0;

            for (int i = 0; i < 6; i++)
            {
                double d = da[i] - db[i];
                sum += d * d;
            }

            return Math.Sqrt(sum);
        }

        private static double Dot(double[] a, double[] b)
        {
            int n = Math.Min(a.Length, b.Length);
            double sum = 0.0;

            for (int i = 0; i < n; i++)
            {
                sum += a[i] * b[i];
            }

            return sum;
        }

        private static double Norm(double[] v)
        {
            if (v == null)
            {
                return 0.0;
            }

            double sum = 0.0;

            for (int i = 0; i < v.Length; i++)
            {
                sum += v[i] * v[i];
            }

            return Math.Sqrt(sum);
        }

        private static double[] NormalizeSafe(double[] v)
        {
            if (v == null)
            {
                return null;
            }

            double n = Norm(v);

            if (n < 1e-12)
            {
                return new double[v.Length];
            }

            double[] r = new double[v.Length];

            for (int i = 0; i < v.Length; i++)
            {
                r[i] = v[i] / n;
            }

            return r;
        }
    }
}