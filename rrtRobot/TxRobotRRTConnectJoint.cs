using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;
namespace rrtRobot
{
    public struct joint
    {
        public double j1 { get; set; }
        public double j2 { get; set; }
        public double j3 { get; set; }
        public double j4 { get; set; }
        public double j5 { get; set; }
        public double j6 { get; set; }


        public double Sever_Gun { get; set; }
        public joint(double J1, double J2, double J3, double J4, double J5, double J6, double Gun_Opening)
        {
            j1 = J1;
            j2 = J2;
            j3 = J3;

            j4 = J4;
            j5 = J5;
            j6 = J6;

            Sever_Gun = Gun_Opening;

        }
        public double[] ToArray()
        {
            return new[] { j1, j2, j3, j4, j5, j6 };
        }

    };

    public struct point
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double rx { get; set; }
        public double ry { get; set; }
        public double rz { get; set; }

        public double Gun_Open { get; set; }


        public int fromStartorEnd { get; set; } // 定义这个点是从起始点延申出来的，还是终止点延申出来的，起始点为0，终止点为1

        public point(double X, double Y, double Z, double RX, double RY, double RZ, double Gun_Opening)
        {
            x = X;
            y = Y;
            z = Z;

            rx = RX;
            ry = RY;
            rz = RZ;

            Gun_Open = Gun_Opening;
            fromStartorEnd = 0;
        }
        public double[] ToArray()
        {
            return new[] { x, y, z, rx, ry, rz };
        }

    };
    public class Node3D_joint
    {
        public joint loc;
        public double cost;
        public Node3D_joint parent;
        public double step_size;


    }
    public class Node3D
    {
        public double x;
        public double y;
        public double z;
        public double rx;
        public double ry;
        public double rz;
        public double cost;
        public Node3D parent;

        public Node3D(double X, double Y, double Z, double RX, double RY, double RZ)
        {
            this.x = X;
            this.y = Y;
            this.z = Z;
            this.rx = RX;
            this.ry = RY;
            this.rz = RZ;

        }

    }

    // === 分离式自适应步长系统 ===
    public class SeparatedAdaptiveSystem
    {
        // 起始树相关参数
        private double currentStartStepSize;

        private double startEnvironmentComplexityFactor = 1.0;
        private List<double> startRecentSuccessRates = new List<double>();
        public List<double> startRecentStepSizes = new List<double>();
        public int startSuccessfulSamples = 0;
        public int startTotalSamples = 0;

        // 终止树相关参数
        private double currentEndStepSize;

        private double endEnvironmentComplexityFactor = 1.0;
        private List<double> endRecentSuccessRates = new List<double>();
        public List<double> endRecentStepSizes = new List<double>();
        public int endSuccessfulSamples = 0;
        public int endTotalSamples = 0;

        // 环境状态
        public string startEnvironmentState = "Unknown";
        public string endEnvironmentState = "Unknown";
        public bool startIsStagnant = false;
        public bool endIsStagnant = false;


        public SeparatedAdaptiveSystem()
        {

        }

        public void InitializeStepSizes(double startStepSize, double endStepSize)
        {
            currentStartStepSize = startStepSize;
            currentEndStepSize = endStepSize;
        }

        public double GetStartStepSize() => currentStartStepSize;
        public double GetEndStepSize() => currentEndStepSize;

        // 记录起始树采样结果
        public void RecordStartTreeSample(bool success)
        {
            startTotalSamples++;
            if (success) startSuccessfulSamples++;
        }

        // 记录终止树采样结果
        public void RecordEndTreeSample(bool success)
        {
            endTotalSamples++;
            if (success) endSuccessfulSamples++;
        }

        // 分离式自适应调整
        public void SeparatedAdaptiveAdjustment(double minStepSize, double maxStepSize,
                                              joint startPoint, joint endPoint,
                                              List<Node3D_joint> startNodes, List<Node3D_joint> endNodes)
        {
            // 起始树自适应调整
            if (startTotalSamples >= 50)
            {
                AdjustStartTreeStepSize(minStepSize, maxStepSize, startPoint, startNodes);
            }

            // 终止树自适应调整  
            if (endTotalSamples >= 50)
            {
                AdjustEndTreeStepSize(minStepSize, maxStepSize, endPoint, endNodes);
            }

            if ((startEnvironmentState == "Extremely Dense Obstacles" || startEnvironmentState == "Dense Obstacles") && startNodes.Count >= 40) startIsStagnant = true;
            if ((endEnvironmentState == "Extremely Dense Obstacles" || endEnvironmentState == "Dense Obstacles") && endNodes.Count >= 40) endIsStagnant = true;

            // 记录历史
            RecordAdaptiveHistory();
        }

        private void AdjustStartTreeStepSize(double minStepSize, double maxStepSize,
                                           joint startPoint, List<Node3D_joint> startNodes)
        {
            double startSuccessRate = startTotalSamples > 0 ?
                (double)startSuccessfulSamples / startTotalSamples : 0;

            // 基于起始点周围环境复杂度调整
            startEnvironmentComplexityFactor = DetermineEnvironmentComplexity(startSuccessRate, true);
            startEnvironmentState = GetEnvironmentStateDescription(startSuccessRate);

            // 计算起始树的局部环境特征
            double localComplexityFactor = CalculateLocalComplexity(startNodes, startPoint, true);

            // 综合调整因子
            double combinedFactor = (startEnvironmentComplexityFactor + localComplexityFactor) / 2.0;

            // 自适应步长计算
            double adaptiveStartStepSize = currentStartStepSize * combinedFactor;

            // 渐进式调整，避免突变
            double adjustmentRate = 0.5;
            currentStartStepSize = currentStartStepSize * (1 - adjustmentRate) +
                                  adaptiveStartStepSize * adjustmentRate;

            // 边界检查
            currentStartStepSize = Math.Max(minStepSize, Math.Min(maxStepSize, currentStartStepSize));
        }

        private void AdjustEndTreeStepSize(double minStepSize, double maxStepSize,
                                         joint endPoint, List<Node3D_joint> endNodes)
        {
            double endSuccessRate = endTotalSamples > 0 ?
                (double)endSuccessfulSamples / endTotalSamples : 0;

            // 基于终止点周围环境复杂度调整
            endEnvironmentComplexityFactor = DetermineEnvironmentComplexity(endSuccessRate, false);
            endEnvironmentState = GetEnvironmentStateDescription(endSuccessRate);

            // 计算终止树的局部环境特征
            double localComplexityFactor = CalculateLocalComplexity(endNodes, endPoint, false);

            // 综合调整因子
            double combinedFactor = (endEnvironmentComplexityFactor + localComplexityFactor) / 2.0;

            // 自适应步长计算
            double adaptiveEndStepSize = currentEndStepSize * combinedFactor;

            // 渐进式调整，避免突变
            double adjustmentRate = 0.5;
            currentEndStepSize = currentEndStepSize * (1 - adjustmentRate) +
                                adaptiveEndStepSize * adjustmentRate;

            // 边界检查
            currentEndStepSize = Math.Max(minStepSize, Math.Min(maxStepSize, currentEndStepSize));
        }

        // 确定环境复杂度因子
        private double DetermineEnvironmentComplexity(double successRate, bool isStartTree)
        {
            if (successRate < 0.20) return 0.3;      // 极困难环境 - 极小步长
            if (successRate < 0.35) return 0.5;      // 困难环境 - 小步长  
            if (successRate < 0.55) return 1.0;      // 中等复杂度 - 正常步长
            if (successRate < 0.75) return 1.5;      // 相对简单 - 大步长
            return 2.0;                               // 非常简单 - 极大步长
        }

        // 计算局部环境复杂度
        private double CalculateLocalComplexity(List<Node3D_joint> nodes, joint targetPoint, bool isStartTree)
        {
            if (nodes.Count < 5) return 1.0;

            // 分析最近添加的节点的密度和分布
            int recentNodeCount = Math.Min(10, nodes.Count);
            List<Node3D_joint> recentNodes = nodes.GetRange(nodes.Count - recentNodeCount, recentNodeCount);

            // 计算节点间的平均距离
            double totalDistance = 0;
            int pairCount = 0;
            for (int i = 0; i < recentNodes.Count - 1; i++)
            {
                for (int j = i + 1; j < recentNodes.Count; j++)
                {
                    totalDistance += CalculateDistance(recentNodes[i].loc, recentNodes[j].loc);
                    pairCount++;
                }
            }

            if (pairCount == 0) return 1.0;

            double avgDistance = totalDistance / pairCount;

            // 计算到目标点的平均距离
            double avgDistanceToTarget = 0;
            foreach (var node in recentNodes)
            {
                avgDistanceToTarget += CalculateDistance(node.loc, targetPoint);
            }
            avgDistanceToTarget /= recentNodes.Count;

            // 根据密度和到目标距离调整复杂度因子
            double densityFactor = 1.0;
            if (avgDistance < Math.PI / 36) densityFactor = 0.3;       // 极高密度，需要小步长
            else if (avgDistance < Math.PI / 18) densityFactor = 0.6;  // 高密度，正常步长
            else if (avgDistance < Math.PI / 9) densityFactor = 1.0;   // 中密度，正常步长
            else densityFactor = 1.4;                                   // 低密度，可以大步长

            double targetDistanceFactor = 1.0;
            if (avgDistanceToTarget > Math.PI) targetDistanceFactor = 1.2;          // 距离目标远，可以大步长
            else if (avgDistanceToTarget < Math.PI / 4) targetDistanceFactor = 0.8; // 距离目标近，需要小步长

            return (densityFactor + targetDistanceFactor) / 2.0;
        }

        private string GetEnvironmentStateDescription(double successRate)
        {
            if (successRate < 0.20) return "Extremely Dense Obstacles";
            if (successRate < 0.35) return "Dense Obstacles";
            if (successRate < 0.55) return "Moderate Complexity";
            if (successRate < 0.75) return "Relatively Simple";
            return "Very Open Space";
        }

        private void RecordAdaptiveHistory()
        {
            if (startTotalSamples > 0)
            {
                double startSuccessRate = (double)startSuccessfulSamples / startTotalSamples;
                startRecentSuccessRates.Add(startSuccessRate);
                startRecentStepSizes.Add(currentStartStepSize);

                if (startRecentSuccessRates.Count > 100)
                {
                    startRecentSuccessRates.RemoveAt(0);
                    startRecentStepSizes.RemoveAt(0);
                }
            }

            if (endTotalSamples > 0)
            {
                double endSuccessRate = (double)endSuccessfulSamples / endTotalSamples;
                endRecentSuccessRates.Add(endSuccessRate);
                endRecentStepSizes.Add(currentEndStepSize);

                if (endRecentSuccessRates.Count > 100)
                {
                    endRecentSuccessRates.RemoveAt(0);
                    endRecentStepSizes.RemoveAt(0);
                }
            }
        }

        private static double CalculateDistance(joint j1, joint j2)
        {
            double sum = 0;
            for (int i = 0; i < 6; i++)
            {
                sum += (j1.ToArray()[i] - j2.ToArray()[i]) * (j1.ToArray()[i] - j2.ToArray()[i]);
            }
            return Math.Sqrt(sum);
        }

        // 重置统计信息（用于长时间运行时的重置）
        public void ResetStatistics()
        {
            startSuccessfulSamples = 0;
            startTotalSamples = 0;
            endSuccessfulSamples = 0;
            endTotalSamples = 0;
            startIsStagnant = false;
            endIsStagnant = false;
        }
    }

    /// <summary>
    /// 边界节点详细信息类
    /// </summary>
    public class BoundaryNodeInfo
    {
        public int Index { get; set; }
        public joint Position { get; set; }              // 6D关节空间位置
        public point Position3D { get; set; }            // 3D笛卡尔空间位置
        public double DistanceToCenter { get; set; }     // 到中心点的3D距离
        public double[] Direction { get; set; }          // 方向向量（单位向量）
        public int ClusterIndex { get; set; }            // 所属的聚类索引
    }

    /// <summary>
    /// 节点角度聚类信息
    /// </summary>
    public class NodeAngleCluster
    {
        public int ClusterIndex { get; set; }            // 聚类索引
        public double[] RepresentativeDirection { get; set; }  // 代表方向（单位向量）
        public List<NodeInfo3D> Nodes { get; set; }      // 该聚类中的所有节点
        public NodeInfo3D FarthestNode { get; set; }     // 该聚类中距离最远的节点（边界）
    }

    /// <summary>
    /// 节点3D信息类
    /// </summary>
    public class NodeInfo3D
    {
        public int Index { get; set; }
        public joint Position { get; set; }
        public point Position3D { get; set; }
        public double[] Direction { get; set; }          // 从中心指向该节点的单位向量
        public double DistanceToCenter { get; set; }     // 3D欧氏距离
    }


    /* 
     * The TxRobotRRTConnectJoint class is used to calculate the transition point trajectory between solder joints. The algorithm is based on bidirectional RRT tree expansion using the start and end point robot's six axis values.
     * The reference GitHub for the RRT* Connect algorithm is https://github.com/WYoseanLove/RRT-_Connect_3D.
     * During the RRT tree expansion, the robot's six axis values are randomly generated and checked for collisions. If the collision detection is passed, the point is considered valid until a valid path is found.
     * Collision detection first performs single-point detection, and then PTP (Point-to-Point) interpolation. The interpolation function isValidforstepCorss(Control control, joint step, joint near) interpolates based on time. It divides the PTP time based on the angle change; if the angle change is less than 100, it divides and interpolates according to 100.
     * The TxRobotptpPathCal class records the PTP interpolation algorithm for a six-axis robot.
     * The RRT Connect algorithm is enhanced with the calculation of APF (Artificial Potential Field) attractive and repulsive forces to influence the generation of random tree points. The attractive force attracts random points towards the end point and the nearest end node tree point. The repulsive force field records all previous interference points from collision detection, and the repulsive force field is calculated based on the distance between new tree points and these interference points."
     * 
     * TxRobotRRTConnectJoint 类用于计算焊点之间的过渡点轨迹，算法基于起始点和终止点机器人的六个轴值进行rrt树的双向扩展。
     * rrt* connect 算法的参考Github https://github.com/WYoseanLove/RRT-_Connect_3D
     * 在rrt树的扩展中，随机生成机器人的6个轴值，并对其进行碰撞检测，如果通过碰撞检测，则视为有效点，直至扩展出有效的路径；
     * 碰撞检测首先进行单点检测，然后进行ptp的插补运算，插补运算函数isValidforstepCorss(Control control, joint step, joint near)，是基于时间进行插补的，将ptp的时间按照角度变化量进行插补，如果角度变化量小于100，则按照100进行时间分割并插补；
     * TxRobotptpPathCal类记录了六轴机器人ptp的插补运算算法；
     * 在rrt connect 算法上增加了apf引力场和排斥力场的计算，用于干涉随机树点的生成，引力场将随机点朝终止点和最近的endnode树点进行吸引，排斥力场记录了之前碰撞检测的所有干涉点，新的树点与干涉点之间的距离进行排斥力场的计算；
     * 
     */

    public partial class TxRobotRRTConnectJoint : TxrrtRobotPathPlannerForm
    {
        private int connected = 0;
        private int state = 1;
        private int sub_state = 0;
        private const double Max_step_szie = Math.PI / 30;
        private const double Min_step_szie = Math.PI / 180;
        public static double start_step_size = M_PI / 18;
        public static double end_step_size = M_PI / 18;
        private double k_att = 1.0;
        private double k_rep = 5.0;

        private double circle_radius_1 = 20;

        public List<joint> path_points_start = new List<joint>(500);
        private int pathcount_start = 0;
        private List<joint> path_points_end = new List<joint>(500);
        private int pathcount_end = 0;
        public static Random rd;
        private List<Node3D_joint> start_nodes = new List<Node3D_joint>(10000);
        private int nodecount_start = 0;
        private List<Node3D_joint> end_nodes = new List<Node3D_joint>(10000);
        private int nodecount_end = 0;
        public static List<joint> obs;
        private TxComponent openSideCandidateVisual;//开放侧的可视化；

        private int IterationCounts = 0;//记录rrtconnect 的迭代次数；

        public static bool currentpathdone = false;

        public double j1Llimit, j2Llimit, j3Llimit, j4Llimit, j5Llimit, j6Llimit;
        public double j1Ulimit, j2Ulimit, j3Ulimit, j4Ulimit, j5Ulimit, j6Ulimit;
        // === 智能采样和分离式自适应系统相关成员变量 ===
        public static SeparatedAdaptiveSystem separatedAdaptiveSystem;
        // === 机器学习/经验学习出口吸引点系统 ===
        private EscapeOutletAttractorLearner outletLearner = new EscapeOutletAttractorLearner();
        private JointLimitBox outletJointLimits;
        // === 出口吸引点连接线可视化 ===
        private TxComponent outletConnectionVisualStart;
        private TxComponent outletConnectionVisualEnd;

        public static void logpathGenerateOK(string str)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);
            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() + str);
            sw.Close();

        }
        public static void LograndNodeInformation(point p, string information)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);

            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() + " " + information + ": " + p.x.ToString() + " "
           + p.y.ToString() + " "
           + p.z.ToString() + " "
           + (p.rx * 180 / M_PI).ToString() + " "
           + (p.ry * 180 / M_PI).ToString() + " "
           + (p.rz * 180 / M_PI).ToString() + " "
           + (p.Gun_Open).ToString() + " ");

            sw.Close();
        }
        public double dist(joint p1, joint p2)  // To calculate the distance between two points
        {

            return Math.Sqrt(Math.Pow(p2.j1 - p1.j1, 2) + Math.Pow(p2.j2 - p1.j2, 2) + Math.Pow(p2.j3 - p1.j3, 2)
                + Math.Pow(p2.j4 - p1.j4, 2) + Math.Pow(p2.j5 - p1.j5, 2) + Math.Pow(p2.j6 - p1.j6, 2));

        }
        public int Nearest_Node(int fromstart2end, Node3D_joint rand)
        {
            double min = 999.0;
            int index = -1;

            if (fromstart2end == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {

                    if (dist(rand.loc, start_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, start_nodes[i].loc);
                        index = i;
                    }

                }

            }
            else
            {
                for (int i = 0; i < nodecount_end; i++)
                {

                    if (dist(rand.loc, end_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, end_nodes[i].loc);
                        index = i;
                    }

                }



            }




            return index;
        }

        public joint step_func(joint near, joint rand, double size_step)
        {
            double j1 = rand.j1 - near.j1;
            double j2 = rand.j2 - near.j2;
            double j3 = rand.j3 - near.j3;
            double j4 = rand.j4 - near.j4;
            double j5 = rand.j5 - near.j5;
            double j6 = rand.j6 - near.j6;



            double d = Math.Sqrt(Math.Pow(j1, 2) + Math.Pow(j2, 2) + Math.Pow(j3, 2) +
                Math.Pow(j4, 2) + Math.Pow(j5, 2) + Math.Pow(j6, 2));

            joint step = new joint(near.j1 + (size_step) * (j1 / d),
                near.j2 + (size_step) * (j2 / d),
                near.j3 + (size_step) * (j3 / d),
                near.j4 + (size_step) * (j4 / d),
                near.j5 + (size_step) * (j5 / d),
                near.j6 + (size_step) * (j6 / d),
                rand.Sever_Gun
                 );

            return step;
        }
        /// 顶层接口：
        /// 1) 先单点剔除碰撞并微调 step；
        /// 2) 再做整段插值碰撞检测；
        ///    只要 collisioncheck 过程中微调了 step，就重跑一遍，直到“整条跑完都没改动 step” 或者碰撞失败。
        /// </summary>
        public bool isValid(Control control, joint step, joint near, bool stepToNear)
        {
            // 单点检测并尝试调整 step
            if (!collisioncheckforSingleJoint(control, ref step))
                return false;

            // 跨点路径检测，只允许微调 step
            if (isRCSLoaded)
            {
                if (!isValidforstepCorssRCS(control, ref step, near, stepToNear))
                    return false;

            }
            else
            {
                if (!isValidforstepCorss(control, ref step, near, stepToNear))
                    return false;
            }

            return true;
        }

        /// <summary>
        /// 原样保留：对单独一个姿态做碰撞检测，
        /// 如果检测通过返回 true；否则尝试微调 step.Sever_Gun，
        /// 可调通则 true，否则 false。
        /// </summary>
        public static bool collisioncheckforSingleJoint(Control control, ref joint step)
        {
            using (var robotPosture = new TxPoseData())
            {
                step.ToArray();
                var arr = new ArrayList(step.ToArray());
                robotPosture.JointValues = arr;
                var sols = new ArrayList { robotPosture };
                //修改了这里，先生成posture,将机器人的姿态更新，再检测是否超值，
                //适合于FANUC机器人2/3轴联动的
                TxRobotAPIClass.TxRobotPostureGenerate(
                    control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, step.Sever_Gun);
                for (int i = 0; i < 6; i++)
                {
                    if ((step.ToArray()[i] < robot.Joints[i].LowerSoftLimit) || (step.ToArray()[i] > robot.Joints[i].UpperSoftLimit)) return false;

                }
                if (TxRobotAPIClass.Collision_Check(
                    control, cd, queryParams, root, collisionSrc, collisionTar, 5.0))
                    return true;

                const int N = 30;
                for (int i = N - 1; i >= 1; i--)
                {
                    double g = step.Sever_Gun * i / (double)N;
                    TxRobotAPIClass.TxRobotPostureGenerate(
                        control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, g);
                    if (TxRobotAPIClass.Collision_Check(
                        control, cd, queryParams, root, collisionSrc, collisionTar, 5.0))
                    {
                        step.Sever_Gun = g;
                        return true;
                    }
                }

                obs.Add(step);
                return false;
            }
        }


        /// <summary>
        /// 整段路径检测：
        ///   stepToNear=false → 从 step→near，step 是 “起点”，
        ///   stepToNear=true  → 从 near→step，step 是 “终点”。
        /// 只要 collisioncheckforSingleJoint 微调了 step，就把新值写回 step 并重跑一遍，
        /// 直到“一整条跑下来都没再改动 step” 才算 true，
        /// 或者某次单点碰撞失败直接 false。
        /// </summary>
        public static bool isValidforstepCorss(
            Control control,
            ref joint step,
            joint near,
            bool stepToNear
        )
        {
            const int MAX_ITERS = 10;
            int iter = 0;
            while (iter++ < MAX_ITERS)
            {
                //joint before = step;    // 备份本轮进入时的 step

                // 决定插值的 start/end
                joint start = stepToNear ? near : step;
                joint end = stepToNear ? step : near;

                // 计算 PTP 时间和分段数
                var deltas = TxRobotptpPathCal.calculateJointsChange(start, end, robot);
                double dg = TxRobotptpPathCal.calculateServoGunJointChange(start, end, robot);
                double t1 = TxRobotptpPathCal.calculatePTPtime(control, deltas, robot);
                double t2 = TxRobotptpPathCal.calculateServoPTPtime(dg, robServerGun);
                double T = Math.Max(t1, t2);
                int N = (int)(T / 0.01);

                bool bumped = false;
                //InterpolJoints.Add(start);
                //InterpolJoints.Add(end);
                for (int i = 1; i <= N; i++)
                {
                    double t = T * i / N;
                    using (var pose = TxRobotptpPathCal.calCurrentRobotPosedata(
                        control, start, end, robot, t, T))
                    {
                        double gun = TxRobotptpPathCal.calCurrentServoGunJointData(
                            T, t, robServerGun, dg, start.Sever_Gun);

                        joint p = new joint(
                            (double)pose.JointValues[0],
                            (double)pose.JointValues[1],
                            (double)pose.JointValues[2],
                            (double)pose.JointValues[3],
                            (double)pose.JointValues[4],
                            (double)pose.JointValues[5],
                            gun
                        );
                        //InterpolJoints.Add(p);
                        joint before = p;    // 备份本轮进入时的 step
                        // 单点碰撞
                        if (!collisioncheckforSingleJoint(control, ref p))
                        {
                            //InterpolJoints.Clear();
                            return false;
                        }


                        // 如果碰撞检测微调了 p，就把它当做新的 step 然后跳出重跑
                        if (!JointEquals(p, before))
                        {
                            step.Sever_Gun = p.Sever_Gun;
                            bumped = true;
                            break;
                        }
                    }
                }

                if (!bumped)
                {

                    // 打印出插补点；
                    //LoginterPoljoints(InterpolJoints);
                    //InterpolJoints.Clear();
                    return true;    // 本轮没有被微调，说明完整路径都 OK

                }
            }

            // 超过最大迭代次数仍未“收敛”，认为失败
            //InterpolJoints.Clear();
            return false;
        }
        // 只用于最终的轨迹优化
        public static bool isValidforstepCorss(
           Control control,
           joint start,
           joint end
          )
        {

            // 计算 PTP 时间和分段数
            var deltas = TxRobotptpPathCal.calculateJointsChange(start, end, robot);
            double dg = TxRobotptpPathCal.calculateServoGunJointChange(start, end, robot);
            double t1 = TxRobotptpPathCal.calculatePTPtime(control, deltas, robot);
            double t2 = TxRobotptpPathCal.calculateServoPTPtime(dg, robServerGun);
            double T = Math.Max(t1, t2);
            int N = (int)(T / 0.01);


            for (int i = 1; i <= N; i++)
            {
                double t = T * i / N;
                using (var pose = TxRobotptpPathCal.calCurrentRobotPosedata(
                    control, start, end, robot, t, T))
                {
                    double gun = TxRobotptpPathCal.calCurrentServoGunJointData(
                        T, t, robServerGun, dg, start.Sever_Gun);

                    joint p = new joint(
                        (double)pose.JointValues[0],
                        (double)pose.JointValues[1],
                        (double)pose.JointValues[2],
                        (double)pose.JointValues[3],
                        (double)pose.JointValues[4],
                        (double)pose.JointValues[5],
                        gun
                    );
                    joint before = p;


                    if (!collisioncheckforSingleJoint(control, ref p))
                        return false;

                    if (Math.Abs(p.Sever_Gun - before.Sever_Gun) > 1e-4)
                        return false;
                }
            }


            return true;

        }

        static bool JointEquals(joint a, joint b)
        {
            const double EPS = 1e-2;
            return
              Math.Abs(a.j1 - b.j1) < EPS && Math.Abs(a.j2 - b.j2) < EPS &&
              Math.Abs(a.j3 - b.j3) < EPS && Math.Abs(a.j4 - b.j4) < EPS &&
              Math.Abs(a.j5 - b.j5) < EPS && Math.Abs(a.j6 - b.j6) < EPS &&
              Math.Abs(a.Sever_Gun - b.Sever_Gun) < EPS;
        }

        /// <summary>
        /// 最小代价优化（性能优化版 - 只检查最近的节点）
        /// </summary>
        /// <param name="control">控制器</param>
        /// <param name="step">新节点</param>
        /// <param name="threadholdIter">迭代阈值</param>
        public void minimal_cost(Control control, Node3D_joint step, int threadholdIter)
        {
            double new_cost;
            double min_cost = step.cost;
            int index = -1;

            if (state == 1)
            {
                circle_radius_1 = 2 * start_step_size;

                // ================================================================
                // 性能优化：只遍历最后10个节点，如果不足10个则全部遍历
                // ================================================================
                int checkCount = Math.Min(20, nodecount_start);
                int startIdx = Math.Max(0, nodecount_start - checkCount);


                for (int i = startIdx; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_1 &&
                        isValid(control, step.loc, start_nodes[i].loc, true)
                       )
                    {
                        new_cost = dist(start_nodes[i].loc, step.loc) + start_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }
                }

                if (min_cost < step.cost)
                {
                    step.parent = start_nodes[index];
                    step.cost = min_cost;
                }
            }
            else
            {
                circle_radius_1 = 2 * end_step_size;

                // ================================================================
                // 性能优化：只遍历最后10个节点，如果不足10个则全部遍历
                // ================================================================
                int checkCount = Math.Min(20, nodecount_end);
                int startIdx = Math.Max(0, nodecount_end - checkCount);

                for (int i = startIdx; i < nodecount_end; i++)
                {
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_1 &&
                        isValid(control, step.loc, end_nodes[i].loc, false))
                    {
                        new_cost = dist(end_nodes[i].loc, step.loc) + end_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }
                }

                if (min_cost < step.cost)
                {
                    step.parent = end_nodes[index];
                    step.cost = min_cost;
                }
            }
        }
        private void AppendPathEndNodesToStartTree(Control control)
        {
            if (TxrrtRobotPathPlannerForm.Pathend_nodes.Count <= 1)
            {
                TxrrtRobotPathPlannerForm.Pathend_nodes.Clear();
                return;
            }

            TxrrtRobotPathPlannerForm.Pathend_nodes.Reverse();

            for (int i = 1; i < TxrrtRobotPathPlannerForm.Pathend_nodes.Count; i++)
            {
                Node3D_joint newJointLoc = new Node3D_joint();
                Node3D_joint parentNode = (i == 1) ? start_nodes[0] : start_nodes.Last();

                newJointLoc.loc = TxrrtRobotPathPlannerForm.Pathend_nodes[i];
                newJointLoc.parent = parentNode;
                newJointLoc.step_size = dist(newJointLoc.loc, parentNode.loc);
                newJointLoc.cost = newJointLoc.step_size + parentNode.cost;

                minimal_cost(control, newJointLoc, IterationCounts);

                start_nodes.Add(newJointLoc);
                nodecount_start++;
            }

            TxrrtRobotPathPlannerForm.Pathend_nodes.Clear();
        }
        public void path_Points(int index_1, int index_2) // final path points will be keep at path_points_start and path_points_end List
        {
            pathcount_start = 0;
            pathcount_end = 0;
            Node3D_joint n1, n2;
            double d = Min_step_szie;

            n2 = start_nodes[index_1 - 1];
            n1 = n2.parent;

            path_points_start.Add(n2.loc);
            pathcount_start++;

            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_start[pathcount_start - 1]) < d)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;

                    path_points_start.Add(n2.loc);
                    pathcount_start++;
                }
            }
            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;

            path_points_end.Add(n2.loc);
            pathcount_end++;
            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_end[pathcount_end - 1]) < d / 4)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;
                    path_points_end.Add(n2.loc);
                    pathcount_end++;
                }
            }



        }
        public double GetRandomDouble(double minValue, double maxValue, double lowerLimit, double UpperLimit)
        {
            if (minValue < lowerLimit) minValue = lowerLimit;
            if (maxValue > UpperLimit) maxValue = UpperLimit;



            return rd.NextDouble() * (maxValue - minValue) + minValue;
        }

        private double[] getAttractiveforceField(joint q, joint goal, double k_att)
        {
            double[] gradient = new double[6];

            // 计算吸引势场
            for (int i = 0; i < 6; i++) // 对位置和旋转分量进行吸引势场计算
            {
                gradient[i] = k_att * (goal.ToArray()[i] - q.ToArray()[i]);

            }
            return gradient;

        }
        private double[] ApfCalculateMethod(Control control, joint q, joint nearfromGoalNodes, joint goal, double step_size, List<joint> obsList, double k_att, double k_rep, double d0)
        {

            double[] gradient = new double[6];
            double[] gradientfromGoalNodes = new double[6];
            double[] RepulsiveForce = new double[6] { 0, 0, 0, 0, 0, 0 };
            //计算吸引势场
            gradient = getAttractiveforceField(q, goal, k_att);
            //计算与对手nodelist里面最近点的吸引势场
            gradientfromGoalNodes = getAttractiveforceField(q, nearfromGoalNodes, 3 * k_att);
            //计算与obs的排斥力场
            foreach (var obs in obsList)
            {

                double distance = dist(q, obs);
                if (distance > d0) continue;
                if (distance == 0)
                {
                    for (int i = 0; i < RepulsiveForce.Length; i++)
                    {
                        RepulsiveForce[i] = 1000000; // 或者任何其他计算方式
                    }
                    continue;
                }

                for (int i = 0; i < 6; i++) // 对位置和旋转分量进行吸引势场计算
                {
                    RepulsiveForce[i] += k_rep * (1 / distance - 1 / d0) * (1 / (distance * distance)) * (q.ToArray()[i] - obs.ToArray()[i]) / distance;

                }


            }

            for (int i = 0; i < RepulsiveForce.Count(); i++)
            {

                gradient[i] += RepulsiveForce[i] + gradientfromGoalNodes[i];

            }


            return Normalize(gradient);

        }

        private static double[] Normalize(double[] vector)
        {
            double norm = Math.Sqrt(vector.Sum(x => x * x));
            return vector.Select(x => x / norm).ToArray();
        }


        public void rrt_connectJointPtp(Control control, joint p_start, joint p_end)
        {
            // 初始化智能采样和分离式自适应系统

            separatedAdaptiveSystem = new SeparatedAdaptiveSystem();
            // 初始化出口吸引点学习器
            if (outletLearner == null)
            {
                outletLearner = new EscapeOutletAttractorLearner();
            }

            // 初始化关节限位
            outletJointLimits = new JointLimitBox(
                j1Llimit, j2Llimit, j3Llimit, j4Llimit, j5Llimit, j6Llimit,
                j1Ulimit, j2Ulimit, j3Ulimit, j4Ulimit, j5Ulimit, j6Ulimit);

            connected = 0;
            state = 1;
            sub_state = 0;
            nodecount_start = 0;
            nodecount_end = 0;
            IterationCounts = 0;

            Node3D_joint start_node = new Node3D_joint();
            Node3D_joint end_node = new Node3D_joint();
            Node3D_joint rand_node = new Node3D_joint();
            int index;


            Node3D_joint step_node;
            Node3D_joint sub_step_node;

            rd = new Random(unchecked((int)DateTime.Now.Ticks));

            start_node.loc = new joint(p_start.j1, p_start.j2, p_start.j3, p_start.j4, p_start.j5, p_start.j6, p_start.Sever_Gun);

            start_node.parent = new Node3D_joint();
            start_node.cost = 0;
            end_node.loc = new joint(p_end.j1, p_end.j2, p_end.j3, p_end.j4, p_end.j5, p_end.j6, p_end.Sever_Gun);

            end_node.parent = new Node3D_joint();
            end_node.cost = 0;

            start_nodes.Add(start_node);

            nodecount_start++;
            end_nodes.Add(end_node);
            nodecount_end++;
            currentpathdone = false;

            int gun_open_splict = 30;

            obs = new List<joint>();

            while (!collisioncheckforSingleJoint(control, ref p_start))
            {

                return;
            }

            start_node.loc.Sever_Gun = p_start.Sever_Gun;


            while (!collisioncheckforSingleJoint(control, ref p_end))
            {

                return;
            }

            end_node.loc.Sever_Gun = p_end.Sever_Gun;
            int threshold = 1000;

            // 初始化分离式自适应步长系统

            double p_end2p_start = dist(p_start, p_end);

            start_step_size = p_end2p_start / 5;
            end_step_size = p_end2p_start / 5;

            start_step_size = Math.Min(Math.PI / 18, start_step_size);
            end_step_size = Math.Min(Math.PI / 18, end_step_size);

            separatedAdaptiveSystem.InitializeStepSizes(start_step_size, end_step_size);
            separatedAdaptiveSystem.startRecentStepSizes.Add(start_step_size);
            separatedAdaptiveSystem.endRecentStepSizes.Add(end_step_size);
            TxComponent x = TxRobotAPIClass.CreateResourcePathCurve(0, "pathCurve");
            TxComponent y = TxRobotAPIClass.CreateResourcePathCurve(1, "pathCurve");

            while (connected != 1)
            {
                Application.DoEvents();
                if (!rrtconnectCal_ongoing)
                {
                    break;
                }
                // 每100次迭代进行分离式自适应调整
                if (IterationCounts % 100 == 0 && IterationCounts > 0)
                {
                    if (IterationCounts % 1000 == 0)
                    {
                        start_step_size = separatedAdaptiveSystem.startRecentStepSizes.Average();
                        end_step_size = separatedAdaptiveSystem.endRecentStepSizes.Average();

                        separatedAdaptiveSystem.InitializeStepSizes(start_step_size, end_step_size);
                        separatedAdaptiveSystem.ResetStatistics();


                    }
                    separatedAdaptiveSystem.SeparatedAdaptiveAdjustment(
                        Min_step_szie, Max_step_szie, p_start, p_end, start_nodes, end_nodes);

                    // 获取独立调整后的步长
                    start_step_size = separatedAdaptiveSystem.GetStartStepSize();
                    end_step_size = separatedAdaptiveSystem.GetEndStepSize();

                }
                sub_state = 0;
                IterationCounts++;
                if (IterationCounts >= 10000 && TxrrtRobotPathPlannerForm.Pathend_nodes.Count > 1)
                {
                    AppendPathEndNodesToStartTree(control);
                }
                if ((IterationCounts / threshold) == 12)
                {
                    if (x != null)
                        x.Delete();
                    if (y != null)
                        y.Delete();
                    if (openSideCandidateVisual != null)
                    {
                        openSideCandidateVisual.Delete();
                        openSideCandidateVisual = null;
                    }
                    if (outletConnectionVisualStart != null)
                    {
                        outletConnectionVisualStart.Delete();
                        outletConnectionVisualStart = null;
                    }
                    if (outletConnectionVisualEnd != null)
                    {
                        outletConnectionVisualEnd.Delete();
                        outletConnectionVisualEnd = null;
                    }
                    return; //如果迭代次数超过10000次则退出
                }
                if (state == 1)
                {

                    double rand_node_gun_open = 0;

                    //rand_node_gun_open = ToolJointOpening - rd.Next(0, gun_open_splict) * (ToolJointOpening / gun_open_splict);
                    rand_node_gun_open = ToolJointOpening;
                    rand_node.loc = new joint(GetRandomDouble(start_nodes[0].loc.j1 - M_PI / 2, start_nodes[0].loc.j1 + M_PI / 2, j1Llimit, j1Ulimit),
                         GetRandomDouble(start_nodes[0].loc.j2 - M_PI / 2, start_nodes[0].loc.j2 + M_PI / 2, j2Llimit, j2Ulimit),
                         GetRandomDouble(start_nodes[0].loc.j3 - M_PI / 2, start_nodes[0].loc.j3 + M_PI / 2, j3Llimit, j3Ulimit),
                         GetRandomDouble(start_nodes[0].loc.j4 - M_PI, start_nodes[0].loc.j4 + M_PI, j4Llimit, j4Ulimit),
                         GetRandomDouble(start_nodes[0].loc.j5 - M_PI / 2, start_nodes[0].loc.j5 + M_PI / 2, j5Llimit, j5Ulimit),
                         GetRandomDouble(start_nodes[0].loc.j6 - M_PI, start_nodes[0].loc.j6 + M_PI, j6Llimit, j6Ulimit),
                         rand_node_gun_open);

                    bool useSpecialSelection = (IterationCounts % 2 == 0);

                    // ============================================================
                    // 起点树停滞：用出口吸引点替代 rand_node.loc
                    // 这个吸引点像随机点一样，用于决定哪个 start_nodes 节点被选中扩展
                    // ============================================================
                    if (separatedAdaptiveSystem.startIsStagnant && useSpecialSelection)
                    {
                        int otherTreeIndex = Nearest_Node(2, start_nodes[0]);
                        if (otherTreeIndex < 0) otherTreeIndex = 0;
                        OutletAttractorResult outletResult =
                            outletLearner.PredictOutletAttractor(
                                start_nodes,              // 当前正在扩展的树
                                p_end,                    // 当前树的目标
                                end_nodes[otherTreeIndex].loc,       // 对侧树参考点
                                obs,                      // 障碍/碰撞点
                                start_step_size,          // 当前步长
                                true,                     // 当前是起点树
                                outletJointLimits);       // 关节限位

                        if (outletResult != null && outletResult.Success)
                        {
                            rand_node.loc = outletResult.RandLikeAttractor;

                        }

                        // 关键：仍然用 Nearest_Node，让吸引点决定哪个成功节点扩展
                        index = Nearest_Node(1, rand_node);

                    }
                    else
                    {
                        index = Nearest_Node(1, rand_node);
                    }

                    int index_fromEndNodes = Nearest_Node(2, rand_node);

                    if (index_fromEndNodes < 0)
                    {
                        index_fromEndNodes = 0;
                    }

                    if (index < 0)
                    {
                        continue;
                    }

                    if (index_fromEndNodes < 0) index_fromEndNodes = 0;

                    if (index < 0)
                    {
                        continue;
                    }

                    if (dist(start_nodes[index].loc, rand_node.loc) < start_step_size)
                    {

                        continue;
                    }
                    else
                    {
                        step_node = new Node3D_joint();

                        (step_node.loc) = step_func(start_nodes[index].loc, rand_node.loc, start_step_size);

                        //double[] apf_direction = ArtificialPotentialField(control, step_node.loc, start_nodes[index].loc, end_nodes[index_fromEndNodes].loc, p_end, start_step_size);
                        double[] apf_direction = ApfCalculateMethod(control, step_node.loc, end_nodes[index_fromEndNodes].loc, p_end, start_step_size, obs, k_att, k_rep, 2 * start_step_size);

                        double[] q_rand_array = step_node.loc.ToArray();
                        for (int i = 0; i < 6; i++) // 对位置和旋转分量进行调整
                        {
                            q_rand_array[i] += start_step_size * apf_direction[i];
                        }

                        joint q_rand_modified = new joint(q_rand_array[0], q_rand_array[1], q_rand_array[2], q_rand_array[3], q_rand_array[4], q_rand_array[5], step_node.loc.Sever_Gun);

                        step_node.loc = q_rand_modified;


                    }
                    bool isvalidcorss = isValid(control, step_node.loc, start_nodes[index].loc, true);
                    if (isvalidcorss == false)
                    {
                        // 如果扩展失败，尝试使用局部路径规划
                        if (LocalPathPlanningWithAPF(control, ref step_node.loc, p_start, p_end, start_nodes[index].loc, end_nodes[index_fromEndNodes].loc, true, obs, k_att, k_rep, start_step_size, 5, start_step_size / 2))
                        {
                            isvalidcorss = true;
                        }

                    }
                    separatedAdaptiveSystem.RecordStartTreeSample(isvalidcorss);
                    if (isvalidcorss == true)
                    {

                        step_node.parent = start_nodes[index];
                        step_node.cost = start_nodes[index].cost + start_step_size;
                        minimal_cost(control, step_node, IterationCounts);
                        step_node.step_size = start_step_size;
                        TxRobotAPIClass.TxcreateCurvePath(control, x, step_node.parent.loc, step_node.loc, start_nodes.Count.ToString(), robot, new TxColor(220, 220, 220));

                        start_nodes.Add(step_node);
                        nodecount_start++;
                    }
                    else continue;
                    state = 2;
                    int end_substate = 0;

                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing)
                        {
                            break;
                        }

                        index = Nearest_Node(state, step_node);
                        if (index < 0) continue;
                        if (isValid(control, step_node.loc, end_nodes[index].loc, false) || (!rrtconnectCal_ongoing))
                        {

                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");

                            path_Points(nodecount_start, index + 1);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_joint();

                            (sub_step_node.loc) = step_func(end_nodes[index].loc, step_node.loc, end_step_size);


                        }
                        if (isValid(control, sub_step_node.loc, end_nodes[index].loc, false) == false)
                        {
                            sub_state = 1;
                            separatedAdaptiveSystem.RecordEndTreeSample(false);
                            continue;
                        }
                        else
                        {
                            sub_step_node.parent = end_nodes[index];
                            sub_step_node.cost = end_nodes[index].cost + end_step_size;
                            minimal_cost(control, sub_step_node, IterationCounts);
                            sub_step_node.step_size = end_step_size;
                            separatedAdaptiveSystem.RecordEndTreeSample(true);
                            TxRobotAPIClass.TxcreateCurvePath(control, y, sub_step_node.parent.loc, sub_step_node.loc, end_nodes.Count.ToString(), robot, new TxColor(255, 0, 0));

                            end_nodes.Add(sub_step_node);
                            nodecount_end++;
                            end_substate++;
                            if (end_substate > 5)
                            {
                                sub_state = 1;
                                end_substate = 0;
                            }
                        }

                    }

                }
                if (state == 2)
                {

                    double rand_node_gun_open = 0;

                    //rand_node_gun_open = ToolJointOpening - rd.Next(0, gun_open_splict) * (ToolJointOpening / gun_open_splict);
                    rand_node_gun_open = ToolJointOpening;
                    rand_node.loc = new joint(GetRandomDouble(end_nodes[0].loc.j1 - M_PI / 2, end_nodes[0].loc.j1 + M_PI / 2, j1Llimit, j1Ulimit),
                         GetRandomDouble(end_nodes[0].loc.j2 - M_PI / 2, end_nodes[0].loc.j2 + M_PI / 2, j2Llimit, j2Ulimit),
                         GetRandomDouble(end_nodes[0].loc.j3 - M_PI / 2, end_nodes[0].loc.j3 + M_PI / 2, j3Llimit, j3Ulimit),
                         GetRandomDouble(end_nodes[0].loc.j4 - M_PI, end_nodes[0].loc.j4 + M_PI, j4Llimit, j4Ulimit),
                         GetRandomDouble(end_nodes[0].loc.j5 - M_PI / 2, end_nodes[0].loc.j5 + M_PI / 2, j5Llimit, j5Ulimit),
                         GetRandomDouble(end_nodes[0].loc.j6 - M_PI, end_nodes[0].loc.j6 + M_PI, j6Llimit, j6Ulimit),
                         rand_node_gun_open);
                    bool useSpecialSelection = (IterationCounts % 2 == 0);

                    // ============================================================
                    // 终点树停滞：用出口吸引点替代 rand_node.loc
                    // 这个吸引点像随机点一样，用于决定哪个 end_nodes 节点被选中扩展
                    // ============================================================
                    if (separatedAdaptiveSystem.endIsStagnant && useSpecialSelection)
                    {
                        //joint otherTreeReference = start_nodes.Count > 0 ? start_nodes[0].loc : p_start;
                        int otherTreeIndex = Nearest_Node(1, end_nodes[0]);
                        if (otherTreeIndex < 0) otherTreeIndex = 0;
                        OutletAttractorResult outletResult =
                            outletLearner.PredictOutletAttractor(
                                end_nodes,                // 当前正在扩展的树
                                p_start,                  // 终点树反向扩展时目标是起点
                                start_nodes[otherTreeIndex].loc,       // 对侧树参考点
                                obs,                      // 障碍/碰撞点
                                end_step_size,            // 当前步长
                                false,                    // 当前不是起点树
                                outletJointLimits);       // 关节限位

                        if (outletResult != null && outletResult.Success)
                        {
                            rand_node.loc = outletResult.RandLikeAttractor;

                        }

                        // 关键：仍然用 Nearest_Node，让吸引点决定哪个成功节点扩展
                        index = Nearest_Node(2, rand_node);
                  
                    }
                    else
                    {
                        index = Nearest_Node(state, rand_node);
                    }

                    int index_fromEndNodes = Nearest_Node(1, rand_node);

                    if (index_fromEndNodes < 0)
                    {
                        index_fromEndNodes = 0;
                    }

                    if (index < 0)
                    {
                        continue;
                    }


                    if (dist(end_nodes[index].loc, rand_node.loc) < end_step_size) continue;
                    else
                    {
                        step_node = new Node3D_joint();



                        (step_node.loc) = step_func(end_nodes[index].loc, rand_node.loc, end_step_size);

                        //double[] apf_direction = ArtificialPotentialField(control, step_node.loc, end_nodes[index].loc, start_nodes[index_fromEndNodes].loc, p_start, end_step_size);
                        double[] apf_direction = ApfCalculateMethod(control, step_node.loc, start_nodes[index_fromEndNodes].loc, p_start, end_step_size, obs, k_att, k_rep, end_step_size * 2);

                        double[] q_rand_array = step_node.loc.ToArray();
                        for (int i = 0; i < 6; i++) // 对位置和旋转分量进行调整
                        {
                            q_rand_array[i] += end_step_size * apf_direction[i];
                        }

                        joint q_rand_modified = new joint(q_rand_array[0], q_rand_array[1], q_rand_array[2], q_rand_array[3], q_rand_array[4], q_rand_array[5], step_node.loc.Sever_Gun);

                        step_node.loc = q_rand_modified;

                    }
                    bool isvalidcorss = isValid(control, step_node.loc, end_nodes[index].loc, false);
                    if (isvalidcorss == false)
                    {
                        if (LocalPathPlanningWithAPF(control, ref step_node.loc, p_start, p_end, start_nodes[index_fromEndNodes].loc, end_nodes[index].loc, false, obs, k_att, k_rep, end_step_size, 5, end_step_size / 2))
                        {
                            isvalidcorss = true;
                        }
                    }
                    separatedAdaptiveSystem.RecordEndTreeSample(isvalidcorss);
                    if (isvalidcorss == true)
                    {

                        step_node.parent = end_nodes[index];
                        step_node.cost = end_nodes[index].cost + end_step_size;
                        minimal_cost(control, step_node, IterationCounts);
                        step_node.step_size = end_step_size;
                        end_nodes.Add(step_node);
                        TxRobotAPIClass.TxcreateCurvePath(control, y, step_node.parent.loc, step_node.loc, end_nodes.Count.ToString(), robot, new TxColor(250, 0, 0));

                        nodecount_end++;

                    }
                    else continue;

                    state = 1;
                    int start_substate = 0;
                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing) break;

                        index = Nearest_Node(state, step_node);
                        if (index < 0) continue;
                        if (isValid(control, step_node.loc, start_nodes[index].loc, true) || (!rrtconnectCal_ongoing))
                        {
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                            path_Points(index + 1, nodecount_end);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_joint();

                            (sub_step_node.loc) = step_func(start_nodes[index].loc, step_node.loc, start_step_size);

                        }

                        if (isValid(control, sub_step_node.loc, start_nodes[index].loc, true) == false)
                        {


                            sub_state = 1;
                            separatedAdaptiveSystem.RecordStartTreeSample(false);
                            continue;
                        }
                        else
                        {
                            start_substate++;
                            sub_step_node.parent = start_nodes[index];
                            sub_step_node.cost = start_nodes[index].cost + start_step_size;
                            minimal_cost(control, sub_step_node, IterationCounts);
                            separatedAdaptiveSystem.RecordStartTreeSample(true);
                            start_nodes.Add(sub_step_node);
                            TxRobotAPIClass.TxcreateCurvePath(control, x, sub_step_node.parent.loc, sub_step_node.loc, start_nodes.Count.ToString(), robot, new TxColor(220, 220, 220));

                            sub_step_node.step_size = start_step_size;
                            nodecount_start++;
                            if (start_substate > 5)
                            {
                                sub_state = 1;
                                start_substate = 0;
                            }

                        }

                    }




                }

            }
            path_points_start.Reverse();

            path_points_start.AddRange(path_points_end);
            start_nodes.Clear();
            TxrrtRobotPathPlannerForm.Pathend_nodes.AddRange(path_points_end);
            end_nodes.Clear();
            logpathGenerateOK(" Path Generate OK! " + IterationCounts.ToString());

            currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
            if (x != null)
                x.Delete();
            if (y != null)
                y.Delete();
            if (openSideCandidateVisual != null)
            {
                openSideCandidateVisual.Delete();
                openSideCandidateVisual = null;
            }
            if (outletConnectionVisualStart != null)
            {
                outletConnectionVisualStart.Delete();
                outletConnectionVisualStart = null;
            }
            if (outletConnectionVisualEnd != null)
            {
                outletConnectionVisualEnd.Delete();
                outletConnectionVisualEnd = null;
            }
        }
        // 新增的方法：局部路径规划
        public bool LocalPathPlanningWithAPF(Control control, ref joint current, joint p_start, joint p_goal, joint NearStartNodes, joint NearGoalNodes, bool fromstart2end, List<joint> obsList, double k_att, double k_rep, double influenceRadius, int maxIterations, double learningRate)
        {

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (fromstart2end) //表示从start向end去扩展
                {
                    // 计算当前位置的势场梯度  
                    double[] gradient = ApfCalculateMethod(control, current, NearGoalNodes, p_goal, learningRate, obsList, k_att, k_rep, influenceRadius);

                    // 沿着梯度方向调整位置
                    current.j1 += learningRate * gradient[0];
                    current.j2 += learningRate * gradient[1];
                    current.j3 += learningRate * gradient[2];
                    current.j4 += learningRate * gradient[3];
                    current.j5 += learningRate * gradient[4];
                    current.j6 += learningRate * gradient[5];

                    // 检查调整后的新位置是否有效
                    if (isValid(control, current, NearStartNodes, true))
                    {
                        return true; // 找到有效路径
                    }
                }
                else
                {
                    // 计算当前位置的势场梯度  
                    double[] gradient = ApfCalculateMethod(control, current, NearStartNodes, p_start, learningRate, obsList, k_att, k_rep, influenceRadius);

                    // 沿着梯度方向调整位置
                    current.j1 += learningRate * gradient[0];
                    current.j2 += learningRate * gradient[1];
                    current.j3 += learningRate * gradient[2];
                    current.j4 += learningRate * gradient[3];
                    current.j5 += learningRate * gradient[4];
                    current.j6 += learningRate * gradient[5];

                    // 检查调整后的新位置是否有效
                    if (isValid(control, current, NearGoalNodes, false))
                    {
                        return true; // 找到有效路径
                    }



                }


            }


            return false; // 未能找到有效路径
        }

        // 带rcs的插补算法
        public static TxGenericRoboticOperation opCollison;
        public static bool isValidforstepCorssRCS(Control control, ref joint step, joint near, bool stepToNear)
        {

            const int MAX_ITERS = 10;
            int iter = 0;
            while (iter++ < MAX_ITERS)
            {
                bump_rcs = false;
                before = step;

                joint start = stepToNear ? near : step;
                joint end = stepToNear ? step : near;

                TxPoseData startPose = new TxPoseData();

                ArrayList robJointValue = new ArrayList();

                for (int i = 0; i < start.ToArray().Length; i++)
                {
                    robJointValue.Add(start.ToArray()[i]);
                }

                startPose.JointValues = robJointValue;


                TxPoseData endPose = new TxPoseData();

                robJointValue = new ArrayList();

                for (int i = 0; i < end.ToArray().Length; i++)
                {
                    robJointValue.Add(end.ToArray()[i]);
                }

                endPose.JointValues = robJointValue;



                TxGenericRoboticOperationCreationData txCollisioncheckPath = new TxGenericRoboticOperationCreationData();

                txCollisioncheckPath.Robot = robot;
                if (TxrrtRobotPathPlannerForm.robServerGun != null)
                    txCollisioncheckPath.Tool = TxrrtRobotPathPlannerForm.robServerGun as ITxLocatableObject;
                else
                    txCollisioncheckPath.Tool = TxrrtRobotPathPlannerForm.robServerGun as ITxLocatableObject;

                opCollison = TxApplication.ActiveDocument.OperationRoot.CreateGenericRoboticOperation(txCollisioncheckPath);
                robot.CurrentPose = startPose;
                point p = new point(robot.TCPF.AbsoluteLocation.Translation.X,
                        robot.TCPF.AbsoluteLocation.Translation.Y,
                        robot.TCPF.AbsoluteLocation.Translation.Z,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                        start.Sever_Gun);



                TxRoboticViaLocationOperation RobFramepostLocation = addrobotPathViaLoc("start", p, opCollison, robot, TxrrtRobotPathPlannerForm.robServerGun);

                TxRobotConfigurationData txRobotConfigurationData = robot.GetPoseConfiguration(startPose);
                RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;


                robot.CurrentPose = endPose;
                p = new point(robot.TCPF.AbsoluteLocation.Translation.X,
                        robot.TCPF.AbsoluteLocation.Translation.Y,
                        robot.TCPF.AbsoluteLocation.Translation.Z,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                        robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                        end.Sever_Gun);


                RobFramepostLocation = addrobotPathViaLoc("end", p, opCollison, robot, TxrrtRobotPathPlannerForm.robServerGun);

                txRobotConfigurationData = robot.GetPoseConfiguration(endPose);
                RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;
                //TxApplication.ActiveDocument.CurrentOperation = opCollison;
                robot.CurrentPose = startPose;

                bool isvalidforcross = UseSimulationPlayer(opCollison);


                opCollison.Delete();

                if (!isvalidforcross) return false;
                else
                {
                    if (bump_rcs)
                    {
                        step.Sever_Gun = before.Sever_Gun;
                    }
                    else return true;


                }

            }

            return false;
        }

        public static TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxGenericRoboticOperation weldOperation, TxRobot robot, TxServoGun robServerGun)
        {
            TxRoboticViaLocationOperationCreationData roboticViaLocationOperationCreationData = new TxRoboticViaLocationOperationCreationData(name, "", 2.0);

            TxVector tran = new TxVector(p.x, p.y, p.z);

            TxVector rot = new TxVector(p.rx, p.ry, p.rz);

            roboticViaLocationOperationCreationData.AbsoluteLocation = new TxTransformation(tran, rot, TxTransformation.TxRotationType.RPY_XYZ);

            TxRoboticViaLocationOperation RobFramepostLocation = weldOperation.CreateRoboticViaLocationOperation(roboticViaLocationOperationCreationData);
            TxRobotExternalAxisData[] externalAxisData = new TxRobotExternalAxisData[1];
            externalAxisData[0] = new TxRobotExternalAxisData();
            externalAxisData[0].Device = robServerGun as TxServoGun;
            externalAxisData[0].Joint = (robServerGun as TxServoGun).DrivingJoints.Last() as TxJoint;
            externalAxisData[0].JointValue = p.Gun_Open;

            RobFramepostLocation.RobotExternalAxesData = externalAxisData;
            RobFramepostLocation.SetParameter(new TxRoboticIntParam("RRS_MOTION_TYPE", 1));


            return RobFramepostLocation;
        }
        public static TxSimulationPlayer mSimulationPlayer;
        private static bool UseSimulationPlayer(TxGenericRoboticOperation opCollison)
        {

            mSimulationPlayer = new TxSimulationPlayer();
            collisionResult = true;
            //Here I use the current operation that was set active

            ITxOperation tITxOperation = opCollison as ITxOperation;
            mSimulationPlayer.TimeInterval = 0.01;
            TxApplication.Options.Simulation.SimulationSpeed = 100;

            mSimulationPlayer.TimeIntervalReached += player_TimeIntervalReached;
            mSimulationPlayer.SetOperation(tITxOperation);

            mSimulationPlayer.PlayWithoutRefresh();

            mSimulationPlayer.TimeIntervalReached -= player_TimeIntervalReached;
            //mSimulationPlayer.Rewind();
            mSimulationPlayer.JumpSimulationToTime(0.00, false, TxSimulationPlayerSource.TaskSimulationPlayer);
            return collisionResult;

        }

        public static bool collisionResult = true;
        public static joint before;
        public static bool bump_rcs = false;
        private static void player_TimeIntervalReached(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
        {
            using (var pose = robot.CurrentPose)
            {

                double gun_open = (robServerGun.DrivingJoints.Last() as TxJoint).CurrentValue;


                joint p = new joint(
                       (double)pose.JointValues[0], (double)pose.JointValues[1], (double)pose.JointValues[2],
                       (double)pose.JointValues[3], (double)pose.JointValues[4], (double)pose.JointValues[5], gun_open);
                before = p;
                if (!collisioncheckforSingleJoint(TxrrtRobotPathPlannerForm.mainTxControl, ref p))
                {
                    collisionResult = false;
                    mSimulationPlayer.Stop();
                }
                else
                {
                    // 如果碰撞检测微调了 p，就把它当做新的 step 然后跳出重跑
                    if (!JointEquals(p, before))
                    {
                        before.Sever_Gun = p.Sever_Gun;
                        bump_rcs = true;
                        mSimulationPlayer.Stop();

                    }
                }


            }



        }


        // 只用于最终的轨迹优化
        public static bool isValidforstepCorssRCS(
           Control control,
           joint start,
           joint end
          )
        {
            bump_rcs = false;
            TxPoseData startPose = new TxPoseData();

            ArrayList robJointValue = new ArrayList();

            for (int i = 0; i < start.ToArray().Length; i++)
            {
                robJointValue.Add(start.ToArray()[i]);
            }

            startPose.JointValues = robJointValue;


            TxPoseData endPose = new TxPoseData();

            robJointValue = new ArrayList();

            for (int i = 0; i < end.ToArray().Length; i++)
            {
                robJointValue.Add(end.ToArray()[i]);
            }

            endPose.JointValues = robJointValue;



            TxGenericRoboticOperationCreationData txCollisioncheckPath = new TxGenericRoboticOperationCreationData();

            txCollisioncheckPath.Robot = robot;
            if (TxrrtRobotPathPlannerForm.robServerGun != null)
                txCollisioncheckPath.Tool = TxrrtRobotPathPlannerForm.robServerGun as ITxLocatableObject;
            else
                txCollisioncheckPath.Tool = TxrrtRobotPathPlannerForm.robServerGun as ITxLocatableObject;

            opCollison = TxApplication.ActiveDocument.OperationRoot.CreateGenericRoboticOperation(txCollisioncheckPath);
            robot.CurrentPose = startPose;
            point p = new point(robot.TCPF.AbsoluteLocation.Translation.X,
                    robot.TCPF.AbsoluteLocation.Translation.Y,
                    robot.TCPF.AbsoluteLocation.Translation.Z,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                    start.Sever_Gun);



            TxRoboticViaLocationOperation RobFramepostLocation = addrobotPathViaLoc("start", p, opCollison, robot, TxrrtRobotPathPlannerForm.robServerGun);

            TxRobotConfigurationData txRobotConfigurationData = robot.GetPoseConfiguration(startPose);
            RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;


            robot.CurrentPose = endPose;
            p = new point(robot.TCPF.AbsoluteLocation.Translation.X,
                    robot.TCPF.AbsoluteLocation.Translation.Y,
                    robot.TCPF.AbsoluteLocation.Translation.Z,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                    end.Sever_Gun);


            RobFramepostLocation = addrobotPathViaLoc("end", p, opCollison, robot, TxrrtRobotPathPlannerForm.robServerGun);
            txRobotConfigurationData = robot.GetPoseConfiguration(endPose);
            RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;
            //TxApplication.ActiveDocument.CurrentOperation = opCollison;
            robot.CurrentPose = startPose;
            bool isvalidforcross = UseSimulationPlayerforoptimize(opCollison);

            opCollison.Delete();
            return isvalidforcross;

        }

        private static bool UseSimulationPlayerforoptimize(TxGenericRoboticOperation opCollison)
        {

            mSimulationPlayer = new TxSimulationPlayer();
            collisionResult = true;
            //Here I use the current operation that was set active

            ITxOperation tITxOperation = opCollison as ITxOperation;
            mSimulationPlayer.TimeInterval = 0.01;
            TxApplication.Options.Simulation.SimulationSpeed = 100;

            mSimulationPlayer.TimeIntervalReached += player_TimeIntervalReachedOptimize;
            mSimulationPlayer.SetOperation(tITxOperation);

            mSimulationPlayer.PlayWithoutRefresh();

            mSimulationPlayer.TimeIntervalReached -= player_TimeIntervalReachedOptimize;
            //mSimulationPlayer.Rewind();
            mSimulationPlayer.JumpSimulationToTime(0.00, false, TxSimulationPlayerSource.TaskSimulationPlayer);
            return collisionResult;

        }

        private static void player_TimeIntervalReachedOptimize(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
        {
            using (var pose = robot.CurrentPose)
            {

                double gun_open = (robServerGun.DrivingJoints.Last() as TxJoint).CurrentValue;


                joint p = new joint(
                       (double)pose.JointValues[0], (double)pose.JointValues[1], (double)pose.JointValues[2],
                       (double)pose.JointValues[3], (double)pose.JointValues[4], (double)pose.JointValues[5], gun_open);
                before = p;
                if (!collisioncheckforSingleJoint(TxrrtRobotPathPlannerForm.mainTxControl, ref p))
                {
                    collisionResult = false;
                    mSimulationPlayer.Stop();
                }

            }



        }

    }
}