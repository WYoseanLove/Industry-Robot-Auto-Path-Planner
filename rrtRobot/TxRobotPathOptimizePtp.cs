using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Tecnomatix.Engineering;

namespace rrtRobot
{
    public class TxRobotPathOptimizePtp
    {
        /*TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation, TxRobot robot)
         * 在当前选择计算的焊接轨迹中增加过度点；
         * 1. name:定义过度点的名称；
         * 2. p：过度点的坐标值；
         * 3. weldOperation：计算或增加过度点的轨迹；
         * 4. robot-当前计算和规划路径的机器人；
         * 每个新增加的过度点都定义了相对应的Tooling设备，Tooling的角度值，以及过度点的运动类型，PTP or Line
         * 
         * This code snippet describes adding via points (intermediate points) to a welding trajectory. Let's break down each point:
         * name (1): This assigns a unique identifier or name to the via point. This is useful for tracking and debugging.
         * p (2): This represents the 3D Cartesian coordinates (x, y, z) of the via point in the robot's workspace.
         * weldOperation (3): This likely refers to the welding operation associated with the via point. This could include details like welding parameters (e.g., current, speed, etc.) or instructions for the robot's welding tool. The description implies that this element is either calculated to determine the via point or is used to add the via point to the existing welding trajectory.
         * robot (4): Similar to the previous example, this specifies the robot for which the path is being planned.
         * Tooling, Angle, and Motion Type (5): Each via point is further defined by:
         * Tooling: Specifies the end-effector or tool being used at this via point. This might be a specific welding torch or other tool attached to the robot.
         * Angle: This refers to the orientation of the tooling at the via point. Accurate orientation is critical for successful welding.
         * Motion Type (PTP or Line): This dictates the type of robot motion used to reach the via point
         * PTP (Point-to-Point): The robot moves directly to the point, ignoring the path taken. This is generally faster but less precise.
         * Line: The robot moves along a straight line to the point. This provides more control over the path but might be slower.
         */
        public static TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation, TxRobot robot, TxServoGun robServerGun)
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
        //比较两个角度，并考虑角度的周期性问题(360deg) Compare two angles, taking into account their periodicity (360 degrees).
        private static double AngleDifference(double angle1, double angle2)
        {
            double diff = Math.Abs(angle1 - angle2);
            return Math.Min(diff, 2 * Math.PI - diff);
        }
        //计算某个逆向解与上一个点的总关节角度变化  Calculate the total joint angle change between a given inverse kinematic solution and the previous point.
        private static double CalculateTotalDifference(List<double> current_Solution, List<double> pre_Solution)
        {

            double totalDifference = 0;

            for (int i = 0; i < current_Solution.Count; i++)
            {
                totalDifference += AngleDifference(current_Solution[i], pre_Solution[i]);

            }

            return totalDifference;


        }
        /*static int ChooseBestInverseSolution(ArrayList current_Solution, TxPoseData pre_txPoseData,bool AllJointsorNot)
         * 从所有逆向解列表中选出最优的逆向解，并定义为机器人在该点处的姿态：
         * 1. List<double> current_Solution 所有逆向解的列表；
         * 2. pre_txPoseData 与之前的逆向解进行对比，并挑选出最合适的逆向解；
         * 3. AllJointsorNot 是否考虑所有的关节角度；
         *    如果为true, 则计算所有的6个关节角度，并选出关节角度之和变化最小的逆向解；
         *    如果为false, 则只计算1轴，4轴和6轴，此时已默认机器人的ST值进行了定义
         * The optimal inverse kinematic solution is selected from the list of available solutions and defined as the robot's pose at that point.
         * 1. Inverse Kinematic Solutions: List<double> current_Solution contains all calculated inverse kinematic solutions.
         * 2. Comparison with Previous Solution: pre_txPoseData represents the previous inverse kinematic solution. The optimal solution from current_Solution is selected based on its similarity to pre_txPoseData.
         * 3. Joint Angle Consideration (AllJointsorNot): This parameter determines which joint angles are considered when selecting the optimal solution.
         *    true: All six joint angles are considered. The solution with the minimum total change in joint angles (sum of absolute differences) compared to pre_txPoseData is selected.
         *    false: Only joints 1, 4, and 6 are considered. This assumes that the robot's ST values (likely referring to specific robot parameters or configurations) have already been defined.
         * 
         */
        public static int ChooseBestInverseSolution(ArrayList current_Solution, TxPoseData pre_txPoseData, bool AllJointsorNot)
        {

            double minDifference = double.MaxValue;
            int bestSulotion = int.MaxValue;

            List<double> pre_Solution = new List<double>();
            if (AllJointsorNot)
            {
                pre_Solution.Add((double)pre_txPoseData.JointValues[0]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[1]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[2]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[3]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[4]);
                pre_Solution.Add((double)pre_txPoseData.JointValues[5]);
            }
            else
            {
                pre_Solution.Add((double)pre_txPoseData.JointValues[0]);

                pre_Solution.Add((double)pre_txPoseData.JointValues[3]);

                pre_Solution.Add((double)pre_txPoseData.JointValues[5]);
            }

            for (int i = 0; i < current_Solution.Count; i++)
            {
                List<double> varSolution = new List<double>();
                if (AllJointsorNot)
                {
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[0]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[1]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[2]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[3]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[4]);
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[5]);
                }
                else
                {
                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[0]);

                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[3]);

                    varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[5]);

                }

                double totalDifference = CalculateTotalDifference(varSolution, pre_Solution);

                if (totalDifference < minDifference)
                {
                    minDifference = totalDifference;
                    bestSulotion = i;
                }

            }



            return bestSulotion;
        }
        /*static ArrayList filterRobotConfigSolution(TxRobotConfigurationData pre_locationConfigData, ArrayList current_Solution,TxRobot robot)
         * 从所有的逆向解列表中挑选出来与之前逆向解ST值相同的逆向解，并加入到新的逆向解清单作为筛选出的逆向解；
         * 1. TxRobotConfigurationData pre_locationConfigData 前一个机器人姿态的ST值；
         * 2. List<double> current_Solution 所有逆向解的列表；
         * 3. robot 参与计算和定义逆向解姿态的机器人；
         * 
         * This process selects inverse kinematic solutions from a list that match the ST values of a previous solution and adds them to a new list of filtered solutions.
         * 1. Previous Robot Pose: TxRobotConfigurationData pre_locationConfigData stores the ST values (likely representing specific robot configuration parameters) of the previous robot pose.
         * 2. Inverse Kinematic Solutions: List<double> current_Solution contains all calculated inverse kinematic solutions.
         * 3. Robot: robot specifies the robot used for calculating and defining the inverse kinematic poses.
         */
        public static int ChooseBestInverseSolution(ref ArrayList current_Solution, TxPoseData pre_txPoseData)
        { // 周期常量：度单位下是 360；如果是弧度请改为 2*Math.PI
            const double cycle = 2 * Math.PI;
            double minDifference = double.MaxValue; int bestSolution = -1;

            // 1) 读出上一次解的角度
            List<double> preSolution = new List<double>();

            for (int j = 0; j < 6; j++)
                preSolution.Add((double)pre_txPoseData.JointValues[j]);


            // 2) 遍历所有逆解候选
            for (int i = 0; i < current_Solution.Count; i++)
            {
                TxPoseData new_Solution = new TxPoseData();
                ArrayList newPosedata = new ArrayList();
                var pose = (TxPoseData)current_Solution[i];
                List<double> varSolution = new List<double>();

                for (int j = 0; j < 6; j++)
                    varSolution.Add((double)pose.JointValues[j]);
                // 对第 3、5 轴做模周期调整
                double temp = AdjustAngle(varSolution[3], preSolution[3], cycle);
                if ((temp >= TxrrtRobotPathPlannerForm.robot.Joints[3].LowerSoftLimit) && (temp <= TxrrtRobotPathPlannerForm.robot.Joints[3].UpperSoftLimit))
                {
                    varSolution[3] = temp;
                }
                //varSolution[3] = AdjustAngle(varSolution[3], preSolution[3], cycle);
                temp = AdjustAngle(varSolution[5], preSolution[5], cycle);
                if ((temp >= TxrrtRobotPathPlannerForm.robot.Joints[5].LowerSoftLimit) && (temp <= TxrrtRobotPathPlannerForm.robot.Joints[5].UpperSoftLimit))
                {
                    varSolution[5] = temp;
                }

                for (int j = 0; j < varSolution.Count; j++)
                {
                    newPosedata.Add(varSolution[j]);
                }

                // 3) 计算总差异并更新最优解索引
                double diff = CalculateTotalDifference(varSolution, preSolution);
                if (diff < minDifference)
                {
                    minDifference = diff;
                    bestSolution = i;
                }
                new_Solution.JointValues = newPosedata;
                current_Solution[i] = new_Solution;
            }

            return bestSolution;
        }
        /// <summary>
        /// 把 angle 通过 ±cycle 的整数倍平移到最接近 target 的区域，
        /// 即让 (angle - target) 落在 [-cycle/2, +cycle/2] 区间内。
        /// </summary>
        private static double AdjustAngle(double angle, double target, double cycle)
        {
            // 计算最优的整数倍 n
            double n = Math.Round((target - angle) / cycle);
            // 返回调整后的角度
            return angle + n * cycle;
        }

        public static ArrayList filterRobot3rdConfigSolution(TxRobotConfigurationData pre_locationConfigData, ArrayList current_Solution, TxRobot robot)
        {
            ArrayList filter_Solutions = new ArrayList();

            for (int i = 0; i < current_Solution.Count; i++)
            {
                // Add J3 & J5 Configuration data into filter function to tighten the ST value;
                TxRobotConfigurationData locationConfigData = robot.GetPoseConfiguration((TxPoseData)current_Solution[i]);
                if (
                    (locationConfigData.OverheadState == pre_locationConfigData.OverheadState)
                    &&
                    (
                    (locationConfigData.JointsConfigurations[2] as TxJointConfigurationData).State
                    == (pre_locationConfigData.JointsConfigurations[2] as TxJointConfigurationData).State
                    )
                    &&
                    (
                    (locationConfigData.JointsConfigurations[4] as TxJointConfigurationData).State
                    == (pre_locationConfigData.JointsConfigurations[4] as TxJointConfigurationData).State
                    )
                    )
                {
                    filter_Solutions.Add((TxPoseData)current_Solution[i]);
                }


            }


            return filter_Solutions;


        }
        static int FindIndex(TxObjectList list, string name)
        {
            for (int i = 0; i < list.Count; i++)
            {
                // 从 ArrayList 中获取对象并进行类型转换
                ITxObject target = list[i] as ITxObject;
                if (target != null && target.Name == name)
                {
                    return i;
                }
            }
            return -1; // 如果未找到匹配项，则返回 -1
        }
        public static bool OperationOptimize(ref TxWeldOperation _robweldOperation, TxRobot robot)
        {

            TxTypeFilter opFilter = new TxTypeFilter();
            opFilter.AddIncludedType(typeof(TxWeldLocationOperation));
            opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotWeldOpLocation = _robweldOperation.GetAllDescendants(opFilter);
            int targetSeq = 0;
            for (int i = 0; i < _robotWeldOpLocation.Count; i++)
            {

                if (_robotWeldOpLocation[i].Name != (TxrrtRobotPathPlannerForm.TargetCalLocNames)[targetSeq])
                {
                    _robotWeldOpLocation.RemoveAt(i);
                    i--;
                }
                else
                    targetSeq++;
            }
            //筛选出轨迹路径中的焊点和过渡点位

            TxTypeFilter opFilterpass = new TxTypeFilter();
            opFilterpass.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotviaOpLocation = _robweldOperation.GetAllDescendants(opFilterpass);
            opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotOpLocation = _robweldOperation.GetAllDescendants(opFilter);
            /*
             * _robotWeldOpLocation包括所有的焊点信息
             * _robotviaOpLocation包括所有的过渡点信息；
             * _robotOpLocation包括所有的焊点+过渡点的信息；
             * 
             */
            for (int i = 0; i < _robotWeldOpLocation.Count - 1; i++)
            {

                //第一个焊点在轨迹中的位置
                int _startIndex = FindIndex(_robotOpLocation, _robotWeldOpLocation[i].Name);
                //紧邻的第二个焊点在轨迹中的位置；
                int _endIndex = FindIndex(_robotOpLocation, _robotWeldOpLocation[i + 1].Name);
                _startIndex += 1; //紧邻第一个焊点的过渡点在轨迹中的位置；
                _endIndex -= 1;//紧邻第二个焊点的过渡点在轨迹中的位置；

                for (int j = _startIndex + 1; j < _endIndex; j++) // j从前往后，k是从后往前
                {

                    for (int k = _endIndex - 1; k > j; k--)
                    {
                        TxPoseData targetPoseDatastart = new TxPoseData();
                        TxPoseData targetPoseDataend = new TxPoseData();
                        try
                        {
                            targetPoseDatastart = robot.GetPoseAtLocation((_robotOpLocation[j] as TxRoboticViaLocationOperation));
                        }
                        catch (Exception)
                        {

                            TxRobotRRTConnectJoint.logpathGenerateOK(_robotOpLocation[j].Name + " posture get failed");
                            continue;

                        }
                        try
                        {
                            targetPoseDataend = robot.GetPoseAtLocation((_robotOpLocation[k] as TxRoboticViaLocationOperation));
                        }
                        catch (Exception)
                        {

                            TxRobotRRTConnectJoint.logpathGenerateOK(_robotOpLocation[k].Name + " posture get failed");
                            continue;

                        }


                        joint start = new joint(

                            (double)targetPoseDatastart.JointValues[0],
                            (double)targetPoseDatastart.JointValues[1],
                            (double)targetPoseDatastart.JointValues[2],
                            (double)targetPoseDatastart.JointValues[3],
                            (double)targetPoseDatastart.JointValues[4],
                            (double)targetPoseDatastart.JointValues[5],
                            (_robotOpLocation[j] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                            );

                        joint end = new joint(

                           (double)targetPoseDataend.JointValues[0],
                           (double)targetPoseDataend.JointValues[1],
                           (double)targetPoseDataend.JointValues[2],
                           (double)targetPoseDataend.JointValues[3],
                           (double)targetPoseDataend.JointValues[4],
                           (double)targetPoseDataend.JointValues[5],
                           (_robotOpLocation[k] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                           );

                        if (TxrrtRobotPathPlannerForm.isRCSLoaded)
                        {
                            if (TxRobotRRTConnectJoint.isValidforstepCorssRCS(TxrrtRobotPathPlannerForm.mainTxControl, start, end))
                            {

                                while (j != k - 1)
                                {
                                    //将j后面的过渡点删掉，由于删除了过渡点，则_endIndex 和k值也需要相应的减少index;
                                    _robotOpLocation[j + 1].Delete();
                                    _robotOpLocation.RemoveAt(j + 1);
                                    k--;
                                    _endIndex--;

                                }
                                // j 移动到删除点之后的，再次寻找是否可以找到删除的点;
                                j = k + 1;
                                break;
                            }
                            else
                                continue;
                        }
                        else
                        {
                            if (TxRobotRRTConnectJoint.isValidforstepCorss(TxrrtRobotPathPlannerForm.mainTxControl, start, end))
                            {

                                while (j != k - 1)
                                {
                                    //将j后面的过渡点删掉，由于删除了过渡点，则_endIndex 和k值也需要相应的减少index;
                                    _robotOpLocation[j + 1].Delete();
                                    _robotOpLocation.RemoveAt(j + 1);
                                    k--;
                                    _endIndex--;

                                }
                                // j 移动到删除点之后的，再次寻找是否可以找到删除的点;
                                j = k + 1;
                                break;
                            }
                            else
                                continue;
                        }


                    }

                }
            }

            return true;

        }

    }
}
