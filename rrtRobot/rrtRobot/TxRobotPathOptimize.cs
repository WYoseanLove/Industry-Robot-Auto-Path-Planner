using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;

namespace rrtRobot
{
    
    public class TxRobotPathOptimize
    {
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
        public static bool OperationOptimize2(ref TxWeldOperation _robweldOperation, TxRobot robot)
        {
            TxTypeFilter opFilter = new TxTypeFilter();
            opFilter.AddIncludedType(typeof(TxWeldLocationOperation));
            //opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotWeldOpLocation = _robweldOperation.GetAllDescendants(opFilter);
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
                               //如果前一个点为焊点轨迹，或者后一个点为焊点轨迹，则改变这个过渡点的rx/ry/rz数值
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                (_robotOpLocation[_startIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[_startIndex - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;

                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                (_robotOpLocation[_endIndex] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[_endIndex + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;
                for (int j = _startIndex + 1; j < _endIndex; j++) // j从前往后，k是从后往前
                {

                    for (int k = _endIndex - 1; k > j; k--)
                    {

                        point start = new point(

                       (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                       );

                        point end = new point(

                           (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                          (_robotOpLocation[k] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                         );

                        point p = new point();
                        if (TxRobotRRTConnect.isValidforstepCorss(start, end, out p))
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

            //进行每个过渡点的robot teach 
            TxPoseData txPoseDataSpot = new TxPoseData();
            for (int i = 0; i < _robotOpLocation.Count; i++)
            {

                if (_robotOpLocation[i].GetType() == typeof(TxWeldLocationOperation))
                {
                   
                    txPoseDataSpot = robot.GetPoseAtLocation(_robotOpLocation[i] as ITxRoboticLocationOperation);


                }
                else
                {
                    point pathPoint = new point(

                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[i] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                       );

                    ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, pathPoint);

                    if (Solutions.Count == 0) continue;


                    int besSolution = ChooseBestInverseSolution(Solutions, txPoseDataSpot);

                    if (besSolution > Solutions.Count) continue;

                    TxPoseData txPoseData = (TxPoseData)Solutions[besSolution];


                    TxRobotConfigurationData locationConfigData = robot.GetPoseConfiguration(txPoseData);

                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).RobotConfigurationData = locationConfigData;

                    txPoseDataSpot = txPoseData;



                }



            }


            return true;

        }

        //比较两个角度，并考虑角度的周期性问题(360deg)
        private static double AngleDifference(double angle1, double angle2)
        {
            double diff = Math.Abs(angle1 - angle2);
            return Math.Min(diff, 2 * Math.PI - diff);
        }
        //计算某个逆向解与上一个点的总关节角度变化
        private static double CalculateTotalDifference(List<double> current_Solution, List<double> pre_Solution)
        {

            double totalDifference = 0;

            for (int i = 0; i < current_Solution.Count; i++)
            {
                totalDifference += AngleDifference(current_Solution[i], pre_Solution[i]);

            }

            return totalDifference;


        }

        private static int ChooseBestInverseSolution(ArrayList current_Solution, TxPoseData pre_txPoseData)
        {

            double minDifference = double.MaxValue;
            int bestSulotion = int.MaxValue;

            List<double> pre_Solution = new List<double>();
            pre_Solution.Add((double)pre_txPoseData.JointValues[0]);
            pre_Solution.Add((double)pre_txPoseData.JointValues[1]);
            pre_Solution.Add((double)pre_txPoseData.JointValues[2]);
            pre_Solution.Add((double)pre_txPoseData.JointValues[3]);
            pre_Solution.Add((double)pre_txPoseData.JointValues[4]);
            pre_Solution.Add((double)pre_txPoseData.JointValues[5]);

            for (int i = 0; i < current_Solution.Count; i++)
            {
                List<double> varSolution = new List<double>();
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[0]);
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[1]);
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[2]);
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[3]);
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[4]);
                varSolution.Add((double)((TxPoseData)(current_Solution[i])).JointValues[5]);

                double totalDifference = CalculateTotalDifference(varSolution, pre_Solution);

                if (totalDifference < minDifference)
                {
                    minDifference = totalDifference;
                    bestSulotion = i;
                }

            }



            return bestSulotion;
        }


        private bool OperationOptimize(ref TxWeldOperation _robweldOperation)
        {
            /*  对rrt*start 生成的机器人自动轨迹进行优化，优化的内容包括:
             * 1. 遍历每两个焊点之间的轨迹坐标点，根据冒泡的排序的算法，从后往前进行计算确认此轨迹点是否能到达最后一个的轨迹点，如果能则删除之间所有的轨迹点，不能则留下，让留下的轨迹点作为起始点继续遍历后面的；
             * 2. 焊点后面的轨迹点需要与焊点保持同样的焊接姿态，不能突然变换姿态，减少机器人姿态的波动；
             * 
             */
            TxTypeFilter opFilter = new TxTypeFilter();
            opFilter.AddIncludedType(typeof(TxWeldLocationOperation));
            opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList _robotOpLocation = _robweldOperation.GetAllDescendants(opFilter);
            //筛选出轨迹路径中的焊点和过渡点位

            for (int i = 0; i < _robotOpLocation.Count; i++)
            {
                if (_robotOpLocation[i].GetType() == typeof(TxWeldLocationOperation))//如果是点位为焊点，则继续
                    continue;

                if ((i > 0) && (_robotOpLocation[i - 1].GetType() == typeof(TxWeldLocationOperation)))//如果前一个点为焊点轨迹，或者后一个点为焊点轨迹，则改变这个过渡点的rx/ry/rz数值
                {

                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[i - 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;
                    continue;
                }
                else if ((i < _robotOpLocation.Count - 1) && (_robotOpLocation[i + 1].GetType() == typeof(TxWeldLocationOperation)))
                {
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X = (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y = (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y;
                    (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z = (_robotOpLocation[i + 1] as TxWeldLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z;
                    continue;

                }

                for (int j = i + 2; j < _robotOpLocation.Count; j++)//对过渡点操作
                {
                    if (_robotOpLocation[j].GetType() == typeof(TxWeldLocationOperation)) break;

                    point start = new point(

                        (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                       (_robotOpLocation[i] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                        );

                    point end = new point(

                       (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.Translation.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.X,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Y,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).AbsoluteLocation.RotationRPY_XYZ.Z,
                      (_robotOpLocation[j] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue
                       );
                    point p = new point();
                    if (TxRobotRRTConnect.isValidforstepCorss(start, end, out p))
                    {

                        //TxApplication.ActiveDocument.OperationRoot.AddObject(_robotOpLocation[j - 1]);
                        _robotOpLocation[j - 1].Delete();
                        _robotOpLocation.RemoveAt(j - 1);
                        j--;
                    }
                    else
                    {
                        i = j - 1;
                        TxRobotRRTConnect.LogInformation(p, new List<double>() { 0.0, 0.0, 0.0 });

                        break;
                    }

                }


            }


            return true;
        }
        public static TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation,TxRobot robot)
        {
            TxRoboticViaLocationOperationCreationData roboticViaLocationOperationCreationData = new TxRoboticViaLocationOperationCreationData(name, "", 2.0);

            TxVector tran = new TxVector(p.x, p.y, p.z);

            TxVector rot = new TxVector(p.rx, p.ry, p.rz);

            roboticViaLocationOperationCreationData.AbsoluteLocation = new TxTransformation(tran, rot, TxTransformation.TxRotationType.RPY_XYZ);

            TxRoboticViaLocationOperation RobFramepostLocation = weldOperation.CreateRoboticViaLocationOperation(roboticViaLocationOperationCreationData);
            TxRobotExternalAxisData[] externalAxisData = new TxRobotExternalAxisData[1];
            externalAxisData[0] = new TxRobotExternalAxisData();
            externalAxisData[0].Device = robot.MountedTools[0] as TxServoGun;
            externalAxisData[0].Joint = (robot.MountedTools[0] as TxServoGun).Joints.Last();
            externalAxisData[0].JointValue = p.Gun_Open;

            RobFramepostLocation.RobotExternalAxesData = externalAxisData;

            //ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, p);
            //TxPoseData txPoseData = (TxPoseData)Solutions[0];


            // TxRobotConfigurationData locationConfigData = robot.GetPoseConfiguration(txPoseData);

            //RobFramepostLocation.RobotConfigurationData = locationConfigData;

            RobFramepostLocation.SetParameter(new TxRoboticIntParam("RRS_MOTION_TYPE", 2));


            return RobFramepostLocation;
        }


    }
}
