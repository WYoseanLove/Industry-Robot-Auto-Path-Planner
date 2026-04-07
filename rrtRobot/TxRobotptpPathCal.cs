using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using static System.Windows.Forms.AxHost;

namespace rrtRobot
{
    public class TxRobotptpPathCal
    {
        /* The `TxRobotptpPathCal` class calculates the interpolation algorithm for a PTP (Point-to-Point) trajectory of an industrial robot.
         * First, it obtains the angular change of the six joint values between two PTP trajectory points of the robot. This is achieved by the function `calculateJointsChange(joint start, joint end, TxRobot robot)`.
         * From the angular changes of the six joint values, the axis with the largest angular change is determined. The largest angular change value will determine the motion time between the PTP points. The maximum speed and maximum acceleration of each axis of the industrial robot are directly obtained through the TECNOMATIX API.
         * The maximum axis speed is `robot.Joints[i].MaxSpeed`, and the maximum axis acceleration is `robot.Joints[i].MaxAcceleration`.
         * Based on the maximum angular change of the axis, the maximum speed, and the maximum acceleration, the motion time is calculated using the function `calculatePTPtime(Control control, List<double> jointschange, TxRobot robot)`.
         * The speed profile of the axis values is a trapezoidal diagram. First, it is confirmed whether the actual maximum speed `Maxspeedactual` reached during axis acceleration is equal to the theoretical maximum speed:
         * Maxspeedactual = Min(sqrt(acc*α), MaxSpeedTarget)`.  (Note: α here is likely intended to be alpha, representing angular displacement)
         * If `Maxspeedactual == sqrt(acc*α)`:
         * The time is: `2*sqrt(α/acc)
         * If `Maxspeedactual == MaxSpeedTarget`:
         * The time is: `MaxSpeedTarget/acc + α/MaxSpeedTarget`
         * `MaxSpeedTarget`, `acc`, and `α` are all angular values and can be expressed in degrees or radians.
         * After obtaining the motion time of the joint axis with the largest angular change, interpolation is performed based on this time. The time can be divided into several segments, and the axis values of the other joint axes at these time points are calculated, thereby determining the robot's pose.
         * The relationship between time *t*, angular change *α*, `Maxspeedactual`, and *acc* is:
         * `Maxspeedactual^2 - t * acc * Maxspeedactual + acc * α = 0`
         * If time *t* is known, solving for `Maxspeedactual` requires solving a quadratic equation. This will yield two solutions, which need to be compared and filtered against 0 and `MaxSpeedTarget`.
         * `Maxspeedactual = (t*acc - sqrt(t^2*acc^2 - 4*acc*α))/2`
         * TxPoseData calCurrentRobotAixJointData(Control control, double ptpTime, double t, int JointNumber, TxRobot robot, double JointchangeData, double Joint_start)` is used to calculate the axis value of each joint axis at time point *t*.
         * `TxPoseData calCurrentRobotPosedata(Control control, joint start, joint end, TxRobot robot, double t)` uses the axis values of the six axes at time point *t* to determine the robot's pose.
         *    
         * 
         * TxRobotptpPathCal 类计算工业机器人ptp轨迹的插补算法；
         * 首先获取机器人ptp两个轨迹点之间六个轴值的角度变化，实现函数为calculateJointsChange(joint start, joint end, TxRobot robot)；
         * 从6个轴值的角度变化中获取最大的角度变化轴值，最大的角度变化轴值将决定了ptp之间的运动时间，工业机器人的每个轴的最大速度和最大加速度通过tecnomatix api直接获取；
         * 最大的轴值速度为 robot.Joints[i].MaxSpeed，最大的轴值加速度为robot.Joints[i].MaxAcceleration；
         * 根据最大的轴值变化角度，最大速度和最大加速度，来计算运动时间，实现函数为calculatePTPtime(Control control, List<double> jointschange, TxRobot robot)；
         * 轴值的速度曲线为梯形图，首先确认轴值加速到最大速度Maxspeedactual是否达到了理论最大速度：
         * 	Ø Maxspeedactual=Min(sqrt(acc*α),MaxSpeedTarget );
         * 	如果Maxspeedactual==sqrt(acc*α)；
         * 	时间为：2*sqrt(α/acc)
         * 	如果Maxspeedactual==MaxSpeedTarget 
         * 	时间为：MaxSpeedTarget/acc+α/MaxSpeedTarget
         * 	Ø MaxSpeedTarget, acc,α均为角度值，也可以同为弧度值
         * 	获取最大角度变化关节轴的运动时间之后，根据这个时间进行插补，可以将时间分为若干份，根据时间计算其他关节轴在这个时间点的轴值，从而决定机器人的姿态；
         * 	
         * 	时间t, 角度变化α， Maxspeedactual, acc之间的关系是：
         * 	Ø Maxspeedactual2-t *acc*Maxspeedactual +acc*α=0;
         * 	如果已知时间t，求Maxspeedactual 需要求解一元二次方程，会得到两个解，并于0和MaxSpeedTarget进行比较和筛选
         * 	Maxspeedactual =(t*acc - sqrt(t2*acc2-4*acc*α))/2;
         * 	TxPoseData calCurrentRobotAixJointData(Control control, double ptpTime, double t, int JointNumber, TxRobot robot, double JointchangeData, double Joint_start) 用于计算每个关节轴的在时间点t的轴值
         * 	TxPoseData calCurrentRobotPosedata(Control control, joint start, joint end, TxRobot robot, double t) 为六个轴在时间点t时的轴值来确定机器人姿态
         */
        // 将角度规范化到0到360度之间
        private static double NormalizeAngle(double angle, double targetAngle)
        {
            double twoPi = 2 * Math.PI;

            // 将角度差值规范化到 -π 到 π 之间
            double difference = angle - targetAngle;
            difference = difference % twoPi;

            if (difference > Math.PI)
            {
                difference -= twoPi; // 如果差值大于 π，通过减去2π使其更接近目标
            }
            else if (difference < -Math.PI)
            {
                difference += twoPi; // 如果差值小于 -π，通过加上2π使其更接近目标
            }

            // 计算最接近目标的角度
            double closestAngle = targetAngle + difference;

            return closestAngle;
        }

        public static TxPoseData calculateStartPointPosture(Control control, point start, TxPoseData targetPoseData, TxRobot robot)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (TxPoseData)control.Invoke(new Func<Control, point, TxPoseData, TxRobot, TxPoseData>(
                    calculateStartPointPosture), control, start, targetPoseData, robot);
            }
            else
            {
                // Execute the main logic of the function
                int bestSolution_start = int.MaxValue;
                TxRobotConfigurationData targetConfigData = robot.GetPoseConfiguration(targetPoseData);
                ArrayList Solutions_start = TxRobotAPIClass.robotInverseCal(control, robot, start);
                ArrayList filter_Solutions = new ArrayList();
                filter_Solutions = TxRobotPathOptimizePtp.filterRobot3rdConfigSolution(targetConfigData, Solutions_start, robot);

                if (filter_Solutions.Count != 0)
                {
                    int bestIndex = TxRobotPathOptimizePtp.ChooseBestInverseSolution(filter_Solutions, targetPoseData, false);
                    bestSolution_start = Solutions_start.IndexOf(filter_Solutions[bestIndex]);
                }
                else
                {
                    bestSolution_start = TxRobotPathOptimizePtp.ChooseBestInverseSolution(Solutions_start, targetPoseData, true);
                }

                // 将四轴和6轴的角度值进行归一化，变成最接近0度的解
                ArrayList RobJointValue = new ArrayList();
                TxPoseData txPoseData = Solutions_start[bestSolution_start] as TxPoseData;
                RobJointValue.Add(txPoseData.JointValues[0]);
                RobJointValue.Add(txPoseData.JointValues[1]);
                RobJointValue.Add(txPoseData.JointValues[2]);

                double normalizedAngle = NormalizeAngle((double)txPoseData.JointValues[3], (double)targetPoseData.JointValues[3]);
                if (Math.Abs(normalizedAngle) <= robot.Joints[3].UpperSoftLimit)
                {
                    RobJointValue.Add(NormalizeAngle((double)txPoseData.JointValues[3], (double)targetPoseData.JointValues[3]));
                }
                else
                {
                    RobJointValue.Add(txPoseData.JointValues[3]);
                }

                RobJointValue.Add(txPoseData.JointValues[4]);

                normalizedAngle = NormalizeAngle((double)txPoseData.JointValues[5], (double)targetPoseData.JointValues[5]);
                if (Math.Abs(normalizedAngle) <= robot.Joints[5].UpperSoftLimit)
                {
                    RobJointValue.Add(NormalizeAngle((double)txPoseData.JointValues[5], (double)targetPoseData.JointValues[5]));
                }
                else
                {
                    RobJointValue.Add(txPoseData.JointValues[5]);
                }

                txPoseData.JointValues = RobJointValue;

                return txPoseData;

            }
        }

        public static List<double> calculateJointsChange(Control control, point start, point end, TxPoseData targetstartPoseData, TxPoseData targetendPoseData, TxRobot robot)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (List<double>)control.Invoke(new Func<Control, point, point, TxPoseData, TxPoseData, TxRobot, List<double>>(
                    calculateJointsChange), control, start, end, targetstartPoseData, targetendPoseData, robot);
            }
            else
            {
                // Execute the main logic of the function
                TxPoseData startPose = calculateStartPointPosture(control, start, targetstartPoseData, robot);
                TxPoseData endPose = calculateStartPointPosture(control, end, targetendPoseData, robot);
                List<double> jointschange = new List<double>();

                for (int i = 0; i < 6; i++)
                {
                    jointschange.Add((double)(endPose).JointValues[i] -
                                     (double)(startPose).JointValues[i]);
                }
                //startPose.Dispose();
                //endPose.Dispose();

                return jointschange;
            }
        }
        public static List<double> calculateJointsChange(joint start, joint end, TxRobot robot)
        {

            List<double> jointschange = new List<double>();

            for (int i = 0; i < 6; i++)
            {

                jointschange.Add(end.ToArray()[i] - start.ToArray()[i]);
            }

            return jointschange;
        }
       
        public static double  calculateServoGunJointChange(joint start, joint end, TxRobot robot)
        {

            return end.Sever_Gun-start.Sever_Gun;
        }
        

        
        
        public static (double value, int index) FindLargestAbsoluteWithIndex(List<double> numbers)
        {
            if (numbers == null || numbers.Count == 0)
            {
                throw new ArgumentException("The list cannot be null or empty.");
            }

            var max = numbers.Select((x, i) => (Math.Abs(x), i))
                             .OrderByDescending(x => x.Item1)
                             .First();

            return (max.Item1, max.Item2);
        }
        public static double calculatePTPtime(Control control, List<double> jointschange, TxRobot robot)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (double)control.Invoke(new Func<Control, List<double>, TxRobot, double>(
                    calculatePTPtime), control, jointschange, robot);
            }
            else
            {
                // Execute the main logic of the function
                double[] MaxSpeedTarget =
                {
                    robot.Joints[0].MaxSpeed, robot.Joints[1].MaxSpeed,  robot.Joints[2].MaxSpeed,
                    robot.Joints[3].MaxSpeed, robot.Joints[4].MaxSpeed, robot.Joints[5].MaxSpeed
                 };

                double[] acc =
                {
                    robot.Joints[0].MaxAcceleration, robot.Joints[1].MaxAcceleration,  robot.Joints[2].MaxAcceleration,
                    robot.Joints[3].MaxAcceleration, robot.Joints[4].MaxAcceleration, robot.Joints[5].MaxAcceleration
                };

                (double value, int index) result = FindLargestAbsoluteWithIndex(jointschange);

                double Maxspeedactual = Math.Min(Math.Sqrt(acc[result.index] * Math.Abs(result.value)), MaxSpeedTarget[result.index]);

                double T = 0;

                if (Maxspeedactual != MaxSpeedTarget[result.index])
                {
                    T = 2 * Math.Sqrt(Math.Abs(result.value) / acc[result.index]);
                }
                else
                {
                    T = MaxSpeedTarget[result.index] / acc[result.index] + Math.Abs(result.value) / MaxSpeedTarget[result.index];
                }

                return T;
            }
        }

        public static double calculateServoPTPtime(double jointschange, TxServoGun Gun)
        {

            double MaxSpeedTarget = (Gun.DrivingJoints.Last() as TxJoint).MaxSpeed;

            double acc = (Gun.DrivingJoints.Last() as TxJoint).MaxAcceleration;

            double Maxspeedactual = Math.Min(Math.Sqrt(acc * Math.Abs(jointschange)), MaxSpeedTarget);

            double T = 0;

            if (Maxspeedactual != MaxSpeedTarget)
            {

                T = 2 * Math.Sqrt(Math.Abs(jointschange) / acc);
            }
            else
                T = MaxSpeedTarget / acc + Math.Abs(jointschange) / MaxSpeedTarget;

            Console.WriteLine("maxGunspeedactual" + Maxspeedactual.ToString());
            return T;


        }



        public double[] calculateJointsactualSpeed(Control control, List<double> jointschange, TxRobot robot, double ptpTime)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (double[])control.Invoke(new Func<Control, List<double>, TxRobot, double, double[]>(
                    calculateJointsactualSpeed), control, jointschange, robot, ptpTime);
            }
            else
            {
                // Execute the main logic of the function
                double[] MaxSpeedTarget =
                {
                    robot.Joints[0].MaxSpeed, robot.Joints[1].MaxSpeed,  robot.Joints[2].MaxSpeed,
                    robot.Joints[3].MaxSpeed, robot.Joints[4].MaxSpeed, robot.Joints[5].MaxSpeed
                 };

                double[] acc =
                {
                    robot.Joints[0].MaxAcceleration, robot.Joints[1].MaxAcceleration,  robot.Joints[2].MaxAcceleration,
                    robot.Joints[3].MaxAcceleration, robot.Joints[4].MaxAcceleration, robot.Joints[5].MaxAcceleration
                 };

                double[] JointMaxspeedactual = new double[6];
                for (int i = 0; i < 6; i++)
                {
                    JointMaxspeedactual[i] = (ptpTime * acc[i] - Math.Sqrt(ptpTime * ptpTime * acc[i] * acc[i] - 4 * acc[i] * Math.Abs(jointschange[i]))) / 2;
                }

                return JointMaxspeedactual;
            }
        }
        public static TxPoseData calCurrentRobotPosedata(Control control, joint start, joint end, TxRobot robot, double t, double ptpTime)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (TxPoseData)control.Invoke(new Func<Control, joint, joint, TxRobot, double,double, TxPoseData>(
                    calCurrentRobotPosedata), control, start, end, robot, t, ptpTime);
            }
            else
            {
                // Execute the main logic of the function
                TxPoseData txPosedataStart = new TxPoseData();

                ArrayList robJointValue = new ArrayList();

                for (int i = 0; i < start.ToArray().Length; i++)
                {
                    robJointValue.Add(start.ToArray()[i]);
                }

                txPosedataStart.JointValues = robJointValue;

                List<double> jointschange = calculateJointsChange( start, end, robot);

                //double ptpTime = calculatePTPtime(control, jointschange, robot);

                ArrayList RobJointValue = new ArrayList();

                TxPoseData robPoseData = robot.CurrentPose;

                for (int i = 0; i < 6; i++)
                {
                    TxPoseData jointposedata = calCurrentRobotAixJointData(control, ptpTime, t, i, robot, jointschange[i], (double)txPosedataStart.JointValues[i]);

                    RobJointValue.Add(jointposedata.JointValues[0]);
                }

                robPoseData.JointValues = RobJointValue;

                return robPoseData;
            }
        }
        public static TxPoseData calCurrentRobotAixJointData(Control control, double ptpTime, double t, int JointNumber, TxRobot robot, double JointchangeData, double Joint_start)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (TxPoseData)control.Invoke(new Func<Control, double, double, int, TxRobot, double, double, TxPoseData>(
                    calCurrentRobotAixJointData), control, ptpTime, t, JointNumber, robot, JointchangeData, Joint_start);
            }
            else
            {
                // Execute the main logic of the function
                TxPoseData JointPoseData = new TxPoseData();

                if (JointNumber > 5) return JointPoseData;

                double maxspeedTarget = robot.Joints[JointNumber].MaxSpeed;
                double acc = robot.Joints[JointNumber].MaxAcceleration;

                double joint_t = 0;

                double Maxspeedactual = Math.Min(Math.Sqrt(acc * Math.Abs(JointchangeData)), maxspeedTarget);

                double T = 0;

                if (Maxspeedactual != maxspeedTarget)
                {
                    T = 2 * Math.Sqrt(Math.Abs(JointchangeData) / acc);
                }
                else
                {
                    T = maxspeedTarget / acc + Math.Abs(JointchangeData) / maxspeedTarget;
                }

                if (T < ptpTime)
                {
                    Maxspeedactual = 0.5 * (ptpTime * acc - Math.Sqrt(Math.Abs(ptpTime * ptpTime * acc * acc - 4 * acc * Math.Abs(JointchangeData))));
                }

                double t_acc = Maxspeedactual / acc;

                if (t <= t_acc)
                {
                    joint_t = t * acc * t * 0.5;
                }
                else if ((t > t_acc) && (t <= ptpTime - t_acc))
                {
                    joint_t = Maxspeedactual * t_acc * 0.5 + (t - t_acc) * Maxspeedactual;
                }
                else if (t > ptpTime - t_acc)
                {
                    joint_t = Maxspeedactual * t_acc * 0.5 + (ptpTime - 2 * t_acc) * Maxspeedactual + Maxspeedactual * t_acc * 0.5 - (ptpTime - t) * acc * (ptpTime - t) * 0.5;
                }
                else
                {
                    joint_t = Maxspeedactual * t_acc * 0.5 + (ptpTime - 2 * t_acc) * Maxspeedactual + Maxspeedactual * t_acc * 0.5;
                }

                ArrayList JointValue = new ArrayList();
                JointValue.Add(joint_t * Math.Sign(JointchangeData) + Joint_start);

                JointPoseData.JointValues = JointValue;

                return JointPoseData;
                
            }
        }
        public static double calCurrentServoGunJointData(double ptpTime, double t, TxServoGun Gun, double JointchangeData, double Joint_start)
        {
            // 首先先要判断这个轴的角度变换时间是否等于ptpTime, 如果不等于，则需要计算出速度曲线公式
            //TxPoseData JointPoseData = new TxPoseData();

            double maxspeedTarget = (Gun.DrivingJoints.Last() as TxJoint).MaxSpeed;
            double acc = (Gun.DrivingJoints.Last() as TxJoint).MaxAcceleration;

            double joint_t = 0;


            double Maxspeedactual = Math.Min(Math.Sqrt(acc * Math.Abs(JointchangeData)), maxspeedTarget);

            double T = 0;

            if (Maxspeedactual != maxspeedTarget)
            {

                T = 2 * Math.Sqrt(Math.Abs(JointchangeData) / acc);
            }
            else
                T = maxspeedTarget / acc + Math.Abs(JointchangeData) / maxspeedTarget;


            //double Maxspeedactual1 = 0;
            if (T < ptpTime)
            {
                double x = ptpTime * ptpTime * acc * acc - 4 * acc * Math.Abs(JointchangeData);
                if (x < 0) { double z = 0; }
                Maxspeedactual = 0.5 * (ptpTime * acc - Math.Sqrt(Math.Abs(ptpTime * ptpTime * acc * acc - 4 * acc * Math.Abs(JointchangeData))));

            }

            double t_acc = Maxspeedactual / acc;

            if (t <= t_acc)
            {
                joint_t = t * acc * t * 0.5;


            }
            else if ((t > t_acc) && (t <= ptpTime - t_acc))

            {
                joint_t = Maxspeedactual * t_acc * 0.5 + (t - t_acc) * Maxspeedactual;

            }
            else if (t > ptpTime - t_acc)
            {

                joint_t = Maxspeedactual * t_acc * 0.5 + (ptpTime - 2 * t_acc) * Maxspeedactual + Maxspeedactual * t_acc * 0.5 - (ptpTime - t) * acc * (ptpTime - t) * 0.5;
            }
            else
            {
                joint_t = Maxspeedactual * t_acc * 0.5 + (ptpTime - 2 * t_acc) * Maxspeedactual + Maxspeedactual * t_acc * 0.5;


            }

            double gun_Open = 0;

            //ArrayList JointValue = new ArrayList();
            if (JointchangeData == 0)
            {
                gun_Open = Joint_start;
            }
            else
                gun_Open = joint_t * Math.Sign(JointchangeData) + Joint_start;






            return gun_Open;

        }

    }
}
