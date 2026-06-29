using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using static System.Windows.Forms.AxHost;

namespace Try_teccommand
{
    public class TxRobotptpInterpolationCal
    {
        [DllImport("kernel32.dll", SetLastError = true)][return: MarshalAs(UnmanagedType.Bool)] private static extern bool AllocConsole();


        public void InitializeConsole()
        {
            AllocConsole();
        }
        private static double RadiansToDegrees(string radians)
        {
            return double.Parse(radians) * (180.0 / Math.PI);
        }

        public struct point
        {
            public double x { get; set; }
            public double y { get; set; }
            public double z { get; set; }
            public double rx { get; set; }
            public double ry { get; set; }
            public double rz { get; set; }
            public double open { get; set; }


            public point(double X, double Y, double Z, double RX, double RY, double RZ, double OPEN)
            {
                x = X;
                y = Y;
                z = Z;

                rx = RX;
                ry = RY;
                rz = RZ;
                open = OPEN;

            }

        };
        public TxRobotptpInterpolationCal()
        {

            ITxObject selectItem = TxApplication.ActiveSelection.GetLastPickedItem() as ITxObject;
            if (selectItem == null || selectItem.GetType().Name != "TxWeldOperation")
            {
                //TxMessageBox.Show("Please select the Weld OP firstly before connect!", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            TxOperationRoot txOperationRoot = TxApplication.ActiveDocument.OperationRoot;
            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxRoboticViaLocationOperation));
            TxWeldOperation weldTargetOperation = selectItem as TxWeldOperation;
            TxObjectList allLocPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if (allLocPointsExist.Count < 2)
            {
                //TxMessageBox.Show("No Weld Points founded under the Robot Welding Operation !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;

            }
            TxRobot robot = weldTargetOperation.Robot as TxRobot;

            InitializeConsole();
            OutputRobotType(robot);

            TxPoseData startPose = calculateStartPointPosture(robot, allLocPointsExist[0] as TxRoboticViaLocationOperation);
            TxPoseData endPose = calculateStartPointPosture(robot, allLocPointsExist[1] as TxRoboticViaLocationOperation);

            Console.WriteLine("start Pose :" + RadiansToDegrees(startPose.JointValues[0].ToString())
                               + " " + RadiansToDegrees(startPose.JointValues[1].ToString())
                               + " " + RadiansToDegrees(startPose.JointValues[2].ToString())
                               + " " + RadiansToDegrees(startPose.JointValues[3].ToString())
                               + " " + RadiansToDegrees(startPose.JointValues[4].ToString())
                               + " " + RadiansToDegrees(startPose.JointValues[5].ToString())
                               );

            Console.WriteLine("end Pose :" + RadiansToDegrees(endPose.JointValues[0].ToString())
                              + " " + RadiansToDegrees(endPose.JointValues[1].ToString())
                              + " " + RadiansToDegrees(endPose.JointValues[2].ToString())
                              + " " + RadiansToDegrees(endPose.JointValues[3].ToString())
                              + " " + RadiansToDegrees(endPose.JointValues[4].ToString())
                              + " " + RadiansToDegrees(endPose.JointValues[5].ToString())
                              );


            List<double> jointschange = new List<double>();

            jointschange = calculateJointsChange(allLocPointsExist[0] as TxRoboticViaLocationOperation, allLocPointsExist[1] as TxRoboticViaLocationOperation, robot);

            for (int i = 0; i < jointschange.Count; i++)
            {
                Console.WriteLine("Jointschange :" + RadiansToDegrees(jointschange[i].ToString())

                              );
            }
            TxServoGun Gun = weldTargetOperation.Gun as TxServoGun;

            for (int i = 0; i < Gun.DrivingJoints.Count; i++)
            {

                Console.WriteLine("ServerGun Max speed :" + (Gun.DrivingJoints[i] as TxJoint).MaxSpeed);
            }
            double servoGunjointChange = (allLocPointsExist[1] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue -
                (allLocPointsExist[0] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue;

            for (int i = 0; i < robot.Joints.Count; i++)
            {
                Console.WriteLine("JointsMaxspeed :" + RadiansToDegrees(robot.Joints[i].MaxSpeed.ToString())

                              );
            }

            for (int i = 0; i < robot.Joints.Count; i++)
            {
                Console.WriteLine("JointsMaxAcc J" + (i + 1) + " :" + RadiansToDegrees(robot.Joints[i].MaxAcceleration.ToString()));
            }

            double ptpTime = calculatePTPtime(jointschange, robot);
            double ServoGunTime = calculateServoPTPtime(servoGunjointChange, Gun);
            Console.WriteLine("ptp Time" + ptpTime);
            Console.WriteLine("ServoGunTime Time" + ServoGunTime);
            if (ptpTime <= ServoGunTime)
                ptpTime = ServoGunTime;

            double[] jointspeed = calculateJointsactualSpeed(jointschange, robot, ptpTime);

            for (int i = 0; i < jointspeed.Count(); i++)
            {

                Console.WriteLine("Aix speed " + RadiansToDegrees(jointspeed[i].ToString()));

            }

            OutputAxisPhaseTimes(jointschange, robot, ptpTime);


            (double value, int index) result = FindLargestAbsoluteWithIndex(jointschange);

            int div = (int)(ptpTime / 0.01);


            double step = 0.01;

            for (int i = 0; i < div; i++)
            {
                TxPoseData robotJoint = calCurrentRobotPosedata(allLocPointsExist[0] as TxRoboticViaLocationOperation, allLocPointsExist[1] as TxRoboticViaLocationOperation,
                    robot, step * (i + 1), ptpTime);
                double severGundata = calCurrentServoGunJointData(ptpTime, step * (i + 1), Gun, servoGunjointChange, (allLocPointsExist[0] as TxRoboticViaLocationOperation).RobotExternalAxesData[0].JointValue);

                robot.CurrentPose = robotJoint;

                point p = new point(robot.TCPF.AbsoluteLocation.Translation.X,
                    robot.TCPF.AbsoluteLocation.Translation.Y,
                    robot.TCPF.AbsoluteLocation.Translation.Z,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                    severGundata);

                TxRoboticViaLocationOperation RobFramepostLocation = addrobotPathViaLoc(i.ToString(), p, weldTargetOperation, robot);
                weldTargetOperation.MoveChildAfter((TxRoboticViaLocationOperation)allLocPointsExist[1], RobFramepostLocation);

            }

        }

        public static TxPoseData calculateStartPointPosture(TxRobot robot, TxRoboticViaLocationOperation Loc)
        {
            TxPoseData txPoseData = robot.GetPoseAtLocation(Loc as ITxRoboticLocationOperation);

            return txPoseData;


        }


        public static List<double> calculateJointsChange(TxRoboticViaLocationOperation start, TxRoboticViaLocationOperation end, TxRobot robot)
        {

            TxPoseData startPose = calculateStartPointPosture(robot, start);

            TxPoseData endPose = calculateStartPointPosture(robot, end);


            List<double> jointschange = new List<double>();

            for (int i = 0; i < 6; i++)
            {

                jointschange.Add((double)(endPose).JointValues[i] -
                    (double)(startPose).JointValues[i]);
            }

            return jointschange;
        }
        private static (double value, int index) FindLargestAbsoluteWithIndex(List<double> numbers)
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

        private static bool HasSameMaximumSpeed(TxRobot robot)
        {
            double firstAxisMaximumSpeed = robot.Joints[0].MaxSpeed;
            for (int i = 1; i < 6; i++)
            {
                if (robot.Joints[i].MaxSpeed != firstAxisMaximumSpeed)
                {
                    return false;
                }
            }

            return true;
        }

        private static int FindLowestMaximumSpeedAxis(TxRobot robot)
        {
            int lowestMaximumSpeedAxisIndex = 0;
            for (int i = 1; i < 6; i++)
            {
                if (robot.Joints[i].MaxSpeed < robot.Joints[lowestMaximumSpeedAxisIndex].MaxSpeed)
                {
                    lowestMaximumSpeedAxisIndex = i;
                }
            }

            return lowestMaximumSpeedAxisIndex;
        }

        private static void OutputRobotType(TxRobot robot)
        {
            if (HasSameMaximumSpeed(robot))
            {
                Console.WriteLine("Robot type: same maximum speed for J1-J6");
                return;
            }

            int lowestMaximumSpeedAxisIndex = FindLowestMaximumSpeedAxis(robot);
            Console.WriteLine("Robot type: different maximum speeds for J1-J6; lowest maximum speed axis: J" + (lowestMaximumSpeedAxisIndex + 1));
        }


        public static double calculatePTPtime(List<double> jointschange, TxRobot robot)
        {
            double[] MaxSpeedTarget =
            {
            robot.Joints[0].MaxSpeed, robot.Joints[1].MaxSpeed,  robot.Joints[2].MaxSpeed,robot.Joints[3].MaxSpeed,robot.Joints[4].MaxSpeed,robot.Joints[5].MaxSpeed
        };

            double[] acc =
            {
            robot.Joints[0].MaxAcceleration, robot.Joints[1].MaxAcceleration,  robot.Joints[2].MaxAcceleration,robot.Joints[3].MaxAcceleration,robot.Joints[4].MaxAcceleration,robot.Joints[5].MaxAcceleration
        };

            double T = 0;
            int bottleneckAxisIndex = 0;

            for (int i = 0; i < 6; i++)
            {
                double axisTime = calculateAxisMinimumTime(jointschange[i], acc[i], MaxSpeedTarget[i], out bool isTrapezoid);

                Console.WriteLine("J" + (i + 1) + " profile: " + (isTrapezoid ? "trapezoid" : "triangle") + ", min time: " + axisTime);

                if (axisTime > T)
                {
                    T = axisTime;
                    bottleneckAxisIndex = i;
                }
            }

            Console.WriteLine("Bottleneck axis: J" + (bottleneckAxisIndex + 1) + ", ptp time: " + T);
            return T;

        }

        private static double calculateAxisMinimumTime(double jointChange, double acc, double maxSpeed, out bool isTrapezoid)
        {
            double s = Math.Abs(jointChange);
            isTrapezoid = false;

            if (s <= 0 || acc <= 0 || maxSpeed <= 0)
            {
                return 0;
            }

            if (s >= maxSpeed * maxSpeed / acc)
            {
                isTrapezoid = true;
                return maxSpeed / acc + s / maxSpeed;
            }

            return 2 * Math.Sqrt(s / acc);
        }

        private static void OutputAxisPhaseTimes(List<double> jointschange, TxRobot robot, double ptpTime)
        {
            for (int i = 0; i < 6; i++)
            {
                double s = Math.Abs(jointschange[i]);
                double acc = robot.Joints[i].MaxAcceleration;
                double vmax = robot.Joints[i].MaxSpeed;
                double tAcc = 0;
                double tCruise = 0;

                if (s > 0 && acc > 0 && vmax > 0)
                {
                    double ownTime = calculateAxisMinimumTime(jointschange[i], acc, vmax, out bool isTrapezoid);
                    double vPeak = isTrapezoid ? vmax : Math.Sqrt(acc * s);

                    if (ownTime < ptpTime)
                    {
                        vPeak = 0.5 * (ptpTime * acc - Math.Sqrt(Math.Max(0, ptpTime * ptpTime * acc * acc - 4 * acc * s)));
                    }

                    tAcc = vPeak / acc;
                    tCruise = Math.Max(0, ptpTime - 2 * tAcc);
                }

                Console.WriteLine("J" + (i + 1) + " accel time: " + tAcc.ToString("F4")
                    + " s, cruise time: " + tCruise.ToString("F4")
                    + " s, decel time: " + tAcc.ToString("F4") + " s");
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
        public double[] calculateJointsactualSpeed(List<double> jointschange, TxRobot robot, double ptpTime)
        {

            double[] MaxSpeedTarget =
            {
            robot.Joints[0].MaxSpeed, robot.Joints[1].MaxSpeed,  robot.Joints[2].MaxSpeed,robot.Joints[3].MaxSpeed,robot.Joints[4].MaxSpeed,robot.Joints[5].MaxSpeed
        };

            double[] acc =
            {
            robot.Joints[0].MaxAcceleration, robot.Joints[1].MaxAcceleration,  robot.Joints[2].MaxAcceleration,robot.Joints[3].MaxAcceleration,robot.Joints[4].MaxAcceleration,robot.Joints[5].MaxAcceleration
        };
            double[] JointMaxspeedactual = new double[6];
            for (int i = 0; i < 6; i++)
            {
                double x = ptpTime * ptpTime * acc[i] * acc[i] - 4 * acc[i] * Math.Abs(jointschange[i]);
                if (x < 0)
                {
                    int z = 0;

                }
                JointMaxspeedactual[i] = (ptpTime * acc[i] - Math.Sqrt(Math.Abs(ptpTime * ptpTime * acc[i] * acc[i] - 4 * acc[i] * Math.Abs(jointschange[i])))) / 2;

            }

            return JointMaxspeedactual;

        }
        public static TxPoseData calCurrentRobotPosedata(TxRoboticViaLocationOperation start, TxRoboticViaLocationOperation end, TxRobot robot, double t, double ptpTime)
        {
            TxPoseData txPosedataStart = calculateStartPointPosture(robot, start);

            List<double> jointschange = calculateJointsChange(start, end, robot);

            //double ptpTime = calculatePTPtime(jointschange, robot);

            ArrayList RobJointValue = new ArrayList();

            TxPoseData robPoseData = robot.CurrentPose;

            for (int i = 0; i < 6; i++)
            {

                TxPoseData jointposedata = calCurrentRobotAixJointData(ptpTime, t, i, robot, jointschange[i], (double)txPosedataStart.JointValues[i]);

                RobJointValue.Add(jointposedata.JointValues[0]);

            }

            robPoseData.JointValues = RobJointValue;




            return robPoseData;


        }

        public static TxPoseData calCurrentRobotAixJointData(double ptpTime, double t, int JointNumber, TxRobot robot, double JointchangeData, double Joint_start)
        {
            // 首先先要判断这个轴的角度变换时间是否等于ptpTime, 如果不等于，则需要计算出速度曲线公式
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
                T = maxspeedTarget / acc + Math.Abs(JointchangeData) / maxspeedTarget;


            double Maxspeedactual1 = 0;
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

            ArrayList JointValue = new ArrayList();
            if (JointchangeData == 0)
            {
                JointValue.Add(Joint_start);
            }
            else
                JointValue.Add(joint_t * Math.Sign(JointchangeData) + Joint_start);



            JointPoseData.JointValues = JointValue;


            return JointPoseData;

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

        public static TxRoboticViaLocationOperation addrobotPathViaLoc(string name, point p, TxWeldOperation weldOperation, TxRobot robot)
        {
            TxRoboticViaLocationOperationCreationData roboticViaLocationOperationCreationData = new TxRoboticViaLocationOperationCreationData(name, "", 2.0);

            TxVector tran = new TxVector(p.x, p.y, p.z);

            TxVector rot = new TxVector(p.rx, p.ry, p.rz);

            roboticViaLocationOperationCreationData.AbsoluteLocation = new TxTransformation(tran, rot, TxTransformation.TxRotationType.RPY_XYZ);

            TxRoboticViaLocationOperation RobFramepostLocation = weldOperation.CreateRoboticViaLocationOperation(roboticViaLocationOperationCreationData);
            TxRobotExternalAxisData[] externalAxisData = new TxRobotExternalAxisData[1];
            externalAxisData[0] = new TxRobotExternalAxisData();
            externalAxisData[0].Device = weldOperation.Gun as TxServoGun;
            externalAxisData[0].Joint = (weldOperation.Gun as TxServoGun).DrivingJoints.Last() as TxJoint;
            externalAxisData[0].JointValue = p.open;

            RobFramepostLocation.RobotExternalAxesData = externalAxisData;
            RobFramepostLocation.SetParameter(new TxRoboticIntParam("RRS_MOTION_TYPE", 1));


            return RobFramepostLocation;
        }

    }
}