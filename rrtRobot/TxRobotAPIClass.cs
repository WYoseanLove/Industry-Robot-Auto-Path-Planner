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
    public class TxRobotAPIClass
    {
        public static double M_PI = 3.1415926;
        public static System.Collections.Hashtable objHash = new Hashtable();

        /*  TxRobotPostureGenerate(TxRobot robot, ArrayList Solutions, double Gun_openning)
         * Update the robot's pose based on the results of the robot inverse kinematics calculation:
         * 1.  TxRobot robot-Which robot's posture needs updating;
         * 
         * 2. Solutions-an ArrayList, stores all inverse kinematic solutions for the robot poses. 
         * The robot pose is updated using the first inverse kinematic solution. 
         * This update is to verify the collision detection results between the robot flange's installed tooling and the environment; 
         * it is unrelated to path smoothing. The optimal solution will be selected in the path optimization function.
         * 
         * 3. Gun_Opening- The Gun_Opening parameter stores the numerical values for the axis mounted on the robot flange, such as Server Weld Gun;
         * 
         * 根据机器人逆向学运算的结果,更新机器人的姿态:
         * 1. robot指的是哪个机器人的姿态需要更新;
         * 2. Solutions 作为ArrayList 数据,存储了所有的机器人姿态的逆向解,
         * 这里机器人姿态的更新是按照第一个逆向解进行姿态更新, 
         * 这里更新机器人姿态的目的是为了验证机器人法兰盘安装Tooling的碰撞检测结果, 
         * 与路径的平滑无关,在路径的优化函数中,会挑选出最优解;
         * 3. Gun_Opening 参数存储了法兰盘安装轴的数值例如焊钳
         */
        public static void TxRobotPostureGenerate(Control control, TxRobot robot, TxServoGun servoGun, ArrayList Solutions, double Gun_openning)
        {

            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                control.Invoke(new Action<Control, TxRobot, TxServoGun, ArrayList, double>(TxRobotPostureGenerate), control, robot, servoGun, Solutions, Gun_openning);
            }
            else
            {
                // Execute the main logic of setting the robot and tool poses
                if (Solutions.Count == 0)
                    return;

                TxPoseData poseData = (TxPoseData)Solutions[0];
                robot.CurrentPose = poseData;

                ITxDevice tSelectObject = servoGun as ITxDevice;

                ArrayList ToolJointValue = new ArrayList();
                ToolJointValue.Add(Gun_openning);


                TxPoseData txToolPoseData = tSelectObject.CurrentPose;
                txToolPoseData.JointValues = ToolJointValue;
                tSelectObject.CurrentPose = txToolPoseData;

            }


        }
        /* public static ArrayList robotInverseCal(TxRobot robot, point target)
         * Robot inverse kinematics calculations are performed using the built-in functions of TECNOMATIX. 
         * Other inverse kinematics functions (custom-defined) can also be substituted.
         * 通过TECNOMATIX自带的函数进行机器人逆向学计算，这里也可以更换成其他的逆向学函数(自定义的)
         */

        public static ArrayList robotInverseCal(Control control, TxRobot robot, point target)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (ArrayList)control.Invoke(new Func<Control, TxRobot, point, ArrayList>(robotInverseCal), control, robot, target);
            }
            else
            {
                // The main logic of the method
                TxRobotInverseData txRobotInverseData = new TxRobotInverseData();
                TxTransformation robotTarget = new TxTransformation();
                TxVector rotation = new TxVector();
                rotation.X = target.rx;
                rotation.Y = target.ry;
                rotation.Z = target.rz;

                TxTransformation txTransformation = new TxTransformation();
                txTransformation.RotationRPY_XYZ = rotation;
                txTransformation.Translation = new TxVector(target.x, target.y, target.z);

                txRobotInverseData.Destination = txTransformation;

                ArrayList solutions = robot.CalcInverseSolutions(txRobotInverseData);

                return solutions;
            }
        }
        /*  
         * Implement collision detection using TECNOMATIX's internal functions. Call TxCollisionQueryResults via a using statement to guarantee proper disposal of resources.
         * 通过TECNOMATIX自带的函数进行碰撞检测，通过using 调用TxCollisionQueryResults，并及时释放资源
         */

        public static bool Collision_Check(Control control, TxCollisionPairCreationData cd, TxCollisionQueryParams queryParams, TxCollisionRoot root, TxObjectList collisionSrc, TxObjectList collisionTar, double Clearance)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (bool)control.Invoke(new Func<Control, TxCollisionPairCreationData, TxCollisionQueryParams, TxCollisionRoot, TxObjectList, TxObjectList, double, bool>(
                    Collision_Check), control, cd, queryParams, root, collisionSrc, collisionTar, Clearance);
            }
            else
            {
                // Execute the main logic of the function
                queryParams.Mode = TxCollisionQueryParams.TxCollisionQueryMode.DefinedPairs;
                queryParams.NearMissDistance = Clearance;

                using (TxCollisionQueryResults results = root.GetCollidingObjects(queryParams))
                {
                    if (results.States.Count == 0)
                    {
                        results.States.Clear();
                        cd.Dispose();
                        return true;
                    }

                    for (int i = 0; i < results.States.Count; i++)
                    {
                        if ((results.States[i] as TxCollisionState).Type == TxCollisionState.TxCollisionStateType.Collision)
                        {
                            results.States.Clear();
                            cd.Dispose();
                            return false;
                        }
                    }
                    return true;

                }

            }
        }
        // 输出碰撞点的坐标；
        public static TxVector Collision_CheckPoint(Control control, TxCollisionPairCreationData cd, TxCollisionQueryParams queryParams, TxCollisionRoot root, TxObjectList collisionSrc, TxObjectList collisionTar, double Clearance)
        {
            if (control.InvokeRequired)
            {
                // Use Invoke to call this method on the UI thread
                return (TxVector)control.Invoke(new Func<Control, TxCollisionPairCreationData, TxCollisionQueryParams, TxCollisionRoot, TxObjectList, TxObjectList, double, TxVector>(
                    Collision_CheckPoint), control, cd, queryParams, root, collisionSrc, collisionTar, Clearance);
            }
            else
            {
                queryParams.Mode = TxCollisionQueryParams.TxCollisionQueryMode.DefinedPairs;
                queryParams.NearMissDistance = Clearance;

                using (TxCollisionQueryResults results = root.GetCollidingObjects(queryParams))
                {
                    if (results.States.Count == 0)
                    {
                        results.States.Clear();
                        cd.Dispose();
                        return new TxVector(0, 0, 0);
                    }

                    for (int i = 0; i < results.States.Count; i++)
                    {
                        if ((results.States[i] as TxCollisionState).Type == TxCollisionState.TxCollisionStateType.Collision)
                        {
                            //results.States.Clear();
                            // cd.Dispose();
                            double clearance = 0;
                            TxVector pointOnTool = new TxVector();
                            TxVector pointOnCollison = new TxVector();

                            ((results.States[i] as TxCollisionState).FirstObject as ITxLocatableObject).GetMinimalDistance(

                                ((results.States[i] as TxCollisionState).SecondObject as ITxLocatableObject)
                                , out clearance, out pointOnTool, out pointOnCollison

                                );
                            return pointOnTool;
                        }
                    }
                    return new TxVector(0, 0, 0);

                }



            }

        }

        public static void DisposeTxposureData(ArrayList Solutions)
        {
            if (Solutions.Count == 0) return;

            for (int i = 0; i < Solutions.Count; i++)
            {

                (Solutions[i] as TxPoseData).Dispose();
            }

        }

        public static void CreateSweptVolume(TxWeldOperation weldOp)
        {
            objHash.Clear();

            TxSimulationPlayer player = new TxSimulationPlayer();
            player.SetOperation(weldOp);
            player.TimeIntervalReached += player_TimeIntervalReached;
            player.PlayWithoutRefresh();
            player.TimeIntervalReached -= player_TimeIntervalReached;
            player.Rewind();


            TxRobot rob = weldOp.Robot as TxRobot;
            TxSweptVolumeCreationData svCredata = new TxSweptVolumeCreationData();
            TxSweptVolume sv = weldOp.CreateSweptVolume(svCredata);
            sv.Accuracy = 50;
            sv.Clearance = 5.0;
            sv.Name = rob.Name;

            TxSweptVolumeCalculationData data = new TxSweptVolumeCalculationData();

            foreach (ITxLocatableObject locObj in objHash.Keys)
            {
                data.AddObjectData(objHash[locObj] as TxObjectSweptVolumeCalculationData);
            }
            TxSweptVolume_MayProceedCalculation MayProceed = calculator;

            bool calculator(int x)
            {
                return true;
            }

            sv.Calculate(data, MayProceed);
            sv.Lock();
            void player_TimeIntervalReached(object sender, TxSimulationPlayer_TimeIntervalReachedEventArgs args)
            {
                //Add also the attached objects if you need (the gun for example)            
                TxRobot robot = weldOp.Robot as TxRobot;
                TxObjectList children = robot.GetAllDescendants(new TxTypeFilter(typeof(ITxLocatableObject)));
                children.Add(weldOp.Gun);
                foreach (ITxLocatableObject child in children)
                {
                    if (!objHash.ContainsKey(child))
                    {
                        //if no data was added for that object (child) create new data
                        TxObjectSweptVolumeCalculationData osData = new TxObjectSweptVolumeCalculationData(child);
                        osData.AddTransformation(child.AbsoluteLocation);
                        objHash.Add(child, osData);
                    }
                    else
                    {
                        //Get the data and add a new transformation
                        TxObjectSweptVolumeCalculationData osData = objHash[child] as TxObjectSweptVolumeCalculationData;
                        osData.AddTransformation(child.AbsoluteLocation);
                    }
                }
            }




        }


        public static T GetObjectByName<T>(string objectName) where T : class, ITxObject
        {
            TxObjectList foundObjects = TxApplication.ActiveDocument.GetObjectsByName(objectName);

            T foundObj = null;
            int objectsNum = 0;

            foreach (ITxObject obj in foundObjects)
            {
                if (obj is T)
                {
                    foundObj = obj as T;
                    objectsNum++;
                }
            }

            return foundObj;
        }

        public static TxComponent CreateResourcePathCurve(int n, string Name)
        {
            TxLocalComponentCreationData creationData = new TxLocalComponentCreationData(Name + n.ToString());
            TxComponent x = TxApplication.ActiveDocument.PhysicalRoot.CreateLocalComponent(creationData);

            return x;
        }
        public static TxComponent CreateResourceobbs(int n, string Name)
        {
            TxLocalComponentCreationData creationData = new TxLocalComponentCreationData(Name + n.ToString());
            TxComponent comp = TxApplication.ActiveDocument.PhysicalRoot.CreateLocalComponent(creationData);


            return comp;
        }
        public static void CreateSphereView(Control control, joint q, TxRobot robot, TxComponent comp, TxColor color)
        {
            try
            {


                var arr = new ArrayList(q.ToArray());
                var robotPosture = new TxPoseData();
                robotPosture.JointValues = arr;

                var sols = new ArrayList { robotPosture };
                TxRobotAPIClass.TxRobotPostureGenerate(
                    control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, q.Sever_Gun);

                point start = new point(

                        robot.TCPF.AbsoluteLocation.Translation.X,
                        robot.TCPF.AbsoluteLocation.Translation.Y,
                        robot.TCPF.AbsoluteLocation.Translation.Z,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                      q.Sever_Gun
                   );
                TxSphereCreationData data = new TxSphereCreationData();
                data.Name = "obs";
                data.AbsoluteLocation = new TxTransformation(1, 0, 0, start.x, 0, 1, 0, start.y, 0, 0, 1, start.z);

                data.Radius = 6;

                data.SetAsDisplay();

                comp.CreateSolidSphere(data);
                comp.Color = color;

            }
            catch (Exception)
            {
                return;

            }

        }
        public static void TxcreateCurvePath(Control control, TxComponent x, joint START, joint END, string Index, TxRobot robot, TxColor color)
        {
            try
            {
                var arr = new ArrayList(START.ToArray());
                var robotPosture = new TxPoseData();
                robotPosture.JointValues = arr;

                var sols = new ArrayList { robotPosture };
                TxRobotAPIClass.TxRobotPostureGenerate(
                    control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, START.Sever_Gun);

                point start = new point(

                        robot.TCPF.AbsoluteLocation.Translation.X,
                        robot.TCPF.AbsoluteLocation.Translation.Y,
                        robot.TCPF.AbsoluteLocation.Translation.Z,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                      START.Sever_Gun
                   );



                arr = new ArrayList(END.ToArray());
                robotPosture = new TxPoseData();
                robotPosture.JointValues = arr;

                sols = new ArrayList { robotPosture };
                TxRobotAPIClass.TxRobotPostureGenerate(
                    control, TxrrtRobotPathPlannerForm.robot, TxrrtRobotPathPlannerForm.robServerGun, sols, END.Sever_Gun);

                point end = new point(

                        robot.TCPF.AbsoluteLocation.Translation.X,
                        robot.TCPF.AbsoluteLocation.Translation.Y,
                        robot.TCPF.AbsoluteLocation.Translation.Z,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                       robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                      END.Sever_Gun
                   );

                if (x == null) return;
                TxSphereCreationData data = new TxSphereCreationData();
                data.Name = "start" + Index;
                data.AbsoluteLocation = new TxTransformation(1, 0, 0, start.x, 0, 1, 0, start.y, 0, 0, 1, start.z);

                data.Radius = 1;

                data.SetAsDisplay();

                x.CreateSolidSphere(data);

                data = new TxSphereCreationData();
                data.Name = "end" + Index;
                data.AbsoluteLocation = new TxTransformation(1, 0, 0, end.x, 0, 1, 0, end.y, 0, 0, 1, end.z);

                data.Radius = 1;
                data.SetAsDisplay();

                x.CreateSolidSphere(data);


                TxLineCreationData dataline = new TxLineCreationData();

                dataline.StartPoint = new TxVector(start.x, start.y, start.z);
                dataline.EndPoint = new TxVector(end.x, end.y, end.z);

                dataline.SetAsDisplay();
                x.CreateLine(dataline);
                x.Color = color;


            }
            catch (Exception)
            {
                return;

            }

        }


    }
}