using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Ui;
using static System.Net.Mime.MediaTypeNames;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;



namespace rrtRobot
{
    /* rrtRobot dll文件是基于SIMENSE Tecnomatix Process simulate 开发的一款用于工业机器人路径自动仿真的插件；
     * 生成的dll需要参照参考文件"如何激活dll在tecnomatix仿真软件功能"的参考文件进行安装和调试；
     * dll代码中使用的是基于Tecnomatix APi函数进行的开发，主要用到其碰撞检测，机器人的正逆向运动学运算，机器人posture 的改变及路径点的自动化生成；
     * 类TxRobotKinematix中包含了以上常用的函数供系统调用；
     * 类TxRobotRRTConnect类中包含了常用的路径生成算法RRT* CONNECT, 代码的可靠性已经在VTK三维中进行了验证，具体可以参考如下Github链接：
     * https://github.com/WYoseanLove/RRT-_Connect_3D
     * 系统在计算过程中的log文件保存在Documents/rrtRobot里面，也可以自己修改log 函数生成自身需要的参数信息；
     * 本程序目前只试用于机器人伺服焊钳的点焊轨迹的生成；
     * 软件界面首先需要选取需要自动生成轨迹的机器人，具体代码在函数void m_txRobotName_Picked()中定义，需要保证选取机器人安装的工具正是生成轨迹的Weld Gun;
     * 软件界面再次选取的是碰撞检测的内容，这里需要将所有用于碰撞检测的数据均添加其中；
     * 完成以上任务之后，所有的配置已经完成；
     * 选中需要计算的机器人轨迹(只能包括焊点，其他的类型点不参加计算）, 点击path connect,计算完成后，点击生成轨迹即可；
     * 
     * The rrtRobot.dll file is a plugin for industrial robot path simulation, developed using Siemens Tecnomatix Process Simulate.
     * Installation: 
     * The generated DLL requires installation and debugging according to the instructions in the reference document "How to activate the DLL in Tecnomatix simulation software."
     * Functionality: 
     * The DLL utilizes the Tecnomatix API, primarily for collision detection, robot forward and inverse kinematics, robot posture manipulation, and automated path point generation.
     * Key Class: TxRobotKinematix: This class contains commonly used functions for robot kinematics, accessible by the system.
     * Key Class: TxRobotRRTConnect: This class implements the RRT*-Connect path planning algorithm. The code's reliability has been verified using VTK 3D visualization (see GitHub link).
     * https://github.com/WYoseanLove/RRT-_Connect_3D
     * Logging: 
     * Log files are saved in Documents/rrtRobot/. The log function can be modified to output custom parameters
     * Application: 
     * This program is currently designed for generating welding trajectories for robot-mounted servo welding guns (spot welding).
     * Robot Selection (m_txRobotName_Picked()):
     * The software interface requires selecting the robot for which to generate the trajectory. Ensure the selected robot has the correct welding gun attached
     * Collision Detection Data: 
     * The interface also requires selecting all data necessary for collision detection
     * Workflow: 
     * After configuring the robot, collision data, then select the welding points (only welding points are considered; 
     * other point types are ignored), click "path connect," and after the calculation is complete, click "generate trajectory."
     */

    public partial class TxrrtRobotPathPlannerForm : TxForm
    {

        public static double M_PI = 3.1415926;
        public static Control mainTxControl;
        public static TxTransformation TCPLocation; //记录所选机器人的TCP Frame 值，从robot 的Tool frame 到TCP frame 的转换矩阵，用于正向运动学计算
        public static TxTransformation ToolFrameLocation; //记录所选机器人的Tool Frame 值，从robot 的Tcp frame 到Tool frame 的转换矩阵,用于逆运动学计算
        public static TxFrame baseFrame; //记录所选机器人的Base Frame 值;
        public static TxObjectList collisionTar;
        public static TxCollisionPairCreationData cd;
        public static TxCollisionPair cp;
        public static TxCollisionQueryParams queryParams;
        public static TxObjectList collisionSrc;
        public static TxCollisionRoot root;
        public static TxRobot robot;
        public static List<TxWeldOperation> TargetCalWeldOp = new List<TxWeldOperation>();
        public static TxServoGun robServerGun;
        private int progressbarNumber = 0;
        private int progressbarCount = 0;
        public TxWeldOperation weldTargetOperation;
        public static double ToolJointOpening; // 存放的是焊钳的open的尺寸
        public static bool rrtconnectCal_ongoing = false;
        public List<Tuple<Tuple<Node3D, string>, TxPoseData>> node3D_startptpList;
        public List<Tuple<Tuple<Node3D, string>, TxPoseData>> node3D_endptpList;
        public static List<List<joint>> fullpath;
        public static List<string> TargetCalLocNames = new List<string>();
        public static List<joint> Pathend_nodes = new List<joint>();

        public static bool isRCSLoaded = false;


        // 用于log文件txt的生成，在系统Documents/rrtRobot文件夹下面
        // 获取当前用户的Documents路径
        public static string LogfilePath;

        // 定义子文件夹名称和文件名

        public char spotagainstCollisionSrc;
        public TxrrtRobotPathPlannerForm()
        {
            GenerateLogfile("dataLog.txt");

        }
        public bool checkLicense()
        {
            // 获取当前时间
            DateTime currentTime = DateTime.Now;

            // 设定目标时间为2025年3月31日下午4点
            DateTime targetTime = new DateTime(2027, 6, 30, 8, 0, 0);

            // 比较当前时间与设定时间
            if (currentTime > targetTime)
            {
                // 当前时间超过设定时间
                return false;
            }

            return true;
        }

        public void GenerateLogfile(string fileName)
        {
            // 组合完整的子文件夹路径和文件路径
            string documentsPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            string subFolderName = "rrtRobot";
            //string fileName = "dataLog.txt";

            string subFolderPath = Path.Combine(documentsPath, subFolderName);
            LogfilePath = Path.Combine(subFolderPath, fileName);

            // 如果子文件夹不存在，则创建它
            if (!Directory.Exists(subFolderPath))
            {
                Directory.CreateDirectory(subFolderPath);
            }
            // 检查文件是否存在，如果不存在则创建并写入文件
            if (!File.Exists(LogfilePath))
            {
                File.Create(LogfilePath).Dispose();

            }

        }
        public void Form_Setup()
        {
            //对对话框控件的字体进行修改，在WPF中修改的字体无法在Tecnomatix中显示；
            m_pathGenerate.Enabled = false;
            
            m_pathGenerate.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            button1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));

            m_spotDirec.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            Collision_Src.Font = new System.Drawing.Font("Microsoft Sans Serif", 12);
           // this.FormBorderStyle = FormBorderStyle.Fixed3D;
            this.StartPosition = FormStartPosition.CenterParent;
            m_collisionListPick.Enabled = false;
            button1.Enabled = false;
            collisionSrc = new TxObjectList();
        }

        private void txTargetGroupOpEditBoxCtrl_Picked(object sender, TxObjEditBoxCtrl_PickedEventArgs args)
        {

            ITxObject txTargetGroupOp = m_GroupTargetOpPick.Object as ITxObject;
            if (txTargetGroupOp == null) return;
            textLogfile.Clear();
            // 递归方法，输入一个 ITxObject，如果是 TxWeldOperation，则添加到目标列表
            // 如果是 TxCompoundOperation，递归遍历它所有子元素

            // 判断是否是合法的类型 (TxCompoundOperation 或 TxWeldOperation)，否则报错提示
            if (!(txTargetGroupOp is TxCompoundOperation) && !(txTargetGroupOp is TxWeldOperation))
            {
                //TxMessageBox.Show("Please Select Target Calculation Operation", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Please Select Target Calculation Operation!" + Environment.NewLine);

                textLogfile.SelectionStart = textLogfile.TextLength;
                textLogfile.ScrollToCaret();

                return;
            }

            // 递归收集所有 TxWeldOperation
            CollectWeldOperations(txTargetGroupOp, TargetCalWeldOp);

            if (TargetCalWeldOp.Count == 0)
            {
                //TxMessageBox.Show("No Validated Weld Group selected", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " No Validated Weld Group selected!" + Environment.NewLine);

                textLogfile.SelectionStart = textLogfile.TextLength;
                textLogfile.ScrollToCaret();

                m_GroupTargetOpPick.Text = string.Empty;
                return;
            }
            else
            {
                m_GroupTargetOpPick.LoseFocus();

                for (int i = 0; i < TargetCalWeldOp.Count; i++)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " The " + (i + 1).ToString() + "th target calculate weld op is " +
                        TargetCalWeldOp[i].Name + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();

                }


                m_collisionListPick.Enabled = true;
                button1.Enabled = true;

            }

            // 这里 TargetCalWeldOp 就包含了所有递归搜到的 TxWeldOperation，可继续后续处理


        }

        private void CollectWeldOperations(ITxObject op, List<TxWeldOperation> targetList)
        {
            // 类型名字更安全可以用is判断，或者Type做判断
            if (op is TxWeldOperation weldOp)
            {
                targetList.Add(weldOp);
            }
            else if (op is TxCompoundOperation compoundOp)
            {
                for (int i = 0; i < compoundOp.Count; i++)
                {
                    ITxObject child = compoundOp.GetChildAt(i);
                    CollectWeldOperations(child, targetList);
                }
            }
            // 其他类型不用管，直接跳过
        }

        private void m_collisionListPicked(object sender, TxObjComboBoxCtrl_PickedEventArgs args)
        {
            TxObjectBase txObjects = m_collisionListPick.Object as TxObjectBase;
            if (txObjects == null)
            {
                m_collisionListPick.LoseFocus();
                if (collisionSrc.Count != 0)
                {
                    m_collisionListPick.SelectObject(collisionSrc[collisionSrc.Count - 1]);
                }
                return;
            }



            collisionSrc.Add(m_collisionListPick.Object);

            m_collisionListPick.AddItem(m_collisionListPick.Object.Name, m_collisionListPick.Object);
            m_collisionListPick.LoseFocus();
        }
        public TxObjectBase RobotLocationRotationSteps(ITxObject RobLocation, bool Weld_Via, double Steps_X, double Steps_Y, double Steps_Z, double Move_Steps_X, double Move_Steps_Y, double Move_Steps_Z)
        {
            /*
             以原有的自身坐标系为起始坐标系；
             绕Z轴旋转Steps_Z角度，值为度数，不是弧度；
             绕旋转之后的新坐标系Y轴旋转Steps_Y角度，值为度数，不是弧度；
             绕旋转之后的新坐标系X轴旋转Steps_X角度，值为度数，不是弧度；

             */

            TxVector rotation = new TxVector();
            rotation.X = Steps_X * M_PI / 180;
            rotation.Y = Steps_Y * M_PI / 180;
            rotation.Z = Steps_Z * M_PI / 180;


            TxTransformation txTransformation = new TxTransformation();

            txTransformation.Translation.X = Move_Steps_X;
            txTransformation.Translation.Y = Move_Steps_Y;
            txTransformation.Translation.Z = Move_Steps_Z;

            txTransformation.RotationRPY_XYZ = rotation;

            TxVector Translation = new TxVector();
            Translation.X = Move_Steps_X;
            Translation.Y = Move_Steps_Y;
            Translation.Z = Move_Steps_Z;

            TxTransformation loca_Translate = new TxTransformation(Translation, TxTransformation.TxTransformationType.Translate);


            if (Weld_Via)
            {
                TxTransformation weldFrameLocation = ((TxWeldLocationOperation)RobLocation).AbsoluteLocation;
                ((TxWeldLocationOperation)RobLocation).AbsoluteLocation = weldFrameLocation * txTransformation * loca_Translate;
                return ((TxWeldLocationOperation)RobLocation);

            }
            else
            {
                TxTransformation weldFrameLocation = ((TxRoboticViaLocationOperation)RobLocation).AbsoluteLocation;
                ((TxRoboticViaLocationOperation)RobLocation).AbsoluteLocation = weldFrameLocation * txTransformation * loca_Translate;
                return ((TxRoboticViaLocationOperation)RobLocation);

            }



        }
        public double GetRandomDouble(double minValue, double maxValue, double lowerLimit, double UpperLimit)
        {
            if (minValue < lowerLimit) minValue = lowerLimit;
            if (maxValue > UpperLimit) maxValue = UpperLimit;

            Random random = new Random();

            return random.NextDouble() * (maxValue - minValue) + minValue;
        }
       
        private TxRoboticCompositeCommandStringElement CreateOLPCommandElement(string command, TxRoboticViaLocationOperation location)
        {

            var commandElement = new TxRoboticCompositeCommandStringElement(command);

            commandElement.Value = command;
            return commandElement;


        }
        private TxRoboticCompositeCommandStringElement CreateOLPCommandElement(string command, TxWeldLocationOperation location)
        {

            var commandElement = new TxRoboticCompositeCommandStringElement(command);

            commandElement.Value = command;
            return commandElement;


        }
        private void m_pathGenerate_Click(object sender, EventArgs e)
        {

            if (TargetCalWeldOp.Count == 0) return;

            textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Target Robot Path Swept is generating.... " + Environment.NewLine);

            textLogfile.SelectionStart = textLogfile.TextLength;
            textLogfile.ScrollToCaret();

            for (int i = 0; i < TargetCalWeldOp.Count; i++)
            {

                TxRobotAPIClass.CreateSweptVolume(TargetCalWeldOp[i]);

            }
            textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Target Robot Path Swept has been generated ! " + Environment.NewLine);

            textLogfile.SelectionStart = textLogfile.TextLength;
            textLogfile.ScrollToCaret();

            /* 逐个遍历TargetCalWeldOp里面的轨迹，
             * 当前遍历的轨迹依次与后面的轨迹形成干涉区，并用两个机器人的名称来命名
             * 通过两个机器人的名称来找寻干涉区swept;
             * 创建碰撞检查依次提取出进入干涉区的点位
             * uncheck 所有的碰撞检测，并重新设置碰撞检测
             * 
             */


            root = TxApplication.ActiveDocument.CollisionRoot;

            for (int i = 0; i < root.PairList.Count; i++)
            {
                (root.PairList[i] as TxCollisionPair).Active = false;

            }
            if (mainTxControl == null) mainTxControl = this;

            for (int i = 0; i < TargetCalWeldOp.Count; i++)
            {
                for (int j = 0; j < TargetCalWeldOp.Count; j++)
                {
                    if (i == j) continue;

                    collisionSrc.Clear();

                    collisionTar = new TxObjectList();
                    collisionTar.Add(TargetCalWeldOp[i].Robot);
                    collisionTar.Add(TargetCalWeldOp[i].Gun);
                    collisionSrc.Add((TxRobotAPIClass.GetObjectByName<TxSweptVolume>(TargetCalWeldOp[j].Robot.Name)) as ITxObject);
                    (root.PairList[root.PairList.Count - 1] as TxCollisionPair).Active = false;
                    cd = new TxCollisionPairCreationData("cp_collisionZone" + TargetCalWeldOp[i].Robot.Name + "_" + TargetCalWeldOp[j].Robot.Name + "swept", collisionSrc, collisionTar, 3.0);

                    cp = root.CreateCollisionPair(cd);

                    queryParams = new TxCollisionQueryParams();


                    TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
                    opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));

                    TxObjectList allPointsExist = TargetCalWeldOp[i].GetDirectDescendants(opFilter);
                    bool collision_test = false;
                    for (int k = 0; k < allPointsExist.Count; k++)
                    {

                        if (allPointsExist[k].GetType() == typeof(TxWeldLocationOperation))
                        {

                            TxPoseData currentPosedata = (TargetCalWeldOp[i].Robot).GetPoseAtLocation((TxWeldLocationOperation)allPointsExist[k]);
                            if (currentPosedata != null)
                            {
                                (TargetCalWeldOp[i].Robot as TxRobot).CurrentPose = currentPosedata;
                            }

                        }
                        else
                        {
                            TxPoseData currentPosedata = (TargetCalWeldOp[i].Robot).GetPoseAtLocation((TxRoboticViaLocationOperation)allPointsExist[k]);
                            if (currentPosedata != null)
                            {
                                (TargetCalWeldOp[i].Robot as TxRobot).CurrentPose = currentPosedata;
                            }
                        }



                        if ((!TxRobotAPIClass.Collision_Check(mainTxControl, cd, queryParams, root, collisionSrc, collisionTar, 3.0)) && (!collision_test))
                        {
                            //表示有干涉
                            collision_test = true;
                            if (allPointsExist[k].GetType() == typeof(TxRoboticViaLocationOperation))
                            {
                                ArrayList elements = new ArrayList();
                                elements.Add(CreateOLPCommandElement("Enter " + TargetCalWeldOp[j].Robot.Name + " inteference Zone", allPointsExist[k] as TxRoboticViaLocationOperation));

                                TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData = new TxRoboticCompositeCommandCreationData(elements);
                                (allPointsExist[k] as TxRoboticViaLocationOperation).CreateCompositeCommand(txRoboticCompositeCommandCreationData);

                            }
                            else
                            {
                                ArrayList elements = new ArrayList();
                                elements.Add(CreateOLPCommandElement("Enter " + TargetCalWeldOp[j].Robot.Name + " inteference Zone", allPointsExist[k] as TxWeldLocationOperation));

                                TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData = new TxRoboticCompositeCommandCreationData(elements);
                                (allPointsExist[k] as TxWeldLocationOperation).CreateCompositeCommand(txRoboticCompositeCommandCreationData);


                            }


                        }
                        else if (collision_test && TxRobotAPIClass.Collision_Check(mainTxControl, cd, queryParams, root, collisionSrc, collisionTar, 3.0))
                        {

                            collision_test = false;
                            if (allPointsExist[k].GetType() == typeof(TxRoboticViaLocationOperation))
                            {
                                ArrayList elements = new ArrayList();
                                elements.Add(CreateOLPCommandElement("Exit " + TargetCalWeldOp[j].Robot.Name + " inteference Zone", allPointsExist[k] as TxRoboticViaLocationOperation));

                                TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData = new TxRoboticCompositeCommandCreationData(elements);
                                (allPointsExist[k] as TxRoboticViaLocationOperation).CreateCompositeCommand(txRoboticCompositeCommandCreationData);

                            }
                            else
                            {
                                ArrayList elements = new ArrayList();
                                elements.Add(CreateOLPCommandElement("Exit " + TargetCalWeldOp[j].Robot.Name + " inteference Zone", allPointsExist[k] as TxWeldLocationOperation));

                                TxRoboticCompositeCommandCreationData txRoboticCompositeCommandCreationData = new TxRoboticCompositeCommandCreationData(elements);
                                (allPointsExist[k] as TxWeldLocationOperation).CreateCompositeCommand(txRoboticCompositeCommandCreationData);


                            }

                        }
                        else continue;


                    }



                }


            }

            textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Target Robot Path Inteference Zone SetUp finished ! " + Environment.NewLine);

            textLogfile.SelectionStart = textLogfile.TextLength;
            textLogfile.ScrollToCaret();



        }

        private void PathGenerate()
        {
            /* RRT CONNECT计算结束，生辰轨迹点；
            * 首先将没有计算出的焊点移出现有的轨迹
            * 按照焊点顺序和fullpath 的顺序依次生成轨迹坐标

            */
            if (fullpath.Count == 0)
            {
                TxMessageBox.Show("No Path Generated !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
            opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));
            TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if (fullpath.Count != node3D_startptpList.Count)
            {
                // 首先将没算完成的焊点移出去再生成轨迹
                string lastpathweldName = "";
                for (int i = fullpath.Count; i >= 0; i--)
                {
                    if (node3D_startptpList[i].Item1.Item2 != "bypass")
                    {
                        lastpathweldName = node3D_startptpList[i].Item1.Item2;
                        break;
                    }
                }

                for (int i = allWeldPointsExist.Count - 1; i >= 0; i--)
                {
                    if (allWeldPointsExist[i].Name != lastpathweldName)
                    {
                        TxApplication.ActiveDocument.OperationRoot.AddObject(allWeldPointsExist[i]);
                        allWeldPointsExist.Remove(allWeldPointsExist[i]);
                    }
                    else
                        break;

                }

            }


            //确认每个焊点是否都进行了进行robot teach 
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                if (allWeldPointsExist[i].GetType() == typeof(TxWeldLocationOperation))
                {
                    if ((allWeldPointsExist[i] as TxWeldLocationOperation).RobotConfigurationData == null)
                    {
                        TxMessageBox.Show("please Teach the weld spot target point as reference for Robot ConfigurationData !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }
                    if ((allWeldPointsExist[i] as TxWeldLocationOperation).RobotExternalAxesData == null)
                    {
                        TxMessageBox.Show("please Setup ServerGun Joint Value !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }

                }
                else
                {
                    if ((allWeldPointsExist[i] as TxRoboticViaLocationOperation).RobotConfigurationData == null)
                    {
                        TxMessageBox.Show("please Teach the weld spot target point as reference for Robot ConfigurationData !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }
                    if ((allWeldPointsExist[i] as TxRoboticViaLocationOperation).RobotExternalAxesData == null)
                    {
                        TxMessageBox.Show("please Setup ServerGun Joint Value !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                        return;
                    }


                }



            }

            int pathindex = 0;
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                if (pathindex >= fullpath.Count) break;

                if (i == (allWeldPointsExist.Count - 1)) continue;

                for (int j = 0; j < fullpath[pathindex].Count; j++)
                {

                    TxPoseData robotPosture = new TxPoseData();
                    ArrayList robJointValue = new ArrayList();

                    for (int k = 0; k < fullpath[pathindex][j].ToArray().Length; k++)
                    {
                        robJointValue.Add(fullpath[pathindex][j].ToArray()[k]);

                    }

                    robotPosture.JointValues = robJointValue;

                    robot.CurrentPose = robotPosture;


                    point p = new point(

                     robot.TCPF.AbsoluteLocation.Translation.X,
                     robot.TCPF.AbsoluteLocation.Translation.Y,
                     robot.TCPF.AbsoluteLocation.Translation.Z,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.X,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Y,
                    robot.TCPF.AbsoluteLocation.RotationRPY_XYZ.Z,
                   fullpath[pathindex][j].Sever_Gun
                );



                    TxRoboticViaLocationOperation RobFramepostLocation = TxRobotPathOptimizePtp.addrobotPathViaLoc("LocTemp" + allWeldPointsExist[i].Name + j.ToString(), new point(p.x, p.y, p.z,
                       p.rx, p.ry, p.rz, p.Gun_Open), weldTargetOperation, robot, robServerGun);

                    if (allWeldPointsExist[i + 1].GetType() == typeof(TxWeldLocationOperation))
                    {
                        weldTargetOperation.MoveChildAfter((TxWeldLocationOperation)allWeldPointsExist[i + 1], RobFramepostLocation);
                    }
                    else
                    {
                        weldTargetOperation.MoveChildAfter((TxRoboticViaLocationOperation)allWeldPointsExist[i + 1], RobFramepostLocation);
                    }

                    TxRobotConfigurationData txRobotConfigurationData = robot.GetPoseConfiguration(robotPosture);
                    RobFramepostLocation.RobotConfigurationData = txRobotConfigurationData;
                }
                if ((pathindex + 1) >= node3D_startptpList.Count) continue;
                if (allWeldPointsExist[i + 1].Name != node3D_startptpList[pathindex + 1].Item1.Item2) i--;

                pathindex++;
            }
            try
            {
                TxRobotPathOptimizePtp.OperationOptimize(ref weldTargetOperation, robot);
            }
            catch (Exception e) { TxRobotRRTConnectJoint.logpathGenerateOK(e.Message); }

            rrtCalThread = null;

            GC.Collect();


        }



        private Task rrtCalThread;

        bool checkAbsoluteFrameSameorNot(TxTransformation r1, TxTransformation r2)
        {

            point p1 = new point(r1.Translation.X, r1.Translation.Y, r1.Translation.Z,
                r1.RotationRPY_XYZ.X, r1.RotationRPY_XYZ.Y, r1.RotationRPY_XYZ.Z, 0);

            point p2 = new point(r2.Translation.X, r2.Translation.Y, r2.Translation.Z,
               r2.RotationRPY_XYZ.X, r2.RotationRPY_XYZ.Y, r2.RotationRPY_XYZ.Z, 0);


            double posTolerance = 0.1;
            double angleTolerance = 1e-3;

            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            double dz = p1.z - p2.z;
            double distSquared = dx * dx + dy * dy + dz * dz;
            if (distSquared > posTolerance * posTolerance)
                return false;

            // 2. 判断旋转误差，分开考虑rx, ry, rz，处理角度周期2π问题
            double rxDiff = Math.Abs(p1.rx - p2.rx) % (2 * Math.PI);
            if (rxDiff > Math.PI)
                rxDiff = 2 * Math.PI - rxDiff;
            if (rxDiff > angleTolerance)
                return false;

            double ryDiff = Math.Abs(p1.ry - p2.ry) % (2 * Math.PI);
            if (ryDiff > Math.PI)
                ryDiff = 2 * Math.PI - ryDiff;
            if (ryDiff > angleTolerance)
                return false;

            double rzDiff = Math.Abs(p1.rz - p2.rz) % (2 * Math.PI);
            if (rzDiff > Math.PI)
                rzDiff = 2 * Math.PI - rzDiff;
            if (rzDiff > angleTolerance)
                return false;

            return true;


        }

        bool MoveSpotWeldZdirectionAvoidCollision(TxWeldLocationOperation spot, double direction, double movesTipZ, out TxTransformation newSpotLoc, out ArrayList Solutions)
        {
            double x = ((TxWeldLocationOperation)spot).AbsoluteLocation.Translation.X;
            double y = ((TxWeldLocationOperation)spot).AbsoluteLocation.Translation.Y;
            double z = ((TxWeldLocationOperation)spot).AbsoluteLocation.Translation.Z;
            double rx = ((TxWeldLocationOperation)spot).AbsoluteLocation.RotationRPY_XYZ.X;
            double ry = ((TxWeldLocationOperation)spot).AbsoluteLocation.RotationRPY_XYZ.Y;
            double rz = ((TxWeldLocationOperation)spot).AbsoluteLocation.RotationRPY_XYZ.Z;

            int gunindex = 0;
            for (int i = 0; i < spot.RobotExternalAxesData.Count(); i++)
            {
                if (spot.RobotExternalAxesData[i].Device.GetType() == typeof(TxServoGun))
                {
                    gunindex = i; break;
                }

            }

            double gunopening = spot.RobotExternalAxesData[gunindex].JointValue;

            TxPoseData spotPoseData = robot.GetPoseAtLocation(spot as ITxRoboticLocationOperation);


            TxVector rotation = new TxVector();
            rotation.X = 0;
            rotation.Y = 0;
            rotation.Z = 0;
            movesTipZ *= direction;
            TxTransformation txTransformation = new TxTransformation();

            txTransformation.Translation.X = 0;
            txTransformation.Translation.Y = 0;
            txTransformation.Translation.Z = movesTipZ; //退枪3mm;

            txTransformation.RotationRPY_XYZ = rotation;

            TxVector Translation = new TxVector();
            Translation.X = 0;
            Translation.Y = 0;
            Translation.Z = movesTipZ;


            TxTransformation loca_Translate = new TxTransformation(Translation, TxTransformation.TxTransformationType.Translate);

            TxTransformation weldFrameLocation = ((TxWeldLocationOperation)spot).AbsoluteLocation;

            newSpotLoc = weldFrameLocation * txTransformation * loca_Translate;


            double tx = newSpotLoc.Translation.X;
            double ty = newSpotLoc.Translation.Y;
            double tz = newSpotLoc.Translation.Z;
            double Rx = newSpotLoc.RotationRPY_XYZ.X;
            double Ry = newSpotLoc.RotationRPY_XYZ.Y;
            double Rz = newSpotLoc.RotationRPY_XYZ.Z;
            Solutions = TxRobotAPIClass.robotInverseCal(mainTxControl, robot, new point(tx, ty, tz, rx, ry, rz, gunopening));
            if (Solutions.Count == 0)
            {
                return false;
            }
            else
            {
                TxRobotAPIClass.TxRobotPostureGenerate(mainTxControl, robot, TxrrtRobotPathPlannerForm.robServerGun, Solutions, gunopening);
                if (TxRobotAPIClass.Collision_Check(mainTxControl, cd, queryParams, root, collisionSrc, collisionTar, 3.0))
                {
                    return true;
                }
                else
                {
                    if (direction < 0) return false;
                    else if (!MoveSpotWeldZdirectionAvoidCollision(spot, -1 * direction, movesTipZ, out newSpotLoc, out Solutions)) return false;


                }


            }

            return true;

        }


        private void button1_Click(object sender, EventArgs e)
        {

            if (mainTxControl == null) mainTxControl = this;
            if (!rrtconnectCal_ongoing)
            {
                /*
            * 清空log文件
            */
                FileStream stream = File.Open(LogfilePath, FileMode.OpenOrCreate, FileAccess.Write);
                stream.Seek(0, SeekOrigin.Begin);
                stream.SetLength(0);
                stream.Close();
            }
            if (TxApplication.ActiveDocument.SimulationPlayer.TimeInterval != 0.01)
            {
                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Warning: Please set the simulation Time Interval value to 0.01!" + Environment.NewLine);

                textLogfile.SelectionStart = textLogfile.TextLength;
                textLogfile.ScrollToCaret();
                return;

            }
            TxObjectList allWeldPointsExist;
            // 首先确认每个轨迹的焊点是否进行了config， 是否具备gun open的数据
            for (int i = 0; i < TargetCalWeldOp.Count; i++)
            {

                if (TargetCalWeldOp[i].Robot == null)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + TargetCalWeldOp[i].Name + " No Robot select for the Operation !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    return;
                }
                if (TargetCalWeldOp[i].Gun == null)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + TargetCalWeldOp[i].Name + " No Servo Weld Gun select for the Operation !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    return;
                }


                weldTargetOperation = TargetCalWeldOp[i] as TxWeldOperation;

                allWeldPointsExist = weldTargetOperation.GetDirectDescendants(new TxTypeFilter(typeof(TxWeldLocationOperation)));

                if (allWeldPointsExist.Count == 0)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + "No Weld Points founded under the Robot Welding Operation !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    return;

                }
                for (int j = 0; j < allWeldPointsExist.Count - 1; j++)
                {


                    if ((allWeldPointsExist[j] as TxWeldLocationOperation).RobotConfigurationData == null)
                    {
                        textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + allWeldPointsExist[j].Name + " please Teach the weld spot target point as reference for Robot ConfigurationData !" + Environment.NewLine);
                        textLogfile.SelectionStart = textLogfile.TextLength;
                        textLogfile.ScrollToCaret();
                        return;
                    }
                    if ((allWeldPointsExist[j] as TxWeldLocationOperation).RobotExternalAxesData == null)
                    {
                        textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + allWeldPointsExist[j].Name + " please Setup ServerGun Joint Value !" + Environment.NewLine);

                        textLogfile.SelectionStart = textLogfile.TextLength;
                        textLogfile.ScrollToCaret();
                        return;
                    }


                }
            }

            //创建碰撞干涉的检查类组;
            root = TxApplication.ActiveDocument.CollisionRoot;
            for (int i = 0; i < root.PairList.Count; i++)
            {
                string collisionName = root.PairList[i].Name;


                if ((collisionName.Length >= 2 ? collisionName.Substring(0, 2) : collisionName) == "cp")
                {
                    root.PairList[i].Delete();
                    i--;
                }

                else
                {

                    (root.PairList[i] as TxCollisionPair).Active = false;
                    continue;
                }

            }
            //遍历所有需要计算的机器人轨迹，逐个轨迹进行计算
            allWeldPointsExist = new TxObjectList();
            for (int i = 0; i < TargetCalWeldOp.Count; i++)
            {

                isRCSLoaded = false;
                robot = (TargetCalWeldOp[i].Robot) as TxRobot;
                robServerGun = (TargetCalWeldOp[i].Gun) as TxServoGun;

                if ((robot.IsRCSTurnedOffForNonSimulationActions == false
                    && robot.IsRCSTurnedOffForSimulationActions == false
                    && robot.Controller.Name != "default"))

                {
                    isRCSLoaded = true;

                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + robot.Name + " rrtRobot checked the robot controller is not default, will run as " + robot.Controller.Name + " status,which may affect calculation speed !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();

                }

                //确认robot 是否加载了rcs

                if (robServerGun.NonCollidingEntities.Count != 0)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + robServerGun.Name + " Gun Tip not set Non-collision check for" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    continue;

                }
                progressbarNumber = 0;
                TxObjectList gunPoseList = robServerGun.PoseList;
                ArrayList toolJointValues = new ArrayList();
                for (int j = 0; j < gunPoseList.Count; j++)
                {

                    string poseName = ((TxPose)gunPoseList[j]).Name;
                    poseName = poseName.ToLower();
                    if (poseName == "open")
                    {
                        toolJointValues = ((TxPose)gunPoseList[j]).PoseData.JointValues;
                        break;
                    }
                    else
                        continue;

                }
                ToolJointOpening = (double)Convert.ToInt16(toolJointValues[0]);

                if (ToolJointOpening == 0)
                {

                    TxMessageBox.Show("Not Get the " + (i + 1).ToString() + " Weld Path Gun Openning Data, please re-check the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;

                }
                if (!checkAbsoluteFrameSameorNot(robot.TCPF.AbsoluteLocation, robServerGun.TCPF.AbsoluteLocation))
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Current the " + (i + 1).ToString() + " Weld Path Robot TCP is not Gun TCP !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    return;

                }
                collisionTar = new TxObjectList();
                collisionTar.Add(robot);
                collisionTar.Add(robServerGun);

                cd = new TxCollisionPairCreationData("cp1", collisionSrc, collisionTar, 3.0);
                if (cd.FirstList.Count == 0 || cd.SecondList.Count == 0)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " No collision src or target list are setted up successfully" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    return;
                }

                cp = root.CreateCollisionPair(cd);

                queryParams = new TxCollisionQueryParams();

                if (rrtconnectCal_ongoing)// 在计算未完成的时候，强行终止目前的计算
                {
                    rrtconnectCal_ongoing = false;
                    UpdateProgressBar((int)PathprogressBar.Maximum);
                    button1.Text = "Path Connect";
                    m_pathGenerate.Enabled = true;
                    return;

                }

                weldTargetOperation = TargetCalWeldOp[i] as TxWeldOperation;
                TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
                opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));

                allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);


                //if (!weldOperationSpotAllocate(weldTargetOperation)) return;


                progressbarCount = allWeldPointsExist.Count - 1;
                fullpath = new List<List<joint>>();
                node3D_startptpList = new List<Tuple<Tuple<Node3D, string>, TxPoseData>>();
                node3D_endptpList = new List<Tuple<Tuple<Node3D, string>, TxPoseData>>();
                for (int j = 0; j < allWeldPointsExist.Count - 1; j++)
                {
                    if (allWeldPointsExist[j].GetType() == typeof(TxWeldLocationOperation))
                    {
                        TargetCalLocNames.Add(((TxWeldLocationOperation)allWeldPointsExist[j]).Name);
                        double x = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.X;
                        double y = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.Y;
                        double z = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.Z;
                        double rx = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.X;
                        double ry = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.Y;
                        double rz = ((TxWeldLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.Z;



                        TxPoseData spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[j] as ITxRoboticLocationOperation);

                        TxTransformation newSpotLoc = new TxTransformation();
                        ArrayList Solutions = new ArrayList();
                        bool spotFlip = MoveSpotWeldZdirectionAvoidCollision(((TxWeldLocationOperation)allWeldPointsExist[j]), 1.0, 5.0, out newSpotLoc, out Solutions);
                        if (!spotFlip)
                        {

                            spotFlip = MoveSpotWeldZdirectionAvoidCollision(((TxWeldLocationOperation)allWeldPointsExist[j]), 1.0, 3.0, out newSpotLoc, out Solutions);

                            if (!spotFlip)
                            {
                                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Warning : Spot " + allWeldPointsExist[j].Name + " can not moved in Z direction" + Environment.NewLine);

                                textLogfile.SelectionStart = textLogfile.TextLength;
                                textLogfile.ScrollToCaret();
                                return;
                            }

                        }

                        int nearestIndex = TxRobotPathOptimizePtp.ChooseBestInverseSolution(ref Solutions, spotPoseData);

                        spotPoseData = (TxPoseData)Solutions[nearestIndex];

                        Node3D node3D_start = new Node3D(newSpotLoc.Translation.X, newSpotLoc.Translation.Y, newSpotLoc.Translation.Z,
                               newSpotLoc.RotationRPY_XYZ.X, newSpotLoc.RotationRPY_XYZ.Y, newSpotLoc.RotationRPY_XYZ.Z);

                        node3D_startptpList.Add(Tuple.Create(Tuple.Create(node3D_start, allWeldPointsExist[j].Name), spotPoseData));
                    }
                    else
                    {
                        TargetCalLocNames.Add(((TxRoboticViaLocationOperation)allWeldPointsExist[j]).Name);
                        double x = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.X;
                        double y = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.Y;
                        double z = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.Translation.Z;
                        double rx = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.X;
                        double ry = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.Y;
                        double rz = ((TxRoboticViaLocationOperation)allWeldPointsExist[j]).AbsoluteLocation.RotationRPY_XYZ.Z;



                        TxPoseData spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[j] as ITxRoboticLocationOperation);

                        Node3D node3D_start = new Node3D(x, y, z, rx, ry, rz);
                        node3D_startptpList.Add(Tuple.Create(Tuple.Create(node3D_start, allWeldPointsExist[j].Name), spotPoseData));

                    }

                    if (allWeldPointsExist[j + 1].GetType() == typeof(TxWeldLocationOperation))
                    {
                        double x = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.X;
                        double y = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.Y;
                        double z = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.Z;
                        double rx = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.X;
                        double ry = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.Y;
                        double rz = ((TxWeldLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.Z;


                        // Node3D node3D_goal = new Node3D(x, y, z, rx, ry, rz);
                        TxPoseData spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[j + 1] as ITxRoboticLocationOperation);


                        TxTransformation newSpotLoc = new TxTransformation();
                        ArrayList Solutions = new ArrayList();
                        bool spotFlip = MoveSpotWeldZdirectionAvoidCollision(((TxWeldLocationOperation)allWeldPointsExist[j + 1]), 1.0, 5.0, out newSpotLoc, out Solutions);
                        if (!spotFlip)
                        {

                            spotFlip = MoveSpotWeldZdirectionAvoidCollision(((TxWeldLocationOperation)allWeldPointsExist[j + 1]), 1.0, 3.0, out newSpotLoc, out Solutions);
                            if (!spotFlip)
                            {
                                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " Warning : Spot " + allWeldPointsExist[j + 1].Name + " can not moved in Z direction" + Environment.NewLine);

                                textLogfile.SelectionStart = textLogfile.TextLength;
                                textLogfile.ScrollToCaret();
                                return;
                            }

                        }
                        int nearestIndex = TxRobotPathOptimizePtp.ChooseBestInverseSolution(ref Solutions, spotPoseData);

                        spotPoseData = (TxPoseData)(Solutions[nearestIndex]);

                        Node3D node3D_goal = new Node3D(newSpotLoc.Translation.X, newSpotLoc.Translation.Y, newSpotLoc.Translation.Z,
                               newSpotLoc.RotationRPY_XYZ.X, newSpotLoc.RotationRPY_XYZ.Y, newSpotLoc.RotationRPY_XYZ.Z);


                        node3D_endptpList.Add(Tuple.Create(Tuple.Create(node3D_goal, allWeldPointsExist[j + 1].Name), spotPoseData));

                    }
                    else
                    {
                        double x = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.X;
                        double y = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.Y;
                        double z = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.Translation.Z;
                        double rx = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.X;
                        double ry = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.Y;
                        double rz = ((TxRoboticViaLocationOperation)allWeldPointsExist[j + 1]).AbsoluteLocation.RotationRPY_XYZ.Z;


                        Node3D node3D_goal = new Node3D(x, y, z, rx, ry, rz);
                        TxPoseData spotPoseData = robot.GetPoseAtLocation(allWeldPointsExist[j + 1] as ITxRoboticLocationOperation);
                        node3D_endptpList.Add(Tuple.Create(Tuple.Create(node3D_goal, allWeldPointsExist[j + 1].Name), spotPoseData));

                    }





                }
                if (allWeldPointsExist[allWeldPointsExist.Count - 1].GetType() == typeof(TxWeldLocationOperation))
                {
                    TargetCalLocNames.Add(((TxWeldLocationOperation)allWeldPointsExist[allWeldPointsExist.Count - 1]).Name);

                }
                else
                {
                    TargetCalLocNames.Add(((TxRoboticViaLocationOperation)allWeldPointsExist[allWeldPointsExist.Count - 1]).Name);

                }

                rrtconnectCal_ongoing = true;
                button1.Text = "Stop";

                PathprogressBar.Value = 0;
                m_collisionListPick.Enabled = false;
                m_GroupTargetOpPick.Enabled = false;


                if (collisionSrc.Count != 0)
                {
                    m_collisionListPick.SelectObject(collisionSrc[collisionSrc.Count - 1]);
                }




                /* rrtconnectCalOnGoing_Async()异步启动rrt计算。
                 * Tecnomatix不支持多线程开发，所以为了达到进度条的显示和软件的可视化，采用异步机制；
                 * 之前采用多线程，process simulate 软件总是莫名退出，提示内存读写冲突；
                 */
                TxRobotRRTConnectJoint.logpathGenerateOK("Path Calculate Start !");
                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " The " + (i + 1).ToString()
                    + "th Path Calculate Start !" + Environment.NewLine);

                textLogfile.SelectionStart = textLogfile.TextLength;
                textLogfile.ScrollToCaret();
                Pathend_nodes.Clear();
                rrtconnectCalOnGoingPtp_Async();
                if (!rrtconnectCal_ongoing)
                {
                    textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " The " + (i + 1).ToString()
                     + "th Path Calculate Stopped !" + Environment.NewLine);

                    textLogfile.SelectionStart = textLogfile.TextLength;
                    textLogfile.ScrollToCaret();
                    break;
                }

                PathGenerate();
                TargetCalLocNames.Clear();
                UpdateProgressBar(PathprogressBar.Maximum);
                TxRobotRRTConnectJoint.logpathGenerateOK("Path Calculate end !");
                textLogfile.AppendText(DateTime.Now.ToLocalTime().ToString() + " The " + (i + 1).ToString()
                    + "th Path Calculate End !" + Environment.NewLine);

                textLogfile.SelectionStart = textLogfile.TextLength;
                textLogfile.ScrollToCaret();
                rrtconnectCal_ongoing = false;
            }
            SetButtonState(m_pathGenerate, true);
            SetButtonState(button1, false);
            SetButtonText(button1, "Path Connect");
        }
        private void rrtconnectCalOnGoingPtp_Async()
        {
            try
            {
                for (int i = 0; i < node3D_startptpList.Count; i++)
                {

                    if (!rrtconnectCal_ongoing)
                    {
                        break;
                    }


                    List<joint> path = new List<joint>();
                    Node3D node3D_start = node3D_startptpList[i].Item1.Item1;
                    Node3D node3D_goal = node3D_endptpList[i].Item1.Item1;
                    TxPoseData robstartPosedata = node3D_startptpList[i].Item2;
                    TxPoseData robendPosedata = node3D_endptpList[i].Item2;


                    TxRobotRRTConnectJoint rrt = new TxRobotRRTConnectJoint();
                    rrt.j1Llimit = robot.Joints[0].LowerSoftLimit;
                    rrt.j1Ulimit = robot.Joints[0].UpperSoftLimit;

                    rrt.j2Llimit = robot.Joints[1].LowerSoftLimit;
                    rrt.j2Ulimit = robot.Joints[1].UpperSoftLimit;

                    rrt.j3Llimit = robot.Joints[2].LowerSoftLimit;
                    rrt.j3Ulimit = robot.Joints[2].UpperSoftLimit;

                    rrt.j4Llimit = robot.Joints[3].LowerSoftLimit;
                    rrt.j4Ulimit = robot.Joints[3].UpperSoftLimit;

                    rrt.j5Llimit = robot.Joints[4].LowerSoftLimit;
                    rrt.j5Ulimit = robot.Joints[4].UpperSoftLimit;

                    rrt.j6Llimit = robot.Joints[5].LowerSoftLimit;
                    rrt.j6Ulimit = robot.Joints[5].UpperSoftLimit;


                    joint start_p = new joint(

                      (double)robstartPosedata.JointValues[0],
                      (double)robstartPosedata.JointValues[1],
                      (double)robstartPosedata.JointValues[2],
                      (double)robstartPosedata.JointValues[3],
                      (double)robstartPosedata.JointValues[4],
                      (double)robstartPosedata.JointValues[5],
                      ToolJointOpening

                        );

                    joint end_p = new joint(

                      (double)robendPosedata.JointValues[0],
                      (double)robendPosedata.JointValues[1],
                      (double)robendPosedata.JointValues[2],
                      (double)robendPosedata.JointValues[3],
                      (double)robendPosedata.JointValues[4],
                      (double)robendPosedata.JointValues[5],
                      ToolJointOpening

                     );

                    rrt.rrt_connectJointPtp(mainTxControl, start_p, end_p);

                    if (!TxRobotRRTConnectJoint.currentpathdone) //表示当下的计算没有产生合适的路径而退出
                    {
                        i--;
                        rrt.Dispose();
                        TxRobotRRTConnectJoint.logpathGenerateOK("Re-calcualed the " + (i + 1).ToString() + " Path!");
                        continue;
                    }

                    /*
                    * 如果迭代次数超过1000，而造成退出，则在退出的起始点和终止点增加一个中点，然后再进行计算
                    */
                    if (rrtconnectCal_ongoing)
                    {
                        path = rrt.path_points_start;
                        fullpath.Add(path);
                    }

                    rrt.Dispose();

                    TxRobotRRTConnectJoint.currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
                    if (node3D_endptpList[i].Item1.Item2 != "bypass") progressbarNumber++;
                    if (rrtconnectCal_ongoing)
                        UpdateProgressBar((int)((PathprogressBar.Maximum * ((double)progressbarNumber / progressbarCount))));

                }


            }
            catch (Exception ex)
            {

                TxRobotRRTConnectJoint.logpathGenerateOK(ex.Message);


            }




        }
        private void UpdateProgressBar(int value)
        {
            if (PathprogressBar.InvokeRequired)
            {
                // Use Invoke to update the progress bar on the UI thread
                PathprogressBar.Invoke(new Action(() => PathprogressBar.Value = value));
            }
            else
            {
                // Directly update the progress bar if already on UI thread
                PathprogressBar.Value = value;
            }
        }

        private void SetButtonState(System.Windows.Forms.Button myButton, bool enabled)
        {
            if (myButton.InvokeRequired)
            {
                myButton.Invoke(new Action(() => myButton.Enabled = enabled));
            }
            else
            {
                myButton.Enabled = enabled;
            }
        }

        private void SetButtonText(System.Windows.Forms.Button myButton, string text)
        {
            if (myButton.InvokeRequired)
            {
                myButton.Invoke(new Action(() => myButton.Text = text));
            }
            else
            {
                myButton.Text = text;
            }
        }

        private void TxrrtRobotPathPlannerForm_FormClosed(object sender, FormClosedEventArgs e)
        {
            TargetCalWeldOp.Clear();
            TargetCalLocNames.Clear();
            if (node3D_startptpList != null) node3D_startptpList.Clear();
            if (node3D_endptpList != null) node3D_endptpList.Clear();
            if (fullpath != null) fullpath.Clear();
            if (robot != null) robot = null;
            if (robServerGun != null) robServerGun = null;
            if (collisionSrc != null) collisionSrc = null;
            if (collisionTar != null) collisionTar = null;
        }

       
    }
}
