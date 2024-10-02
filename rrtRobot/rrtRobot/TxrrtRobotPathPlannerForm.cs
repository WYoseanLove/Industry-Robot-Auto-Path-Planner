using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.Threading.Tasks;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Ui;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Button;
using System.Xml.Linq;
using Tecnomatix.Engineering.PrivateImplementationDetails;



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
     * 软件界面需要定义机器人轨迹中间过度点的"逃离方向"，RRT*CONNECT只进行迭代计算1000次，超过1000次会退出并生成中间过度点，起始点和中间过度点，以及中间过度点和终止点会分别再次进行RRT*CONNECT计算，直到生成轨迹
     * 中间过度点的逃离方向是基于机器人基坐标的X/Y/Z轴，用于需要选择沿着哪个轴进行逃离计算可以得到中间过度点，中间过度点的算法优化目前还在开发当中；
     * 完成以上任务之后，所有的配置已经完成；
     * 选中需要计算的机器人轨迹(只能包括焊点，其他的类型点不参加计算）, 点击path connect,计算完成后，点击生成轨迹即可；
     */

    public partial class TxrrtRobotPathPlannerForm : TxForm
    {
        public static double M_PI = 3.1415926;
        //public static IntPtr KukaRobot;
        public static TxTransformation TCPLocation; //记录所选机器人的TCP Frame 值，从robot 的Tool frame 到TCP frame 的转换矩阵，用于正向运动学计算
        public static TxTransformation ToolFrameLocation; //记录所选机器人的Tool Frame 值，从robot 的Tcp frame 到Tool frame 的转换矩阵,用于逆运动学计算
        public static TxFrame baseFrame; //记录所选机器人的Base Frame 值;

        private static readonly object lockObj = new object();

        public static TxObjectList collisionTar;
        public static TxCollisionPairCreationData cd;
        public static TxCollisionPair cp;
        public static TxCollisionQueryParams queryParams ;
        public static TxObjectList collisionSrc;
        public static TxCollisionRoot root;
        public static TxRobot robot;
        private int progressbarNumber = 0;
        private int progressbarCount = 0;
        public TxWeldOperation weldTargetOperation;
        public static double ToolJointOpening; // 存放的是焊钳的open的尺寸
        public static bool rrtconnectCal_ongoing=false;
        public List<Tuple<Node3D, string>> node3D_startList ;
        public List<Tuple<Node3D, string>> node3D_endList ;
        public static int iterate_Count = 0;// 记录由于1000次rrt 迭代计算无法得到结果而跳出的次数，这个次数会决定了rrt生成的RX/RY的旋转角度

        // 用于log文件txt的生成，在系统Documents/rrtRobot文件夹下面
        // 获取当前用户的Documents路径
        public static string LogfilePath;
        // 定义子文件夹名称和文件名

        public char spotagainstCollisionSrc;
        public TxrrtRobotPathPlannerForm()
        {
            InitializeComponent();
            GenerateLogfile();
        }
        public void GenerateLogfile()
        {
            // 组合完整的子文件夹路径和文件路径
            string documentsPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            string subFolderName = "rrtRobot";
             string fileName = "dataLog.txt";

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
            label1.Font=new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold| System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            m_spotDirec.Font= new System.Drawing.Font("Microsoft Sans Serif", 12, System.Drawing.FontStyle.Bold | System.Drawing.FontStyle.Underline, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            Robot_Name.Font= new System.Drawing.Font("Microsoft Sans Serif", 13);
            TCP.Font = new System.Drawing.Font("Microsoft Sans Serif", 13);
            Collision_Src.Font = new System.Drawing.Font("Microsoft Sans Serif", 13);
            groupBox2.Size=new System.Drawing.Size(465, 112);
            Group_Collision.Size = new System.Drawing.Size(465,103);
            groupBox1.Size = new System.Drawing.Size(465, 92);
            groupBox3.Size = new System.Drawing.Size(465, 54);
            this.Size = new System.Drawing.Size(507, 519);
            this.FormBorderStyle = FormBorderStyle.Fixed3D;
            collisionSrc = new TxObjectList();

            m_collisionListPick.Enabled = false;
            button1.Enabled = false;

        }
        private void m_txRobotName_Picked(object sender, TxObjEditBoxCtrl_PickedEventArgs args)
        {
            /* 定义用于轨迹分析和生成的机器人，并将其存储在TxRobot robot 中去，作为成员变量，供其他函数和类调用
             * 选中的机器人的工具坐标会在对话框中显示出来
             * 如果机器人安装了多个工具，则软件会自动将所有的工具全部卸掉，需要人工重新安装，确保机器人上的工具就是进行轨迹分析的焊钳，且只有一个；
             * 目前无法自动识别，安装的工具是焊钳还是其他工具如抓手等等，待找到相应的API之后进行更新；
             * ToolJointOpening 变量存储的是焊钳open的角度，这个角度会作为参数供TxRootRRTConnect等其他类进行调用；
             * 在参数ToolJointOpening赋值的过程中，使用了try,如果报错，则说明Mounted Tool 并非weld Gun, 或者是焊钳的设置有问题，需要重新检查和安装工具
             * 将机器人，和焊钳添加到碰撞检测的target中去，Src的定义需要用户来实现
             */

           robot = m_txRobotName.Object as TxRobot;
            if (robot != null)
            {

                TxFrame tcpFrame = robot.TCPF;
                TxFrame robot_J6_Frame = robot.Toolframe;
                baseFrame = robot.Baseframe;
                TCPLocation = tcpFrame.GetLocationRelativeToObject(robot_J6_Frame);
                ToolFrameLocation = robot_J6_Frame.GetLocationRelativeToObject(tcpFrame);
                m_txTCPFrame.Text = TCPLocation.Translation.ToString(2) + " " + TCPLocation.RotationRPY_XYZ.ToString(2);
                TxApplication.ActiveDocument.WorkingFrame = robot.AbsoluteLocation;
            }
            else
            {
                m_txTCPFrame.Text = string.Empty;
                TxMessageBox.Show("No Robot selected SetUp", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            ITxDevice tSelectObject = robot as ITxDevice;

            ArrayList toolJointValues = new ArrayList();

            if (robot.MountedTools.Count == 0)
            {
                TxMessageBox.Show("No Weld Gun Added in Picked Robot Tool Box !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            tSelectObject = robot.MountedTools[0] as ITxDevice;
            if (robot.MountedTools.Count > 1)
            {
                for (int i = 0; i < robot.MountedTools.Count; i++)
                {
                    robot.UnmountTool(robot.MountedTools[i] as ITxLocatableObject);
                    i--;
                }

                TxMessageBox.Show("Weld Gun not set as the first Mounted Tools, please re-mount the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;


            }

            TxObjectList x = tSelectObject.PoseList;
            try
            {

                for (int i = 0; i < x.Count; i++)
                {

                    string poseName = ((TxPose)x[i]).Name;
                    poseName = poseName.ToLower();
                    if (poseName == "open")
                    {
                        toolJointValues = ((TxPose)x[i]).PoseData.JointValues;
                        break;
                    }
                    else
                        continue;

                }
                ToolJointOpening = (double)Convert.ToInt16(toolJointValues[0]);

                if (ToolJointOpening == 0)
                {
                    for (int i = 0; i < robot.MountedTools.Count; i++)
                    {
                        robot.UnmountTool(robot.MountedTools[i] as ITxLocatableObject);
                        i--;
                    }
                    TxMessageBox.Show("Not Get the Gun Openning Data, please re-check the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;

                }
            }
            catch (Exception e)
            {
                TxMessageBox.Show("Weld Gun not set as System Requested Mounted Tools, please re-mount the Gun !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;

            }
            collisionTar = ((TxRobot)(m_txRobotName.Object)).MountedTools;
            collisionTar.Add(robot);
            m_collisionListPick.Enabled = true;
            button1.Enabled = true;
        }

        private void m_collisionListPicked(object sender, TxObjComboBoxCtrl_PickedEventArgs args)
        {
            TxObjectBase txObjects = m_collisionListPick.Object as TxObjectBase;
            collisionSrc.Add(m_collisionListPick.Object);
            m_collisionListPick.AddItem(m_collisionListPick.Object.Name, m_collisionListPick.Object);
            m_collisionListPick.LoseFocus();
        }

        private void weldOperationSpotAllocate(TxWeldOperation weldOperation)
        {
            /*对所选择的进行轨迹规划的Operation 进行焊点分析，确保每个焊点均均被可达性，且干涉量为0；
             * 如果不可达，或者存在干涉的情况，则自动绕焊点的Z轴进行旋转，每旋转10deg作为一个step, 直到此焊点可达为止，即退出并进行下一个
             * 如果旋转36次之后仍未可达，则将此焊点移出Operation,不进行焊接轨迹的计算；
             * 调用RobotKinemetix.robotInverseCal()函数进行逆向学计算
             * RobotKinemetix.TxRobotPostureGenerate(),将机器人调试至焊点姿态；
             * Collision_Check(),确认是否有干涉存在；
             */

            if (weldOperation == null)
                return;

            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
            TxObjectList allWeldPointsExist = weldOperation.GetDirectDescendants(opFilter);
            bool postureOK = false;
            int LocRotationCount = 0;

            try
            {
                for (int i = 0; i < allWeldPointsExist.Count; i++)
                {
                    double tx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                    double ty = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                    double tz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                    double rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                    double ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                    double rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;

                    point weldPoc = new point(tx, ty, tz, rx, ry, rz,0);
                    ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, weldPoc);
                    while (!postureOK)
                    {
                        if (Solutions.Count != 0)
                        {
                            RobotKinemetix.TxRobotPostureGenerate(robot, Solutions, 0);
                            if (Collision_Check(cd, queryParams, root, collisionSrc, collisionTar, 3.0))
                            {
                                postureOK = true;
                                break;
                            }


                        }
                        ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation = ((TxWeldLocationOperation)RobotLocationRotationSteps(allWeldPointsExist[i], true, 0, 0, 10, 0, 0, 0)).AbsoluteLocation;
                        tx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                        ty = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                        tz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                        rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                        ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                        rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;
                        Solutions = RobotKinemetix.robotInverseCal(robot, new point(tx, ty, tz, rx, ry, rz,0));
                        LocRotationCount++;
                        if (LocRotationCount == 36)
                        {
                            TxApplication.ActiveDocument.OperationRoot.AddObject(((TxWeldLocationOperation)allWeldPointsExist[i]));
                            break;
                        }
                       
                        else
                            continue;



                    }

                    postureOK = false;
                    LocRotationCount = 0;

                }

            }
            catch (Exception)
            {
                throw;
            }

            /*
           * 清空log文件
           */
            FileStream stream = File.Open(LogfilePath, FileMode.OpenOrCreate, FileAccess.Write);
            stream.Seek(0, SeekOrigin.Begin);
            stream.SetLength(0);
            stream.Close();


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

        public static bool Collision_Check(TxCollisionPairCreationData cd, TxCollisionQueryParams queryParams, TxCollisionRoot root, TxObjectList collisionSrc, TxObjectList collisionTar, double Clearance)

        {  
            queryParams.Mode = TxCollisionQueryParams.TxCollisionQueryMode.DefinedPairs;

            queryParams.NearMissDistance = Clearance;
 
            TxCollisionQueryResults results = root.GetCollidingObjects(queryParams);
            if (results.States.Count == 0)
            {
               
                cd.Dispose();

                return true;
            }

            for (int i=0;i< results.States.Count;i++)
            {
                if ((results.States[i] as TxCollisionState).Type==TxCollisionState.TxCollisionStateType.Collision)
                {
                    cd.Dispose();

                    return false;

                }
            }


            cd.Dispose();
           
            return true;
        }
        
        public static List<List<point>> fullpath;
        private bool generatePassThroughPoints(ref point p)
        {
            /*当RRTCONNECT 算法迭代1000次计算，仍未找到合适的路径时候，软件会跳出RRTCONNECT计算，并根据start，end node点生成中间过度点(PassThroughPoints)
             * 首先spotagainstCollisonSrc()函数来确认过度点是按照目前焊点坐标系的哪个轴去移动，用户在界面中输入的spot direction就是用于计算此过度点移动轴
             * spotagainstCollisonSrc()的计算方式是将焊点坐标系投影到机器人的基坐标系中，并计算各个轴在用于界面选取的坐标轴夹角的sin 值，值越小，说明焊点坐标系的这个轴越靠近选取的贝利方向
             * 首先定义stepsize, 根据stepsize 进行移动，如果焊钳TCP距离机器人更远了，说明移动的方向相反，则更改stepsize 的符号；
             * 过度点的位置为起始点和终止点的中点，默认按照中点去不断的靠近终止点，方向与p点的方向相同；
             * 过度点按照事先计算出的移动方向开始移动，直到与干涉物碰撞检测通过之后确认过度点，并加入到path 计算中去
             * 如果移动的次数超过20次仍未通过碰撞检测，则说明失败，则退出，并从轨迹中移除end node， 说明这段轨迹计算失败
             * 目前计算失败的案例都是因为过度点的计算失效的，根本原因就是用户选择的方向，这个方向需要研究算法可以自动计算，但目前无法做到
            */
            List<double> distoAix= RobotKinemetix.spotagainstCollisonSrc(p, new point(robot.AbsoluteLocation.Translation.X,
               robot.AbsoluteLocation.Translation.Y,
               robot.AbsoluteLocation.Translation.Z,
               robot.AbsoluteLocation.RotationRPY_XYZ.X,
               robot.AbsoluteLocation.RotationRPY_XYZ.Y,
               robot.AbsoluteLocation.RotationRPY_XYZ.Z, p.Gun_Open), spotagainstCollisionSrc);
            point temP = p;
            double stepsize = 50;
            // 首先沿着计算出的轴移动50mm，确定stepsize 的方向，移动的方向需要距离机器人更近
            List<double> stepMoves = new List<double>() { 0,0,0};
            stepMoves = confirmStepMovesDirection(stepMoves, distoAix, stepsize, p);
            while (true)
            {
                if (spotAgainstCollisionMotion(stepMoves,  distoAix,  stepsize, p, out temP))
                    break;


                int minIndex = distoAix.IndexOf(distoAix.Min());

                distoAix[minIndex] =Double.MaxValue;
                minIndex = distoAix.IndexOf(distoAix.Min());
                if (distoAix[minIndex] == Double.MaxValue)
                    return false;
                stepMoves = new List<double>() { 0, 0, 0 };
                stepMoves = confirmStepMovesDirection(stepMoves, distoAix, stepsize, p);

            }

            p = temP;

            TxRobotRRTConnect.LogInformation(p, distoAix);
            return true;
        }
        
        private List<double> confirmStepMovesDirection(List<double> stepMoves, List<double> distoAix,double stepsize, point p)
        {
            List<double> tempmoves = new List<double>();
            double newdistance = 0;
            point temP = p;
            double distance = TxRobotRRTConnect.dist(p, new point(robot.AbsoluteLocation.Translation.X,
              robot.AbsoluteLocation.Translation.Y,
             robot.AbsoluteLocation.Translation.Z,
             0, 0, 0, 0));

            int minIndex = distoAix.IndexOf(distoAix.Min());
            if(minIndex>2)
            {
                return stepMoves;
            }

            stepMoves[minIndex] = stepsize;

            tempmoves = TxRobotRRTConnect.getVectorfromTranslate(temP, stepMoves);

            newdistance = TxRobotRRTConnect.dist(new point(tempmoves[0], tempmoves[1], tempmoves[2], p.rx, p.ry, p.rz, 0), new point(robot.AbsoluteLocation.Translation.X,
            robot.AbsoluteLocation.Translation.Y,
            robot.AbsoluteLocation.Translation.Z,
            0, 0, 0, 0));

            if (newdistance > distance)
            {
                stepsize = -stepsize;
                stepMoves[minIndex] = stepsize;
            }

            return stepMoves;
        }

        private bool spotAgainstCollisionMotion(List<double> stepMoves, List<double> distoAix, double stepsize, point p,out point temP)
        {

            List<double> tempmoves = new List<double>();
            temP = p;
            int n = 1;
            while (true)
            {
                if (TxRobotRRTConnect.collisioncheckforSinglePoint(temP)) break;

                tempmoves = TxRobotRRTConnect.getVectorfromTranslate(temP, stepMoves);
                temP = new point(tempmoves[0], tempmoves[1], tempmoves[2], p.rx, p.ry, p.rz, ToolJointOpening);
                int minIndex = distoAix.IndexOf(distoAix.Min());
                stepMoves[minIndex] += stepsize * (stepMoves[minIndex] / (Math.Abs(stepMoves[minIndex])));
                n++;
                if (n > 20) return false;
                Thread.Sleep(50);

            }

            return true;


        }
        private void m_pathGenerate_Click(object sender, EventArgs e)
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
            
            TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if(fullpath.Count!= node3D_startList.Count)
            {
                // 首先将没算完成的焊点移出去再生成轨迹
                string lastpathweldName = "";
                for (int i = fullpath.Count - 1; i >= 0; i--)
                {
                    if (node3D_startList[i].Item2 != "bypass")
                    {
                        lastpathweldName = node3D_startList[i].Item2;
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
                if ((allWeldPointsExist[i] as TxWeldLocationOperation).RobotConfigurationData == null)
                {
                    TxMessageBox.Show("please Teach the weld spot target point as reference for Robot ConfigurationData !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    return;
                }


            }

            int pathindex = 0;
            for (int i = 0; i < allWeldPointsExist.Count; i++)
            {
                if (pathindex >= fullpath.Count) break;

                if (i == (allWeldPointsExist.Count - 1)) continue;

                for (int j = 0; j < fullpath[pathindex].Count; j++)
                {
                  
                    TxRoboticViaLocationOperation RobFramepostLocation = TxRobotPathOptimize.addrobotPathViaLoc("LocTemp" + allWeldPointsExist[i].Name + j.ToString(), new point(fullpath[pathindex][j].x, fullpath[pathindex][j].y, fullpath[pathindex][j].z,
                        fullpath[pathindex][j].rx, fullpath[pathindex][j].ry, fullpath[pathindex][j].rz, fullpath[pathindex][j].Gun_Open), weldTargetOperation,robot);
                   
                    weldTargetOperation.MoveChildAfter((TxWeldLocationOperation)allWeldPointsExist[i + 1], RobFramepostLocation);
                    
                }
                if ((pathindex + 1) >= node3D_startList.Count) continue;
                if (allWeldPointsExist[i + 1].Name != node3D_startList[pathindex + 1].Item2) i--;

                pathindex++;
            }

            bool optimizedDone= TxRobotPathOptimize.OperationOptimize2(ref weldTargetOperation,robot);
            if(optimizedDone)
            {
                cp.Delete();
                TxApplication.ActiveSelection.Clear();
            }
            else
            {

                opFilter.AddIncludedType(typeof(TxRoboticViaLocationOperation));

                TxObjectList _robotOpLocation = weldTargetOperation.GetAllDescendants(opFilter);

                for (int i=0;i< _robotOpLocation.Count;i++)
                {

                    if (_robotOpLocation[i].GetType() == typeof(TxWeldLocationOperation)) continue;
                    else
                    {
                        _robotOpLocation[i].Delete();
                        _robotOpLocation.RemoveAt(i);
                    }
                   
                }

            }
           

        }
        
        private async void button1_Click(object sender, EventArgs e)
        {
            //创建碰撞干涉的检查类组：

            root = TxApplication.ActiveDocument.CollisionRoot;
            for(int i=0;i<root.PairList.Count;i++)
            {
                if (root.PairList[i].Name == "cp1")
                {
                    root.PairList[i].Delete();
                    i--;
                }

                else
                    continue;
            }

            cd = new TxCollisionPairCreationData("cp1", collisionSrc, collisionTar, 3.0);
            if(cd.FirstList.Count==0|| cd.SecondList.Count==0)
            {
                TxMessageBox.Show("No collision src or target list are setted up successfully", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            cp = root.CreateCollisionPair(cd);

            queryParams = new TxCollisionQueryParams();


            if (rrtconnectCal_ongoing)// 在计算未完成的时候，强行终止目前的计算
            {
                rrtconnectCal_ongoing = false;
                PathprogressBar.Value = PathprogressBar.Maximum;
                button1.Text = "Path Connect";
                m_pathGenerate.Enabled = true;
                return;
            }

            /*
             * 1. 首先确认operation的根目录下是否选择的焊接程序，如果有选择的，自动规划已选择的焊点程序
             * 2. 确认选择的焊点程序中每个焊点均是可达的，且没有干涉，如果有干涉，则重新定义焊点的方向直至不干涉；
             * 3. 如果无法达到，且无法避免干涉，则将此焊点移除选择的焊点程序，并放到根目录下；           
             */

             ITxObject selectItem=   TxApplication.ActiveSelection.GetLastPickedItem() as ITxObject;
            if(selectItem ==null|| selectItem.GetType().Name!="TxWeldOperation")
            {
                TxMessageBox.Show("Please select the Weld OP firstly before connect!", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            progressbarCount = 0;
            //确认operation的根目录下是否存在没有分配的焊点，如果有则确认是否具备机器人的可达性           
            TxOperationRoot txOperationRoot = TxApplication.ActiveDocument.OperationRoot;
            TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));
            weldTargetOperation=selectItem as TxWeldOperation;
            TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            if (allWeldPointsExist.Count == 0)
            {
                TxMessageBox.Show("No Weld Points founded under the Robot Welding Operation !", "Warnning", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;

            }

            // 确定选择的焊点轨迹，每个焊点均能够达到，不能达到则改变方向或者移出来
            weldOperationSpotAllocate(weldTargetOperation);
            allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);

            progressbarCount = allWeldPointsExist.Count-1;

            fullpath = new List<List<point>>();
            node3D_startList = new List<Tuple<Node3D, string>>();
            node3D_endList = new List<Tuple<Node3D, string>>();
            for (int i = 0; i < allWeldPointsExist.Count - 1; i++)
            {

                double x = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.X;
                double y = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Y;
                double z = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.Translation.Z;
                double rx = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.X;
                double ry = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Y;
                double rz = ((TxWeldLocationOperation)allWeldPointsExist[i]).AbsoluteLocation.RotationRPY_XYZ.Z;


                Node3D node3D_start = new Node3D(x, y, z, rx, ry, rz);
                node3D_startList.Add(Tuple.Create(node3D_start, allWeldPointsExist[i].Name));

                x = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.X;
                y = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.Y;
                z = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.Translation.Z;
                rx = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.X;
                ry = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.Y;
                rz = ((TxWeldLocationOperation)allWeldPointsExist[i + 1]).AbsoluteLocation.RotationRPY_XYZ.Z;


                Node3D node3D_goal = new Node3D(x, y, z, rx, ry, rz);
                node3D_endList.Add(Tuple.Create(node3D_goal, allWeldPointsExist[i + 1].Name));

            }

            rrtconnectCal_ongoing = true;
            button1.Text = "Stop";

            PathprogressBar.Value = 0;
            m_collisionListPick.Enabled = false;
            m_txRobotName.Enabled = false;
            /* rrtconnectCalOnGoing_Async()异步启动rrt计算。
             * Tecnomatix不支持多线程开发，所以为了达到进度条的显示和软件的可视化，采用异步机制；
             * 之前采用多线程，process simulate 软件总是莫名退出，提示内存读写冲突；
             */
            await rrtconnectCalOnGoing_Async(node3D_startList, node3D_endList);
            PathprogressBar.Value = PathprogressBar.Maximum;
            button1.Text = "Path Connect";
            m_pathGenerate.Enabled = true;

            m_collisionListPick.Enabled = true;
            m_txRobotName.Enabled = true;

        }

        private async Task rrtconnectCalOnGoing_Async(List<Tuple<Node3D, string>> node3D_startList, List<Tuple<Node3D, string>> node3D_endList)
        {

            try
            {
                
                for (int i = 0; i < node3D_startList.Count; i++)
                {
                    if (!rrtconnectCal_ongoing)
                    {
                        break;
                    }


                   List<point> path = new List<point>();
                   Node3D node3D_start = node3D_startList[i].Item1;
                   Node3D node3D_goal = node3D_endList[i].Item1;
                   if((node3D_startList[i].Item2!= "bypass")&& (node3D_endList[i].Item2 != "bypass")) iterate_Count = 0;
                    TxRobotRRTConnect rrt = new TxRobotRRTConnect();
                   await rrt.rrt_connect(new point(node3D_start.x, node3D_start.y, node3D_start.z, node3D_start.rx, node3D_start.ry, node3D_start.rz,ToolJointOpening), new point(node3D_goal.x, node3D_goal.y, node3D_goal.z, node3D_goal.rx, node3D_goal.ry, node3D_goal.rz, ToolJointOpening));
                    /*
                     * 如果迭代次数超过1000，而造成退出，则在退出的起始点和终止点增加一个中点，然后再进行计算
                     */
                   if (!TxRobotRRTConnect.currentpathdone) //表示当下的计算没有产生合适的路径而退出
                   {
                        iterate_Count++;
                        /*
                         * 当迭代1000次之后，需要取中间点，之前的算法是取start 点和end 点的中点;
                         * 改变策略，之前从start 点运动到end 点 找到干涉的姿态后，记录干涉的点；
                         * 以干涉点为起始点，通过generatePassThroughPoints()函数改变干涉嗲拿的姿态并加入到轨迹中进行运算
                        */
                        point p = new point(0, 0, 0, 0, 0, 0, 0);
                        point p_start, p_end;

                        bool crossClossionPoint=TxRobotRRTConnect.isValidforstepCorss(new point(node3D_start.x, node3D_start.y, node3D_start.z, node3D_start.rx, node3D_start.ry, node3D_start.rz, ToolJointOpening), new point(node3D_goal.x, node3D_goal.y, node3D_goal.z, node3D_goal.rx, node3D_goal.ry, node3D_goal.rz, ToolJointOpening),out p);


                        p_end = new point((node3D_goal.x+ node3D_start.x)/2,  (node3D_goal.y+ node3D_start.y)/2, (node3D_goal.z+ node3D_start.z) / 2,
                                 p.rx, p.ry, p.rz, ToolJointOpening);

                        crossClossionPoint = TxRobotRRTConnect.isValidforstepCorss(new point(node3D_goal.x, node3D_goal.y, node3D_goal.z, node3D_goal.rx, node3D_goal.ry, node3D_goal.rz, ToolJointOpening), new point(node3D_start.x, node3D_start.y, node3D_start.z, node3D_start.rx, node3D_start.ry, node3D_start.rz, ToolJointOpening),  out p);
                       

                        p_start = new point((node3D_goal.x + node3D_start.x) / 2, (node3D_goal.y + node3D_start.y) / 2, (node3D_goal.z + node3D_start.z) / 2,
                                 p.rx, p.ry, p.rz, ToolJointOpening);
                        if (!generatePassThroughPoints(ref p_start))
                        {
                            int index = 0;
                            string removedWeldName = "";
                            if (node3D_endList[i].Item2 == "bypass")
                            { 
                                if(i== node3D_startList.Count-1)
                                {
                                    node3D_endList.Remove(node3D_endList.Last());
                                    node3D_startList.Remove(node3D_startList.Last());

                                }
                                else
                                {
                                    node3D_endList.Remove(node3D_endList[i]);
                                    node3D_startList.Remove(node3D_startList[i + 1]);
                                }

                            }
                            else
                            {
                                removedWeldName = node3D_endList[i].Item2;
                                if (i == node3D_startList.Count - 1)
                                {
                                    node3D_endList.Remove(node3D_endList.Last());
                                    node3D_startList.Remove(node3D_startList.Last());
                                }
                                else
                                {
                                    node3D_endList.Remove(node3D_endList[i]);
                                    node3D_startList.Remove(node3D_startList[i + 1]);
                                }
                                TxTypeFilter opFilter = new TxTypeFilter(typeof(TxWeldLocationOperation));

                                TxObjectList allWeldPointsExist = weldTargetOperation.GetDirectDescendants(opFilter);
                                for (int j = 0; j < allWeldPointsExist.Count; j++)
                                {
                                    if (allWeldPointsExist[j].Name == removedWeldName)
                                    {
                                        index = j;
                                        break;
                                    }
                                }
                                TxApplication.ActiveDocument.OperationRoot.AddObject(allWeldPointsExist[index]);
                                progressbarCount = allWeldPointsExist.Count - 1;

                            }
                            i--;

                            continue;
                        }
                        if (!generatePassThroughPoints(ref p_end)) p_end = p_start;
                                                
                      Node3D p_start_Node = new Node3D(p_start.x, p_start.y, p_start.z, p_start.rx, p_start.ry, p_start.rz);
                      Node3D p_end_Node = new Node3D(p_end.x, p_end.y, p_end.z, p_end.rx, p_end.ry, p_end.rz);
                      node3D_startList.Insert(i + 1, Tuple.Create(p_start_Node,"bypass"));
                      node3D_startList.Insert(i + 2, Tuple.Create(p_end_Node, "bypass"));
                      node3D_endList.Insert(i, Tuple.Create(p_start_Node, "bypass"));
                      node3D_endList.Insert(i + 1, Tuple.Create(p_end_Node, "bypass"));
                      i--;
                      //progressbarCount = node3D_startList.Count;
                      continue;
                   }
                   path = rrt.path_points_start;
                   fullpath.Add(path);
                   TxRobotRRTConnect.currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
                   if (node3D_endList[i].Item2 != "bypass") progressbarNumber++;
                   PathprogressBar.Value = (int)((PathprogressBar.Maximum * ((double)progressbarNumber / progressbarCount)));
                   await Task.Delay(500);

                }

                rrtconnectCal_ongoing = false;
            }

            catch (Exception e)
            {
                throw;
            }



        }

        private void m_radioButtonX_CheckedChanged(object sender, EventArgs e)
        {
            spotagainstCollisionSrc = 'X';
        }

        private void m_radioButtonY_CheckedChanged(object sender, EventArgs e)
        {
            spotagainstCollisionSrc = 'Y';
        }

        private void m_radioButtonZ_CheckedChanged(object sender, EventArgs e)
        {
            spotagainstCollisionSrc = 'Z';
        }
    }
}
