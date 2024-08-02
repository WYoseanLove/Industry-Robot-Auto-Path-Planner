using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using System.Numerics;
using System.Collections;
using Tecnomatix.Engineering.Ui;
using Tecnomatix.Engineering;
using System.Threading;

namespace rrtRobot
{
    public class RobotKinemetix
    {
        public static double M_PI = 3.1415926;
      
        public static void TxRobotPostureGenerate(TxRobot robot, ArrayList Solutions, double Gun_openning)
        {

            
            robot.CurrentPose = (TxPoseData)Solutions[0];

            
            ITxDevice tSelectObject = robot.MountedTools[0] as ITxDevice;

            ArrayList ToolJointValue = new ArrayList();

            ToolJointValue.Add(Gun_openning);
            TxPoseData txPoseData = tSelectObject.CurrentPose;

            txPoseData.JointValues = ToolJointValue;

            tSelectObject.CurrentPose=txPoseData;

        }

        /*
         * 下面的计算代码是通过TECNOMATIX自带的函数进行逆向学计算
         */
        public static ArrayList robotInverseCal(TxRobot robot, point target)
        {

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
            

           ArrayList solustions= robot.CalcInverseSolutions(txRobotInverseData);
          
           return solustions;

        }


        public static List<double> MatrixRotate(double rx1, double ry1, double rz1, double rx2, double ry2, double rz2, bool rotate_WorldorSelf)
        {
            List<double> result = new List<double>();

           
            TxTransformation rot1= new TxTransformation(new TxVector(rx1, ry1, rz1), TxTransformation.TxRotationType.RPY_XYZ);

           
            TxTransformation rot2 = new TxTransformation(new TxVector(rx2, ry2, rz2), TxTransformation.TxRotationType.RPY_XYZ);
            TxTransformation rot = new TxTransformation();
            if (rotate_WorldorSelf)
            {
                 rot=  TxTransformation.Multiply(rot1,rot2.Inverse);

            }
            else
                rot = TxTransformation.Multiply(rot2.Inverse,rot1);

           

            result.Add(rot.RotationRPY_XYZ.X);
            result.Add(rot.RotationRPY_XYZ.Y);
            result.Add(rot.RotationRPY_XYZ.Z);

            return result;
        }

        public static List<double> RotationVector(double RX, double RY, double RZ, double rx, double ry, double rz, bool rotate_WorldorSelf)
        {
            List<double> result = new List<double>();

            TxTransformation R_Original = new TxTransformation(new TxVector(RX, RY, RZ), TxTransformation.TxRotationType.RPY_XYZ);


            TxTransformation rotationMatrix = new TxTransformation(new TxVector(rx, ry, rz), TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation rot = new TxTransformation();
            if (rotate_WorldorSelf)
            {
                rot = TxTransformation.Multiply(rotationMatrix, R_Original);

            }
            else
                rot = TxTransformation.Multiply(R_Original, rotationMatrix);

           

            result.Add(rot.RotationRPY_XYZ.X);
            result.Add(rot.RotationRPY_XYZ.Y);
            result.Add(rot.RotationRPY_XYZ.Z);

            return result;

        }


        public static List<double> TranslateVec(double[] vecframe, double[] translate, bool worldorself)
        {
            List<double> result = new List<double>();

            TxTransformation R_VecFrame = new TxTransformation(new TxVector(vecframe[3], vecframe[4], vecframe[5]), // for translation
                new TxVector(vecframe[0], vecframe[1], vecframe[2]), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);


            TxTransformation R_translate = new TxTransformation(new TxVector(translate[0], translate[1], translate[2]), // for translation
               new TxVector(0, 0, 0), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);
            TxTransformation rot = new TxTransformation();
            if (worldorself)
            {
                rot = TxTransformation.Multiply(R_translate, R_VecFrame);

            }
            else
                rot = TxTransformation.Multiply(R_VecFrame, R_translate);


            result.Add(rot.Translation.X);
            result.Add(rot.Translation.Y);
            result.Add(rot.Translation.Z);

            return result;
        }

        public static List<double> PointtoLineDistance(point p,point p_robot)
        {

            TxTransformation tx_p_robot = new TxTransformation(new TxVector(p_robot.x, p_robot.y, p_robot.z), // for translation
                new TxVector(p_robot.rx, p_robot.ry, p_robot.rz), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_World = new TxTransformation(new TxVector(0, 0,0), // for translation
                new TxVector(0, 0, 0), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_P = new TxTransformation(new TxVector(p.x, p.y, p.z), // for translation
               new TxVector(p.rx, p.ry, p.rz), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);

            TxApplication.ActiveDocument.WorkingFrame = tx_P;


            TxTransformation tx_Project = new TxTransformation();


            tx_Project= tx_p_robot.LocationRelativeToWorkingFrame;

            double x = tx_Project.Translation.X;
            double y = tx_Project.Translation.Y;
            double z = tx_Project.Translation.Z;

            TxApplication.ActiveDocument.WorkingFrame = tx_World;

            List<double> result = new List<double>() { Math.Sqrt(z * z + y * y), Math.Sqrt(x * x + z * z), Math.Sqrt(x*x+y*y) };


            return result;
        }

        public static List<double> spotagainstCollisonSrc(point p, point p_robot, char robDir)
        {
            TxTransformation tx_p_robot = new TxTransformation(new TxVector(p_robot.x, p_robot.y, p_robot.z), // for translation
               new TxVector(p_robot.rx, p_robot.ry, p_robot.rz), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_World = new TxTransformation(new TxVector(0, 0, 0), // for translation
                new TxVector(0, 0, 0), // for rotation
                TxTransformation.TxRotationType.RPY_XYZ);

            TxTransformation tx_P = new TxTransformation(new TxVector(p.x, p.y, p.z), // for translation
               new TxVector(p.rx, p.ry, p.rz), // for rotation
               TxTransformation.TxRotationType.RPY_XYZ);

            TxApplication.ActiveDocument.WorkingFrame = tx_p_robot;

            TxTransformation tx_Project = new TxTransformation();


            tx_Project = tx_P.LocationRelativeToWorkingFrame;

            double sinXr = 0;
            double sinYr = 0;
            double sinZr = 0;

            if (robDir=='X')
            {

                sinXr = Math.Sqrt(1-tx_Project[0, 0]* tx_Project[0, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[0, 1] * tx_Project[0, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[0, 2] * tx_Project[0, 2]);

            }

            if (robDir == 'Y')
            {
                sinXr = Math.Sqrt(1 - tx_Project[1, 0] * tx_Project[1, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[1, 1] * tx_Project[1, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[1, 2] * tx_Project[1, 2]);


            }

            if (robDir == 'Z')
            {
                sinXr = Math.Sqrt(1 - tx_Project[2, 0] * tx_Project[2, 0]);
                sinYr = Math.Sqrt(1 - tx_Project[2, 1] * tx_Project[2, 1]);
                sinZr = Math.Sqrt(1 - tx_Project[2, 2] * tx_Project[2, 2]);



            }


            List<double> result = new List<double>() ;

            result.Add(sinXr);
            result.Add(sinYr);
            result.Add(sinZr);

            TxApplication.ActiveDocument.WorkingFrame = tx_World;
            return result;


        }
    
    
    
    }
}
