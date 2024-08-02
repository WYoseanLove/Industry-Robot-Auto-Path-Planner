using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Tecnomatix.Engineering;
using static System.Net.Mime.MediaTypeNames;


namespace rrtRobot
{
    public struct point
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double rx { get; set; }
        public double ry { get; set; }
        public double rz { get; set; }

        public double Gun_Open { get; set; }

        public point(double X, double Y, double Z,double RX, double RY, double RZ,double Gun_Opening)
        {
            x = X;
            y = Y;
            z = Z;

            rx = RX;
            ry = RY;
            rz = RZ;

            Gun_Open = Gun_Opening;
        }

    };
    public class Node3D_star
    {
        public point loc;
        public double cost;
        public Node3D_star parent;

       
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


    public partial class TxRobotRRTConnect : TxrrtRobotPathPlannerForm
    { 
        private int connected = 0;
        private int state = 1;
        private int sub_state = 0;
        private double Start_step_size = 10.0;
        private double End_step_size = 10.0;
        private double circle_radius_1 = 20.0;
        private List<double> rotateStepSizefromStarttoEnd = new List<double>();
        private List<double> rotateStepSizefromEndtoStart = new List<double>();
        private int IterationCounts = 0;//记录rrtconnect 的迭代次数；

        public List<point> path_points_start = new List<point>(500);
        private int pathcount_start = 0;
        private List<point> path_points_end = new List<point>(500);
        private int pathcount_end = 0;

        private List<Node3D_star> start_nodes = new List<Node3D_star>(10000);
        private int nodecount_start = 0;
        private List<Node3D_star> end_nodes = new List<Node3D_star>(10000);
        private int nodecount_end = 0;

        private Random rd;

        // 用于记录连续失效点的数量来决定stepsize的大小
        private int failCount = 0;

        public static bool currentpathdone = false;

        public static void LogInformation(point p, List<double> distoAix)
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);
            string loc = "";
            List<double> temdistoAix = new List<double>() { distoAix[0], distoAix[1], distoAix[2] };
            temdistoAix.Sort(); // 将数值进行排序
           int  indexMin = distoAix.IndexOf(temdistoAix[0]);

           int  indexSecondMin = distoAix.IndexOf(temdistoAix[1]);


            if (indexMin == 0) loc = "X";
            else if (indexMin == 1) loc = "Y";
            else loc = "Z";

            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() + " " +"bypass points" + ": " + p.x.ToString() + " "
            + p.y.ToString() + " "
            + p.z.ToString() + " "
            + (p.rx * 180 / M_PI).ToString() + " "
            + (p.ry * 180 / M_PI).ToString() + " "
            + (p.rz * 180 / M_PI).ToString() + " "
            + (p.Gun_Open).ToString() + " "
            + "Moved direction:  "+loc);

            sw.Close();
        }
        private void logpathGenerateOK()
        {
            StreamWriter sw = new StreamWriter(TxrrtRobotPathPlannerForm.LogfilePath, true);
            sw.WriteLine(DateTime.Now.ToLocalTime().ToString() +" Path Generated!");
            sw.Close();

        }


        public static double dist(point p1, point p2)  // To calculate the distance between two points
        {

            return Math.Sqrt(Math.Pow(p2.x - p1.x, 2) + Math.Pow(p2.y - p1.y, 2) + Math.Pow(p2.z - p1.z, 2));
        }
        public int Nearest_Node(Node3D_star rand)
        {
            double min =Double.MaxValue;
            int index = -1;

            if (state == 1)
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
       
        public point step_func(point near, point rand, double size_step,bool start2end)
        {

            /*
            * step_fun的作用是根据获的startNode 或者EndNode 里面的最近的点与随机生成的rand点来生成step点，并根据isvalid函数来判断这个点是否可以增加到startNode或者endNode里面
            * Loc里面的x/y/z位置坐标根据根据距离生成, 其中的dx/d,dy/d,dz/d的含义是从各个方向的分量占总距离的配比；
            * rx/ry/rz是机器人轨迹点中定义方向向量的，这里的生成方式是：
            * 1). rotateStepSizefromStarttoEnd[]数组中存在的三个维度的数值，依次对应X/Y/Z，是指start 到end 点的姿态，需要相对于自身坐标系旋转的角度
            *     [2]指的是Z轴首先绕着自身的坐标系Z轴旋转的角度，[1]Z轴旋转完成之后Y轴旋转的角度，[0]Y轴旋转完成之后X轴的旋转角度；
            * 2). 将每个旋转角度除以10作为单步的旋转，但注意，必须按照Z-Y-X的旋转顺序，否则无法到达end点的姿态；所以按照start_nodes.Count 进行计算旋转的度数；
            * 3). 在每个轴的旋转过程中会加入相应的rand数值，根据经验Z轴旋转的概率较大，所以定义的范围为-90~90度，其他轴为-10~10度，这里是根据实验的结果改变的
            *     当然可以更小，这样生成的轨迹会更加平稳，但是运算的时间就更长，还有可能无法计算出合适的轨迹
            *     当然也可以更大，这样运算的时间可能缩短，但轨迹的波动很大；
            *     这里是轨迹算法优化的一个点；
            * 4). 轨迹点还有一个参数就是gun的打开角度，这里直接调用rand点里面的open角度
            */
            double dx = rand.x - near.x;
            double dy = rand.y - near.y;
            double dz = rand.z - near.z;
            double d = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2) + Math.Pow(dz, 2));

            //获取从near点到rand点的各个方向总角度的转动值；
            double rx_rand = -Math.PI / 18 + rd.NextDouble() * (2 * Math.PI  / 18);
            double ry_rand = -Math.PI / 18 + rd.NextDouble() * (2 * Math.PI  / 18);
            double rz_rand = -Math.PI / 4 + rd.NextDouble() * (2 * Math.PI / 4);

            point step;
            List<double> resultendVEC = new List<double>();
            if (start2end) // 表示从start 点向end点转动移动
            {
               
                if (start_nodes.Count > 30)
                    resultendVEC= getLocVectorafterRotateDeg(near, new List<double> { rx_rand, ry_rand, rz_rand });

                if (start_nodes.Count <= 10)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rx_rand, ry_rand, rotateStepSizefromStarttoEnd[2]/10+ rz_rand });

                if (start_nodes.Count > 10 && start_nodes.Count <= 20)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rx_rand,  rotateStepSizefromStarttoEnd[1] / 10+ ry_rand, rz_rand });
                if (start_nodes.Count > 20 && start_nodes.Count <= 30)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> {  rotateStepSizefromStarttoEnd[0] / 10+ rx_rand, ry_rand, rz_rand });

            }

            else
            {
                if (end_nodes.Count > 30)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rx_rand, ry_rand, rz_rand });

                if (end_nodes.Count <= 10)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rx_rand, ry_rand, rotateStepSizefromEndtoStart[2] / 10+ rz_rand });

                if (end_nodes.Count > 10 && end_nodes.Count <= 20)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rx_rand, rotateStepSizefromEndtoStart[1] / 10+ ry_rand, rz_rand });
                if (end_nodes.Count > 20 && end_nodes.Count <= 30)
                    resultendVEC = getLocVectorafterRotateDeg(near, new List<double> { rotateStepSizefromEndtoStart[0] / 10+ rx_rand, ry_rand, rz_rand });

            }

            step = new point(near.x + (size_step) * (dx / d)
               , near.y + (size_step) * (dy / d),
               near.z + (size_step) * (dz / d),
                resultendVEC[0],
                resultendVEC[1],
                resultendVEC[2],
                rand.Gun_Open);

            return step;
        }
        public bool isValid(ref point step, point near)
        {
            /*
             * 首先判断step位置点是否和障碍物干涉，如果干涉则执行单点的旋转，看是否有一个方向可以避免干涉;
             * 如果step点不与障碍物干涉，则将near到step点的位置及角度分成linediv份，分别判断是否与障碍物干涉
             * 如果有一个干涉，则判断机器人从near 到step是穿越障碍物的，返回为false;
             */
            //List<point> truePath = new List<point>();
           point p = new point(0, 0, 0, 0, 0, 0, 0);
            if (!collisioncheckforSinglePoint(step))
                    return false;

            if (!isValidforstepCorss(step,near,out p))
                return false;
         
            return true;
        }

        public static bool isValidforstepCorss(point step, point near,out point p)  // 将从step 到near中间的干涉点找出并返回
        {
            int LineDiv = 72; //分成72段，每个端点检测碰撞；

            for (int j = 1; j <= LineDiv; j++)
            {
                List<double> result_near2step = getfromStart2endRotateDgree(near, step);

                result_near2step[0] *= (double)j / LineDiv;
                result_near2step[1] *= (double)j / LineDiv;
                result_near2step[2] *= (double)j / LineDiv;


                List<double> resultendVEC = getLocVectorafterRotateDeg(near, result_near2step);


                 p = new point(near.x + (step.x - near.x) * j / LineDiv,
                    near.y + (step.y - near.y) * j / LineDiv,
                    near.z + (step.z - near.z) * j / LineDiv,
                    resultendVEC[0],
                    resultendVEC[1],
                    resultendVEC[2],
                    step.Gun_Open);

                if (!collisioncheckforSinglePoint(p))
                {
                   
                    Node3D_star temp = new Node3D_star();
                    temp.loc = p;
                    temp.cost = 0;
                    temp.parent = null;
                    return false;
                }


            }
            p = new point(0, 0, 0, 0, 0, 0, 0);
            return true;
        }
       
        public static bool collisioncheckforSinglePoint(point step)
        {
            ArrayList Solutions = RobotKinemetix.robotInverseCal(robot, step);

            if (Solutions.Count == 0) return false;        
            RobotKinemetix.TxRobotPostureGenerate(TxrrtRobotPathPlannerForm.robot, Solutions, step.Gun_Open);
           
            if (Collision_Check(cd, queryParams, root, collisionSrc, collisionTar, 5.0))return true;
            return false;

        }

        public void minimal_cost(Node3D_star step)
        {

            double new_cost;
            double min_cost = step.cost;
            int index = -1;

            if (state == 1)
            {
                circle_radius_1 = 2 * Start_step_size;
                for (int i = 0; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_1 && isValid(ref step.loc, start_nodes[i].loc))
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
                circle_radius_1 = 2 * End_step_size;
                for (int i = 0; i < nodecount_end; i++)
                {
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_1 && isValid(ref step.loc, end_nodes[i].loc))
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

        public void path_Points(int index_1, int index_2) // final path points will be keep at path_points_start and path_points_end List
        {
            pathcount_start = 0;
            pathcount_end = 0;
            Node3D_star n1, n2;
            float d = 20.0f;

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
                if (dist(n1.loc, path_points_end[pathcount_end - 1]) < d/4)
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
        
       
        private static List<double> getfromStart2endRotateDgree(point p_start, point p_end) //用于获得从start的到end的旋转角度值
        {
        /*
        * 下面的接口函数Matrix_Rotate 得到的result 为三个弧度数值
        * 第一个参数为start 坐标系绕着基坐标方向X周旋转的角度；
        * 第二个参数为start 坐标系绕着基坐标方向Y周旋转的角度；
        * 第二个参数为start 坐标系绕着基坐标方向Z周旋转的角度；
        * 旋转完成之后，两个坐标系start 与end的方向是一致的
        * 必须按照先绕X轴，然后绕Y轴，最后绕Z轴的旋转顺序进行
        */

            List<double> result_start2end = new List<double>();
            result_start2end = RobotKinemetix.MatrixRotate(p_end.rx, p_end.ry, p_end.rz, p_start.rx, p_start.ry, p_start.rz, false);

            return result_start2end;
        }
        private static List<double> getLocVectorafterRotateDeg(point p_start, List<double>rotateDeg)// 位置点经过旋转之后得到的新位置点的旋转角度值
        {
            List<double> result_endVec = new List<double>();
            
            result_endVec = RobotKinemetix.RotationVector(p_start.rx, p_start.ry, p_start.rz,
                  rotateDeg[0], rotateDeg[1], rotateDeg[2], false);
            return result_endVec;
        }
        public static List<double> getVectorfromTranslate(point p, List<double> translate)
        {
            double[] vecFrame = new double[6] { p.rx, p.ry, p.rz, p.x, p.y, p.z };
            double[] vecTranslate = new double[3] { translate[0], translate[1] , translate[2] };
            List<double> result_endVec = new List<double>();
            result_endVec = RobotKinemetix.TranslateVec(vecFrame, vecTranslate,false);
            return result_endVec;
        }
       
        public async Task rrt_connect(point p_start, point p_end)
        {
            connected = 0;
            state = 1;
            sub_state = 0;
            nodecount_start = 0;        
            nodecount_end = 0;         

            Node3D_star start_node = new Node3D_star();
            Node3D_star end_node = new Node3D_star();
            Node3D_star rand_node = new Node3D_star();
            int index;


            Node3D_star step_node;
            Node3D_star sub_step_node;

            rd = new Random(unchecked((int)DateTime.Now.Ticks));

            start_node.loc = new point(p_start.x, p_start.y, p_start.z, p_start.rx, p_start.ry, p_start.rz, ToolJointOpening);

            start_node.parent = new Node3D_star();
            start_node.cost = 0;
            end_node.loc = new point(p_end.x, p_end.y, p_end.z,p_end.rx,p_end.ry,p_end.rz, ToolJointOpening);

            end_node.parent = new Node3D_star();
            end_node.cost = 0;

            start_nodes.Add(start_node);

            nodecount_start++;
           
            end_nodes.Add(end_node);
            nodecount_end++;
            currentpathdone = false;

            //获取从 start 到end 的旋转角度，绕坐标系自身旋转，旋转顺序依次为ZYX;

            rotateStepSizefromStarttoEnd = getfromStart2endRotateDgree(p_start, p_end);
            //获取从 end 到start 的旋转角度，绕坐标系自身旋转，旋转顺序依次为ZYX;

            rotateStepSizefromEndtoStart = getfromStart2endRotateDgree(p_end, p_start);

            // 确认p_start 和p_end 点处，在现有的gun_open的状态下是没有干涉的，如果有干涉则减小gun_open的数值到不干涉的状态
            double gun_open_step = p_start.Gun_Open / 30;

            while (!collisioncheckforSinglePoint(p_start))
            {
                p_start.Gun_Open -= gun_open_step;
                if ((Math.Abs(p_start.Gun_Open) <= Math.Abs(gun_open_step))|| (p_start.Gun_Open* gun_open_step<0))
                {
                    return;
                }
            }
            
            start_node.loc.Gun_Open = p_start.Gun_Open;

            gun_open_step = p_end.Gun_Open / 30;

            while (!collisioncheckforSinglePoint(p_end))
            {
                p_end.Gun_Open -= gun_open_step;
                if(Math.Abs(p_end.Gun_Open)<= Math.Abs(gun_open_step) || (p_end.Gun_Open * gun_open_step < 0))
                {
                    return;
                }
            }
            
            end_node.loc.Gun_Open = p_end.Gun_Open;


            int threshold = 100;
            while (connected != 1)
            {

                if (!rrtconnectCal_ongoing)
                {
                    break;
                }

                sub_state = 0;
                IterationCounts++;
                if ((IterationCounts / threshold) == 10) return; //如果迭代次数超过1000次则退出
                else if((IterationCounts % threshold)==0)//每迭代100次则步长距离增加100；
                {
                    Start_step_size += 100;
                    End_step_size += 100;
                }
                
                
                if (state == 1)
                {
                    double rand_node_gun_open= 0;
                    int gun_Open_rd_Limint = (int)(Math.Abs(start_node.loc.Gun_Open) / Math.Abs(gun_open_step));
                    if (iterate_Count >= 8) gun_Open_rd_Limint *= 2;
                    rand_node_gun_open = start_node.loc.Gun_Open - rd.Next(0, gun_Open_rd_Limint/2) * gun_open_step;
                   
                    rand_node.loc = new point(rd.Next((int)(start_nodes[0].loc.x-1000), (int)(start_nodes[0].loc.x + 1000)), 
                        rd.Next((int)(start_nodes[0].loc.y - 1000), (int)(start_nodes[0].loc.y + 1000)), 
                        rd.Next((int)(start_nodes[0].loc.z - 1000), (int)(start_nodes[0].loc.z + 1000)),
                        p_start.rx, p_start.ry, p_start.rz, rand_node_gun_open);
                    index = Nearest_Node(rand_node);
                    if (index < 0)
                    {
                        continue;
                    }
                    if (dist(start_nodes[index].loc, rand_node.loc) < Start_step_size)
                    {
                       
                        continue;
                    }
                    else
                    {
                        step_node = new Node3D_star();
                        (step_node.loc) = step_func(start_nodes[index].loc, rand_node.loc, Start_step_size,true);

                    }
                    if (isValid(ref step_node.loc, start_nodes[index].loc) == false)
                    {

                        failCount++;
                        continue;
                    }
                    else
                    {
                        failCount = 0;
                        step_node.parent = start_nodes[index];
                        step_node.cost = start_nodes[index].cost + Start_step_size;
                        minimal_cost(step_node);
                        start_nodes.Add(step_node);
                        nodecount_start++;
                    }


                    state = 2;
                    int end_substate = 0;

                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing)
                        {
                            break;
                        }

                        index = Nearest_Node(step_node);
                        if (index < 0) continue;
                        if (isValid(ref step_node.loc, end_nodes[index].loc)||(!rrtconnectCal_ongoing))
                        {

                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");

                            path_Points(nodecount_start, index + 1);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_star();
                            (sub_step_node.loc) = step_func(end_nodes[index].loc, step_node.loc, End_step_size,false);
                        }
                       
                        if (dist(sub_step_node.loc, end_nodes[end_nodes.Count - 1].loc) <= 0.01)
                        {
                            sub_step_node.loc.x = (end_nodes[index].loc.x + step_node.loc.x) / 2;
                            sub_step_node.loc.y = (end_nodes[index].loc.y + step_node.loc.y) / 2;
                            sub_step_node.loc.z = (end_nodes[index].loc.z + step_node.loc.z) / 2;
                        }
                       
                        if (isValid(ref sub_step_node.loc, end_nodes[index].loc) == false)
                        {

                            sub_state = 1;                   
                            failCount++;
                            continue;
                        }
                        else
                        {
                            failCount = 0;
                            sub_step_node.parent = end_nodes[index];
                            sub_step_node.cost = end_nodes[index].cost + End_step_size;
                            minimal_cost(sub_step_node);
                           

                            end_nodes.Add(sub_step_node);
                            nodecount_end++;
                            end_substate++;
                            if(end_substate>5)
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
                    int gun_Open_rd_Limint = (int)(Math.Abs(end_node.loc.Gun_Open) / Math.Abs(gun_open_step));
                    if (iterate_Count == 8) gun_Open_rd_Limint *= 2;
                    rand_node_gun_open = end_node.loc.Gun_Open - rd.Next(0, gun_Open_rd_Limint/2) * gun_open_step;
                    rand_node.loc = new point(rd.Next((int)(end_nodes[0].loc.x - 1000), (int)(end_nodes[0].loc.x + 1000)), 
                        rd.Next((int)(end_nodes[0].loc.y - 1000), (int)(end_nodes[0].loc.y + 1000)),
                        rd.Next((int)(end_nodes[0].loc.z - 1000), (int)(end_nodes[0].loc.z + 1000)),
                         p_end.rx, p_end.ry, p_end.rz, rand_node_gun_open);
                    index = Nearest_Node(rand_node);
                   
                    if (index < 0) continue;
                    
                    if (dist(end_nodes[index].loc, rand_node.loc) < End_step_size) continue;
                    else
                    {
                        step_node = new Node3D_star();
                        (step_node.loc) = step_func(end_nodes[index].loc, rand_node.loc, End_step_size,false);
                    }

                    if (isValid(ref step_node.loc, end_nodes[index].loc) == false)
                    {                       
                        failCount++;
                        continue;
                    }
                    else
                    {
                        failCount = 0;
                        step_node.parent = end_nodes[index];
                        step_node.cost = end_nodes[index].cost + End_step_size;
                        minimal_cost(step_node);
                        end_nodes.Add(step_node);
                        nodecount_end++;
                        
                    }

                    state = 1;
                    int start_substate = 0;
                    while (sub_state != 1)
                    {
                        if (!rrtconnectCal_ongoing) break;
                       
                        index = Nearest_Node(step_node);
                        if (index < 0) continue;
                        if (isValid(ref step_node.loc, start_nodes[index].loc) || (!rrtconnectCal_ongoing))
                        {
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                            path_Points(index + 1, nodecount_end);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_star();
                            (sub_step_node.loc) = step_func(start_nodes[index].loc, step_node.loc, Start_step_size,true);
                        }
                        /*
                            */
                        if (dist(sub_step_node.loc, start_nodes[start_nodes.Count - 1].loc) <= 0.01)
                        {
                            sub_step_node.loc.x = (start_nodes[index].loc.x + step_node.loc.x) / 2;
                            sub_step_node.loc.y = (start_nodes[index].loc.y + step_node.loc.y) / 2;
                            sub_step_node.loc.z = (start_nodes[index].loc.z + step_node.loc.z) / 2;
                        }
                        /*
                        */
                        if (isValid(ref sub_step_node.loc, start_nodes[index].loc) == false)
                        {
                            sub_state = 1;
                            failCount++;
                            continue;
                        }
                        else
                        {
                            failCount = 0;
                            start_substate++;
                            sub_step_node.parent = start_nodes[index];
                            sub_step_node.cost = start_nodes[index].cost + Start_step_size;
                            minimal_cost(sub_step_node);
                            start_nodes.Add(sub_step_node);
                            nodecount_start++;
                            if (start_substate > 5)
                            {
                                sub_state = 1;
                                start_substate = 0;
                            }

                        }
                       
                    }

                }

                await Task.Delay(50);

            }
            
            path_points_start.Reverse();

            path_points_start.AddRange(path_points_end);
            logpathGenerateOK();
            currentpathdone = true;// 记录当前的轨迹已经计算结束，无论是正常结束还是手动结束
        }

       
    }
}
