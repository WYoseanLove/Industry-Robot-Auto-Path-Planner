using System;
using System.Threading;
using System.Windows;
using System.Windows.Forms;
using Tecnomatix.Engineering;


namespace rrtRobot
{
    public class TxrrtRobotPathPlanner : TxButtonCommand
    {
        public override string Category
        {

            get
            {
                return StringTable.Category;
            }
        }

        public override string Name
        {

            get
            {
                return StringTable.Name;
            }
        }

        public override void Execute(object cmdParams)
        {

            TxrrtRobotPathPlannerForm rob_Form = new TxrrtRobotPathPlannerForm();
           
            rob_Form.Show();
            rob_Form.Form_Setup();
        }
       
        public override string Bitmap
        {

            get
            {
                return StringTable.Bitmap;
            }
        }

        public override string LargeBitmap
        {

            get
            {
                return StringTable.LargeBitmap;
            }
        }
    }
}
