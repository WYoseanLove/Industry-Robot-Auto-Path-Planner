using System;
using System.Windows.Forms.VisualStyles;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Ui;

namespace rrtRobot
{
    partial class TxrrtRobotPathPlannerForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        public void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(TxrrtRobotPathPlannerForm));
            this.Collision_Src = new System.Windows.Forms.Label();
            this.Group_Collision = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.m_collisionListPick = new Tecnomatix.Engineering.Ui.TxObjComboBoxCtrl();
            this.label2 = new System.Windows.Forms.Label();
            this.m_pathGenerate = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.PathprogressBar = new System.Windows.Forms.ProgressBar();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.textLogfile = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.m_GroupTargetOpPick = new Tecnomatix.Engineering.Ui.TxObjEditBoxCtrl();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.m_spotDirec = new System.Windows.Forms.Label();
            this.Group_Collision.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            // 
            // Collision_Src
            // 
            resources.ApplyResources(this.Collision_Src, "Collision_Src");
            this.Collision_Src.Name = "Collision_Src";
            // 
            // Group_Collision
            // 
            this.Group_Collision.Controls.Add(this.label4);
            this.Group_Collision.Controls.Add(this.m_collisionListPick);
            this.Group_Collision.Controls.Add(this.label2);
            this.Group_Collision.Controls.Add(this.Collision_Src);
            resources.ApplyResources(this.Group_Collision, "Group_Collision");
            this.Group_Collision.Name = "Group_Collision";
            this.Group_Collision.TabStop = false;
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // m_collisionListPick
            // 
            this.m_collisionListPick.DropDownHeight = 106;
            this.m_collisionListPick.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDown;
            resources.ApplyResources(this.m_collisionListPick, "m_collisionListPick");
            this.m_collisionListPick.ListenToPick = true;
            this.m_collisionListPick.Name = "m_collisionListPick";
            this.m_collisionListPick.Object = null;
            this.m_collisionListPick.PickLevel = Tecnomatix.Engineering.Ui.TxPickLevel.Component;
            this.m_collisionListPick.ValidatorType = Tecnomatix.Engineering.Ui.TxValidatorType.AnyLocatableObject;
            this.m_collisionListPick.Picked += new Tecnomatix.Engineering.Ui.TxObjComboBoxCtrl_PickedEventHandler(this.m_collisionListPicked);
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // m_pathGenerate
            // 
            resources.ApplyResources(this.m_pathGenerate, "m_pathGenerate");
            this.m_pathGenerate.Name = "m_pathGenerate";
            this.m_pathGenerate.UseVisualStyleBackColor = true;
            this.m_pathGenerate.Click += new System.EventHandler(this.m_pathGenerate_Click);
            // 
            // button1
            // 
            resources.ApplyResources(this.button1, "button1");
            this.button1.Name = "button1";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // PathprogressBar
            // 
            resources.ApplyResources(this.PathprogressBar, "PathprogressBar");
            this.PathprogressBar.Name = "PathprogressBar";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.textLogfile);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.PathprogressBar);
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // textLogfile
            // 
            this.textLogfile.AcceptsReturn = true;
            this.textLogfile.BackColor = System.Drawing.SystemColors.ButtonHighlight;
            this.textLogfile.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            resources.ApplyResources(this.textLogfile, "textLogfile");
            this.textLogfile.Name = "textLogfile";
            this.textLogfile.ReadOnly = true;
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.m_GroupTargetOpPick);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label5);
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // m_GroupTargetOpPick
            // 
            this.m_GroupTargetOpPick.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.m_GroupTargetOpPick.KeepFaceEmphasizedWhenControlIsNotFocused = true;
            this.m_GroupTargetOpPick.ListenToPick = true;
            resources.ApplyResources(this.m_GroupTargetOpPick, "m_GroupTargetOpPick");
            this.m_GroupTargetOpPick.Name = "m_GroupTargetOpPick";
            this.m_GroupTargetOpPick.Object = null;
            this.m_GroupTargetOpPick.PickAndClear = false;
            this.m_GroupTargetOpPick.PickLevel = Tecnomatix.Engineering.Ui.TxPickLevel.Component;
            this.m_GroupTargetOpPick.PickOnly = false;
            this.m_GroupTargetOpPick.ReadOnly = false;
            this.m_GroupTargetOpPick.ValidatorType = Tecnomatix.Engineering.Ui.TxValidatorType.Operation;
            this.m_GroupTargetOpPick.Picked += new Tecnomatix.Engineering.Ui.TxObjEditBoxCtrl_PickedEventHandler(this.txTargetGroupOpEditBoxCtrl_Picked);
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            // 
            // label5
            // 
            resources.ApplyResources(this.label5, "label5");
            this.label5.Name = "label5";
            // 
            // m_spotDirec
            // 
            resources.ApplyResources(this.m_spotDirec, "m_spotDirec");
            this.m_spotDirec.Name = "m_spotDirec";
            // 
            // TxrrtRobotPathPlannerForm
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_pathGenerate);
            this.Controls.Add(this.Group_Collision);
            this.Controls.Add(this.groupBox1);
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "TxrrtRobotPathPlannerForm";
            this.ShouldCloseOnDocumentUnloading = true;
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.TxrrtRobotPathPlannerForm_FormClosed);
            this.Group_Collision.ResumeLayout(false);
            this.Group_Collision.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

       

        #endregion
       
        private System.Windows.Forms.Label Collision_Src;
        private System.Windows.Forms.GroupBox Group_Collision;
        private System.Windows.Forms.Button m_pathGenerate;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.ProgressBar PathprogressBar;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private TxObjComboBoxCtrl m_collisionListPick;
        
        public System.Windows.Forms.Label m_spotDirec;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private TxObjEditBoxCtrl m_GroupTargetOpPick;
        private System.Windows.Forms.TextBox textLogfile;
    }
}