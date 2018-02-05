namespace SR6009_Concentrator_Tools.FunList
{
    partial class ReadRealData
    {
        /// <summary> 
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region 组件设计器生成的代码
        
        /// <summary> 
        /// 设计器支持所需的方法 - 不要
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.cmbDataFormat = new System.Windows.Forms.ComboBox();
            this.label1 = new System.Windows.Forms.Label();
            this.cbAdditionInfo = new System.Windows.Forms.CheckBox();
            this.dtpTime = new System.Windows.Forms.DateTimePicker();
            this.dtpDate = new System.Windows.Forms.DateTimePicker();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.cmbTimeType = new System.Windows.Forms.ComboBox();
            this.gpAdditon = new System.Windows.Forms.GroupBox();
            this.nudMeterCount = new System.Windows.Forms.NumericUpDown();
            this.label4 = new System.Windows.Forms.Label();
            this.nudTimeSlot = new System.Windows.Forms.NumericUpDown();
            this.FunListTimer = new System.Windows.Forms.Timer(this.components);
            this.gpAdditon.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudMeterCount)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudTimeSlot)).BeginInit();
            this.SuspendLayout();
            // 
            // cmbDataFormat
            // 
            this.cmbDataFormat.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbDataFormat.FormattingEnabled = true;
            this.cmbDataFormat.Items.AddRange(new object[] {
            "实时数据",
            "冻结数据"});
            this.cmbDataFormat.Location = new System.Drawing.Point(71, 8);
            this.cmbDataFormat.Name = "cmbDataFormat";
            this.cmbDataFormat.Size = new System.Drawing.Size(121, 21);
            this.cmbDataFormat.TabIndex = 7;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(11, 12);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(55, 13);
            this.label1.TabIndex = 8;
            this.label1.Text = "数据格式";
            // 
            // cbAdditionInfo
            // 
            this.cbAdditionInfo.AutoSize = true;
            this.cbAdditionInfo.Location = new System.Drawing.Point(35, 48);
            this.cbAdditionInfo.Name = "cbAdditionInfo";
            this.cbAdditionInfo.Size = new System.Drawing.Size(74, 17);
            this.cbAdditionInfo.TabIndex = 9;
            this.cbAdditionInfo.Text = "附带信息";
            this.cbAdditionInfo.UseVisualStyleBackColor = true;
            this.cbAdditionInfo.CheckedChanged += new System.EventHandler(this.cbAdditionInfo_CheckedChanged);
            // 
            // dtpTime
            // 
            this.dtpTime.Cursor = System.Windows.Forms.Cursors.Default;
            this.dtpTime.CustomFormat = " HH:mm:ss";
            this.dtpTime.Enabled = false;
            this.dtpTime.Format = System.Windows.Forms.DateTimePickerFormat.Custom;
            this.dtpTime.Location = new System.Drawing.Point(174, 54);
            this.dtpTime.MaxDate = new System.DateTime(2099, 12, 31, 0, 0, 0, 0);
            this.dtpTime.MinDate = new System.DateTime(2000, 1, 1, 0, 0, 0, 0);
            this.dtpTime.Name = "dtpTime";
            this.dtpTime.ShowUpDown = true;
            this.dtpTime.Size = new System.Drawing.Size(75, 20);
            this.dtpTime.TabIndex = 20;
            // 
            // dtpDate
            // 
            this.dtpDate.Cursor = System.Windows.Forms.Cursors.Default;
            this.dtpDate.CustomFormat = "yyyy-MM-dd";
            this.dtpDate.Enabled = false;
            this.dtpDate.Format = System.Windows.Forms.DateTimePickerFormat.Custom;
            this.dtpDate.Location = new System.Drawing.Point(83, 54);
            this.dtpDate.MaxDate = new System.DateTime(2099, 12, 31, 0, 0, 0, 0);
            this.dtpDate.MinDate = new System.DateTime(2000, 1, 1, 0, 0, 0, 0);
            this.dtpDate.Name = "dtpDate";
            this.dtpDate.Size = new System.Drawing.Size(86, 20);
            this.dtpDate.TabIndex = 19;
            this.dtpDate.Value = new System.DateTime(2016, 9, 22, 0, 0, 0, 0);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(20, 88);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(55, 13);
            this.label2.TabIndex = 21;
            this.label2.Text = "时隙序号";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(20, 116);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(55, 13);
            this.label3.TabIndex = 22;
            this.label3.Text = "表具数量";
            // 
            // cmbTimeType
            // 
            this.cmbTimeType.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbTimeType.FormattingEnabled = true;
            this.cmbTimeType.Items.AddRange(new object[] {
            "当前时间",
            "自定义时间"});
            this.cmbTimeType.Location = new System.Drawing.Point(83, 26);
            this.cmbTimeType.Name = "cmbTimeType";
            this.cmbTimeType.Size = new System.Drawing.Size(121, 21);
            this.cmbTimeType.TabIndex = 25;
            this.cmbTimeType.SelectedIndexChanged += new System.EventHandler(this.cmbTimeType_SelectedIndexChanged);
            // 
            // gpAdditon
            // 
            this.gpAdditon.Controls.Add(this.nudMeterCount);
            this.gpAdditon.Controls.Add(this.label4);
            this.gpAdditon.Controls.Add(this.nudTimeSlot);
            this.gpAdditon.Controls.Add(this.cmbTimeType);
            this.gpAdditon.Controls.Add(this.dtpDate);
            this.gpAdditon.Controls.Add(this.dtpTime);
            this.gpAdditon.Controls.Add(this.label2);
            this.gpAdditon.Controls.Add(this.label3);
            this.gpAdditon.Location = new System.Drawing.Point(13, 50);
            this.gpAdditon.Name = "gpAdditon";
            this.gpAdditon.Size = new System.Drawing.Size(267, 150);
            this.gpAdditon.TabIndex = 26;
            this.gpAdditon.TabStop = false;
            // 
            // nudMeterCount
            // 
            this.nudMeterCount.Location = new System.Drawing.Point(83, 112);
            this.nudMeterCount.Maximum = new decimal(new int[] {
            1024,
            0,
            0,
            0});
            this.nudMeterCount.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudMeterCount.Name = "nudMeterCount";
            this.nudMeterCount.Size = new System.Drawing.Size(100, 20);
            this.nudMeterCount.TabIndex = 28;
            this.nudMeterCount.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudMeterCount.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(20, 30);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(55, 13);
            this.label4.TabIndex = 26;
            this.label4.Text = "时间信息";
            // 
            // nudTimeSlot
            // 
            this.nudTimeSlot.Location = new System.Drawing.Point(83, 83);
            this.nudTimeSlot.Maximum = new decimal(new int[] {
            1024,
            0,
            0,
            0});
            this.nudTimeSlot.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.nudTimeSlot.Name = "nudTimeSlot";
            this.nudTimeSlot.Size = new System.Drawing.Size(100, 20);
            this.nudTimeSlot.TabIndex = 27;
            this.nudTimeSlot.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.nudTimeSlot.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // ReadRealData
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.cbAdditionInfo);
            this.Controls.Add(this.gpAdditon);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.cmbDataFormat);
            this.Name = "ReadRealData";
            this.Size = new System.Drawing.Size(300, 433);
            this.Load += new System.EventHandler(this.ReadRealData_Load);
            this.gpAdditon.ResumeLayout(false);
            this.gpAdditon.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.nudMeterCount)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.nudTimeSlot)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();


            // 
            // timer
            // 
            this.FunListTimer.Enabled = true;
            this.FunListTimer.Interval = 10;
            this.FunListTimer.Tick += new System.EventHandler(this.FunListTimer_Tick);

        }

        #endregion

        private System.Windows.Forms.ComboBox cmbDataFormat;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.CheckBox cbAdditionInfo;
        private System.Windows.Forms.DateTimePicker dtpTime;
        private System.Windows.Forms.DateTimePicker dtpDate;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.ComboBox cmbTimeType;
        private System.Windows.Forms.GroupBox gpAdditon;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.NumericUpDown nudMeterCount;
        private System.Windows.Forms.NumericUpDown nudTimeSlot;
        private System.Windows.Forms.Timer FunListTimer;
    }
}
