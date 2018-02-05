namespace SR6009_Concentrator_Tools.FunList
{
    partial class ValveControl
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
            this.cmbValveCtrl = new System.Windows.Forms.ComboBox();
            this.cmbValveOption = new System.Windows.Forms.ComboBox();
            this.SuspendLayout();
            // 
            // cmbValveCtrl
            // 
            this.cmbValveCtrl.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbValveCtrl.FormattingEnabled = true;
            this.cmbValveCtrl.Items.AddRange(new object[] {
            "开阀",
            "关阀"});
            this.cmbValveCtrl.Location = new System.Drawing.Point(11, 11);
            this.cmbValveCtrl.Name = "cmbValveCtrl";
            this.cmbValveCtrl.Size = new System.Drawing.Size(86, 20);
            this.cmbValveCtrl.TabIndex = 0;
            // 
            // cmbValveOption
            // 
            this.cmbValveOption.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbValveOption.FormattingEnabled = true;
            this.cmbValveOption.Items.AddRange(new object[] {
            "正常",
            "强制"});
            this.cmbValveOption.Location = new System.Drawing.Point(106, 11);
            this.cmbValveOption.Name = "cmbValveOption";
            this.cmbValveOption.Size = new System.Drawing.Size(86, 20);
            this.cmbValveOption.TabIndex = 1;
            // 
            // ValveControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.cmbValveOption);
            this.Controls.Add(this.cmbValveCtrl);
            this.Name = "ValveControl";
            this.Size = new System.Drawing.Size(300, 400);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ComboBox cmbValveCtrl;
        private System.Windows.Forms.ComboBox cmbValveOption;
    }
}
