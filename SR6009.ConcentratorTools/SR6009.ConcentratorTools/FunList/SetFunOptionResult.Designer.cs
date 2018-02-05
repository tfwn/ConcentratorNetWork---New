namespace SR6009_Concentrator_Tools.FunList
{
    partial class SetFunOptionResult
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
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle5 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle6 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle7 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle8 = new System.Windows.Forms.DataGridViewCellStyle();
            this.dgvDoc = new System.Windows.Forms.DataGridView();
            this.cmsMenu = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.tsmiAddToForwardList = new System.Windows.Forms.ToolStripMenuItem();
            this.tsmiDelSelectItem = new System.Windows.Forms.ToolStripMenuItem();
            ((System.ComponentModel.ISupportInitialize)(this.dgvDoc)).BeginInit();
            this.cmsMenu.SuspendLayout();
            this.SuspendLayout();
            // 
            // dgvDoc
            // 
            this.dgvDoc.AllowUserToAddRows = false;
            this.dgvDoc.AllowUserToDeleteRows = false;
            this.dgvDoc.AllowUserToOrderColumns = true;
            this.dgvDoc.AllowUserToResizeRows = false;
            dataGridViewCellStyle5.BackColor = System.Drawing.Color.LightBlue;
            this.dgvDoc.AlternatingRowsDefaultCellStyle = dataGridViewCellStyle5;
            this.dgvDoc.BackgroundColor = System.Drawing.Color.Snow;
            this.dgvDoc.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.dgvDoc.ColumnHeadersBorderStyle = System.Windows.Forms.DataGridViewHeaderBorderStyle.Single;
            dataGridViewCellStyle6.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle6.BackColor = System.Drawing.Color.Snow;
            dataGridViewCellStyle6.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle6.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle6.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle6.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle6.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.dgvDoc.ColumnHeadersDefaultCellStyle = dataGridViewCellStyle6;
            this.dgvDoc.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.DisableResizing;
            this.dgvDoc.ContextMenuStrip = this.cmsMenu;
            dataGridViewCellStyle7.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle7.BackColor = System.Drawing.SystemColors.Window;
            dataGridViewCellStyle7.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle7.ForeColor = System.Drawing.SystemColors.ControlText;
            dataGridViewCellStyle7.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle7.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle7.WrapMode = System.Windows.Forms.DataGridViewTriState.False;
            this.dgvDoc.DefaultCellStyle = dataGridViewCellStyle7;
            this.dgvDoc.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dgvDoc.EditMode = System.Windows.Forms.DataGridViewEditMode.EditProgrammatically;
            this.dgvDoc.GridColor = System.Drawing.Color.DarkGreen;
            this.dgvDoc.Location = new System.Drawing.Point(0, 0);
            this.dgvDoc.Name = "dgvDoc";
            this.dgvDoc.ReadOnly = true;
            this.dgvDoc.RowHeadersBorderStyle = System.Windows.Forms.DataGridViewHeaderBorderStyle.Single;
            dataGridViewCellStyle8.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle8.BackColor = System.Drawing.Color.Snow;
            dataGridViewCellStyle8.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle8.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle8.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle8.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle8.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.dgvDoc.RowHeadersDefaultCellStyle = dataGridViewCellStyle8;
            this.dgvDoc.RowHeadersVisible = false;
            this.dgvDoc.RowHeadersWidth = 45;
            this.dgvDoc.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing;
            this.dgvDoc.RowTemplate.Height = 23;
            this.dgvDoc.SelectionMode = System.Windows.Forms.DataGridViewSelectionMode.FullRowSelect;
            this.dgvDoc.ShowCellErrors = false;
            this.dgvDoc.ShowEditingIcon = false;
            this.dgvDoc.ShowRowErrors = false;
            this.dgvDoc.Size = new System.Drawing.Size(638, 554);
            this.dgvDoc.TabIndex = 1;
            // 
            // cmsMenu
            // 
            this.cmsMenu.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.tsmiAddToForwardList,
            this.tsmiDelSelectItem});
            this.cmsMenu.Name = "cmsAddTask";
            this.cmsMenu.Size = new System.Drawing.Size(185, 48);
            this.cmsMenu.Opening += new System.ComponentModel.CancelEventHandler(this.cmsMenu_Opening);
            // 
            // tsmiAddToForwardList
            // 
            this.tsmiAddToForwardList.Name = "tsmiAddToForwardList";
            this.tsmiAddToForwardList.Size = new System.Drawing.Size(184, 22);
            this.tsmiAddToForwardList.Text = "添加到数据转发队列";
            this.tsmiAddToForwardList.Click += new System.EventHandler(this.tsmiAddToForwardList_Click);
            // 
            // tsmiDelSelectItem
            // 
            this.tsmiDelSelectItem.Name = "tsmiDelSelectItem";
            this.tsmiDelSelectItem.Size = new System.Drawing.Size(184, 22);
            this.tsmiDelSelectItem.Text = "删除所选记录";
            this.tsmiDelSelectItem.Click += new System.EventHandler(this.tsmiDelSelectItem_Click);
            // 
            // SetFunOptionResult
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.dgvDoc);
            this.Name = "SetFunOptionResult";
            this.Size = new System.Drawing.Size(638, 554);
            this.Load += new System.EventHandler(this.SetFunOptionResult_Load);
            ((System.ComponentModel.ISupportInitialize)(this.dgvDoc)).EndInit();
            this.cmsMenu.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.ContextMenuStrip cmsMenu;
        private System.Windows.Forms.ToolStripMenuItem tsmiAddToForwardList;
        private System.Windows.Forms.ToolStripMenuItem tsmiDelSelectItem;
        private System.Windows.Forms.DataGridView dgvDoc;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 序号DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 表具地址DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 命令类型DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 操作时间DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 操作结果DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 磁扰关阀DataGridViewTextBoxColumn1;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 数据加密DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 防拆检测DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 垂直安装DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 上报功能DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 冻结上报DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colCloseWhenOutOfMoneyDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colPrePaidDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colPreventRustDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colCloseWhenPowerOffDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colSleepModeDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colAutoCloseDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colLeakDetectDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn colFlowControlDataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 场强DataGridViewTextBoxColumn;
        //private System.Windows.Forms.DataGridViewTextBoxColumn 场强DataGridViewTextBoxColumn1;

    }
}
