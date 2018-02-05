namespace SR6009_Concentrator_Tools.FunList
{
    partial class GetFunOptionResult
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
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle3 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle4 = new System.Windows.Forms.DataGridViewCellStyle();
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
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle1.BackColor = System.Drawing.Color.LightGreen;
            this.dgvDoc.AlternatingRowsDefaultCellStyle = dataGridViewCellStyle1;
            this.dgvDoc.BackgroundColor = System.Drawing.Color.Snow;
            this.dgvDoc.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.dgvDoc.ColumnHeadersBorderStyle = System.Windows.Forms.DataGridViewHeaderBorderStyle.Single;
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle2.BackColor = System.Drawing.Color.Snow;
            dataGridViewCellStyle2.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle2.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle2.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle2.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle2.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.dgvDoc.ColumnHeadersDefaultCellStyle = dataGridViewCellStyle2;
            this.dgvDoc.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.DisableResizing;
            this.dgvDoc.ContextMenuStrip = this.cmsMenu;
            dataGridViewCellStyle3.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle3.BackColor = System.Drawing.SystemColors.Window;
            dataGridViewCellStyle3.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle3.ForeColor = System.Drawing.SystemColors.ControlText;
            dataGridViewCellStyle3.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle3.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle3.WrapMode = System.Windows.Forms.DataGridViewTriState.False;
            this.dgvDoc.DefaultCellStyle = dataGridViewCellStyle3;
            this.dgvDoc.Dock = System.Windows.Forms.DockStyle.Fill;
            this.dgvDoc.EditMode = System.Windows.Forms.DataGridViewEditMode.EditProgrammatically;
            this.dgvDoc.GridColor = System.Drawing.Color.DarkGreen;
            this.dgvDoc.Location = new System.Drawing.Point(0, 0);
            this.dgvDoc.Name = "dgvDoc";
            this.dgvDoc.ReadOnly = true;
            this.dgvDoc.RowHeadersBorderStyle = System.Windows.Forms.DataGridViewHeaderBorderStyle.Single;
            dataGridViewCellStyle4.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft;
            dataGridViewCellStyle4.BackColor = System.Drawing.Color.Snow;
            dataGridViewCellStyle4.Font = new System.Drawing.Font("宋体", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(134)));
            dataGridViewCellStyle4.ForeColor = System.Drawing.SystemColors.WindowText;
            dataGridViewCellStyle4.SelectionBackColor = System.Drawing.SystemColors.Highlight;
            dataGridViewCellStyle4.SelectionForeColor = System.Drawing.SystemColors.HighlightText;
            dataGridViewCellStyle4.WrapMode = System.Windows.Forms.DataGridViewTriState.True;
            this.dgvDoc.RowHeadersDefaultCellStyle = dataGridViewCellStyle4;
            this.dgvDoc.RowHeadersVisible = false;
            this.dgvDoc.RowHeadersWidth = 45;
            this.dgvDoc.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing;
            this.dgvDoc.RowTemplate.Height = 23;
            this.dgvDoc.SelectionMode = System.Windows.Forms.DataGridViewSelectionMode.FullRowSelect;
            this.dgvDoc.ShowCellErrors = false;
            this.dgvDoc.ShowEditingIcon = false;
            this.dgvDoc.ShowRowErrors = false;
            this.dgvDoc.Size = new System.Drawing.Size(638, 554);
            this.dgvDoc.TabIndex = 2;
            // 
            // cmsMenu
            // 
            this.cmsMenu.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.tsmiAddToForwardList,
            this.tsmiDelSelectItem});
            this.cmsMenu.Name = "cmsMenu";
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
            // GetFunOptionResult
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.dgvDoc);
            this.Name = "GetFunOptionResult";
            this.Size = new System.Drawing.Size(638, 554);
            this.Load += new System.EventHandler(this.GetFunOptionResult_Load);
            ((System.ComponentModel.ISupportInitialize)(this.dgvDoc)).EndInit();
            this.cmsMenu.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.DataGridView dgvDoc;
        private System.Windows.Forms.ContextMenuStrip cmsMenu;
        private System.Windows.Forms.ToolStripMenuItem tsmiAddToForwardList;
        private System.Windows.Forms.ToolStripMenuItem tsmiDelSelectItem;
    }
}
