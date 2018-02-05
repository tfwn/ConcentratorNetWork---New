using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SR6009_Concentrator_Tools.FunList
{
    public partial class GetFunOptionResult : UserControl,IGetResult
    {
        DataTable dt = new DataTable();

        public GetFunOptionResult()
        {
            InitializeComponent();
        }
        private void GetFunOptionResult_Load(object sender, EventArgs e)
        {
            if (dgvDoc == null)
            {
                return;
            }
            dt.TableName = "读取表计功能";
            dt.Columns.Add("序号", typeof(string));
            dt.Columns.Add("表具地址", typeof(string));
            dt.Columns.Add("命令类型", typeof(string));
            dt.Columns.Add("操作时间", typeof(string));
            dt.Columns.Add("操作结果", typeof(string));
            dt.Columns.Add("场强↓", typeof(string));
            dt.Columns.Add("场强↑", typeof(string));
            dt.Columns.Add("磁扰关阀", typeof(string));
            dt.Columns.Add("数据加密", typeof(string));
            dt.Columns.Add("防拆检测", typeof(string));
            dt.Columns.Add("垂直安装", typeof(string));
            dt.Columns.Add("上报功能", typeof(string));
            dt.Columns.Add("冻结上报", typeof(string));
            dt.Columns.Add("欠费关阀", typeof(string));
            dt.Columns.Add("预付费用", typeof(string));
            dt.Columns.Add("阀门防锈", typeof(string));
            dt.Columns.Add("掉电关阀", typeof(string));
            dt.Columns.Add("休眠策略", typeof(string));
            dt.Columns.Add("自动关阀", typeof(string));
            dt.Columns.Add("泄露检测", typeof(string));
            dt.Columns.Add("流速控制", typeof(string));
            dgvDoc.DataSource = dt;
            dgvDoc.Columns["序号"].Width = 40;
            dgvDoc.Columns["序号"].Frozen = true;
            dgvDoc.Columns["表具地址"].Width = 100;
            dgvDoc.Columns["表具地址"].Frozen = true;
            dgvDoc.Columns["命令类型"].Width = 130;
            dgvDoc.Columns["操作时间"].Width = 150;
            dgvDoc.Columns["操作结果"].Width = 100;
            dgvDoc.Columns["场强↓"].Width = 60;
            dgvDoc.Columns["场强↑"].Width = 60;
            dgvDoc.Columns["磁扰关阀"].Width = 70;
            dgvDoc.Columns["数据加密"].Width = 70;
            dgvDoc.Columns["防拆检测"].Width = 70;
            dgvDoc.Columns["垂直安装"].Width = 70;
            dgvDoc.Columns["上报功能"].Width = 70;
            dgvDoc.Columns["冻结上报"].Width = 70;
            dgvDoc.Columns["欠费关阀"].Width = 70;
            dgvDoc.Columns["预付费用"].Width = 70;
            dgvDoc.Columns["阀门防锈"].Width = 70;
            dgvDoc.Columns["掉电关阀"].Width = 70;
            dgvDoc.Columns["休眠策略"].Width = 70;
            dgvDoc.Columns["自动关阀"].Width = 70;
            dgvDoc.Columns["泄露检测"].Width = 70;
            dgvDoc.Columns["流速控制"].Width = 70;
            string strFile = Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath) + "\\DataForward\\GetFunOption.xml";
            if (File.Exists(strFile) == true)
            {
                FileInfo info = new FileInfo(strFile);
                if (info.Length > 0)
                {
                    dt.ReadXml(strFile);
                    dgvDoc.Rows[dgvDoc.Rows.Count - 1].Selected = true;
                    dgvDoc.CurrentCell = dgvDoc.Rows[dgvDoc.Rows.Count - 1].Cells[1];
                }
            }
        }
        // DataBuf:命令字+节点地址+转发结果+转发内容
        // TxBuf:空
        public string GetResultString(byte[] DataBuf, byte[] TxBuf)
        {
            string strInfo = null;
            if (DataBuf[0] != 0x0B)
            {
                return null;
            }    
            DataRow dr = dt.NewRow();
            dr["序号"] = dt.Rows.Count + 1;
            dr["表具地址"] = FrmMain.GetStringAddrFromByte(DataBuf, 1);
            dr["命令类型"] = FrmMain.GetStringFromCmd(DataBuf[0]);
            dr["操作时间"] = DateTime.Now.ToString("yyyy年MM月dd日 HH:mm:ss");
            dr["操作结果"] = FrmMain.GetErrorInfo(DataBuf[1 + FrmMain.AddrLength]); 
            if (DataBuf.Length < FrmMain.AddrLength + 1 + 1)
            {
                return null;
            }
            if (DataBuf.Length >= FrmMain.AddrLength + 4)
            {
                int iPos = 1 + FrmMain.AddrLength + 1;
                dr["磁扰关阀"] = (DataBuf[iPos] & 0x01) == 0 ? "关闭" : "开启";
                dr["数据加密"] = (DataBuf[iPos] & 0x02) == 0 ? "关闭" : "开启";
                dr["防拆检测"] = (DataBuf[iPos] & 0x04) == 0 ? "关闭" : "开启";
                dr["垂直安装"] = (DataBuf[iPos] & 0x08) == 0 ? "关闭" : "开启";
                dr["上报功能"] = (DataBuf[iPos] & 0x10) == 0 ? "关闭" : "开启";
                dr["冻结上报"] = (DataBuf[iPos] & 0x20) == 0 ? "关闭" : "开启";
                dr["欠费关阀"] = (DataBuf[iPos] & 0x40) == 0 ? "关闭" : "开启";
                dr["预付费用"] = (DataBuf[iPos] & 0x80) == 0 ? "关闭" : "开启";
                iPos++;
                dr["阀门防锈"] = (DataBuf[iPos] & 0x02) == 0 ? "关闭" : "开启";
                dr["掉电关阀"] = (DataBuf[iPos] & 0x04) == 0 ? "关闭" : "开启";
                dr["休眠策略"] = (DataBuf[iPos] & 0x08) == 0 ? "关闭" : "开启";
                dr["自动关阀"] = (DataBuf[iPos] & 0x10) == 0 ? "关闭" : "开启";
                dr["泄露检测"] = (DataBuf[iPos] & 0x20) == 0 ? "关闭" : "开启";
                dr["流速控制"] = (DataBuf[iPos] & 0x80) == 0 ? "关闭" : "开启";
                iPos++;
                if (DataBuf.Length > FrmMain.AddrLength + 1 + 1 + 1)
                {
                    dr["场强↓"] = "-" + DataBuf[iPos++].ToString("D") + "dBm";
                    dr["场强↑"] = "-" + DataBuf[iPos++].ToString("D") + "dBm";
                }
                else
                {
                    dr["场强↓"] = "";
                    dr["场强↑"] = "";
                }

                strInfo += "\n 磁扰关阀：" + dr["磁扰关阀"];
                strInfo += "\n 数据加密：" + dr["数据加密"];
                strInfo += "\n 防拆检测：" + dr["防拆检测"];
                strInfo += "\n 垂直安装：" + dr["垂直安装"];
                strInfo += "\n 上报功能：" + dr["上报功能"];
                strInfo += "\n 冻结上报：" + dr["冻结上报"];
                strInfo += "\n 欠费关阀：" + dr["欠费关阀"];
                strInfo += "\n 预付费用：" + dr["预付费用"];
                strInfo += "\n 阀门防锈：" + dr["阀门防锈"];
                strInfo += "\n 掉电关阀：" + dr["掉电关阀"];
                strInfo += "\n 休眠策略：" + dr["休眠策略"];
                strInfo += "\n 自动关阀：" + dr["自动关阀"];
                strInfo += "\n 泄露检测：" + dr["泄露检测"];
                strInfo += "\n 流速控制：" + dr["流速控制"];
            }
            dt.Rows.Add(dr);
            if (dt.Rows.Count > 2000)
            {
                dt.Rows.RemoveAt(0);
            }
            string strPath = Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath) + "\\DataForward";
            if (Directory.Exists(strPath) == false)
            {
                Directory.CreateDirectory(strPath);
            }
            string strFile = strPath + "\\GetFunOption.xml";
            if (File.Exists(strFile) == false)
            {
                File.Create(strFile).Close();
            }
            dt.WriteXml(strFile);
            return strInfo;
        }
        private void tsmiDelSelectItem_Click(object sender, EventArgs e)
        {
            if (dgvDoc.SelectedRows.Count > 0)
            {
                if (DialogResult.Cancel == MessageBox.Show("是否要删除选中的这" + dgvDoc.SelectedRows.Count.ToString() + "个记录吗？", "删除确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
                {
                    return;
                }
                int iCount = dgvDoc.SelectedRows.Count;
                for (int iRow = 0; iRow < iCount; iRow++)
                {
                    int index = Convert.ToInt32(dgvDoc.SelectedRows[0].Cells[0].Value) - 1;
                    dt.Rows.RemoveAt(index);
                    for (int iNewIndex = index; iNewIndex < dt.Rows.Count; iNewIndex++)
                    {
                        dt.Rows[iNewIndex]["序号"] = (iNewIndex + 1).ToString();
                    }
                }
                string strPath = Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath) + "\\DataForward";
                if (Directory.Exists(strPath) == false)
                {
                    Directory.CreateDirectory(strPath);
                }
                string strFile = strPath + "\\GetFunOption.xml";
                if (File.Exists(strFile) == false)
                {
                    File.Create(strFile).Close();
                }
                dt.WriteXml(strFile);
            }
        }
        private void tsmiAddToForwardList_Click(object sender, EventArgs e)
        {
            int index, iLoop;
            if (dgvDoc.SelectedRows.Count <= 0)
            {
                return;
            }
            iLoop = 0;
            if (null == FrmMain.strDataFwdNodeAddrList)
            {
                FrmMain.strDataFwdNodeAddrList = new string[1];
                FrmMain.strDataFwdNodeAddrList[0] = dgvDoc.SelectedRows[0].Cells[1].Value.ToString();
                iLoop = 1;
            }
            for (; iLoop < dgvDoc.SelectedRows.Count; iLoop++)
            {
                for (index = 0; index < FrmMain.strDataFwdNodeAddrList.Length; index++)
                {
                    if (dgvDoc.SelectedRows[iLoop].Cells[1].Value.ToString() == FrmMain.strDataFwdNodeAddrList[index].ToString())
                    {
                        break;
                    }
                }
                if (index >= FrmMain.strDataFwdNodeAddrList.Length)
                {
                    Array.Resize(ref FrmMain.strDataFwdNodeAddrList, FrmMain.strDataFwdNodeAddrList.Length + 1);
                    FrmMain.strDataFwdNodeAddrList[FrmMain.strDataFwdNodeAddrList.Length - 1] = dgvDoc.SelectedRows[iLoop].Cells[1].Value.ToString();
                }
            }
        }
        private void cmsMenu_Opening(object sender, CancelEventArgs e)
        {
            if (dgvDoc.SelectedRows.Count > 0)
            {
                tsmiDelSelectItem.Enabled = true;
                tsmiAddToForwardList.Enabled = true;
            }
            else
            {
                tsmiDelSelectItem.Enabled = false;
                tsmiAddToForwardList.Enabled = false;
            }
        }
    }
}
