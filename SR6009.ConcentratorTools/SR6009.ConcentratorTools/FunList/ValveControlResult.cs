using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;

namespace SR6009_Concentrator_Tools.FunList
{
    public partial class ValveControlResult : UserControl,IGetResult
    {
        DataTable dt = new DataTable();

        public ValveControlResult()
        {
            InitializeComponent();
        }
        private void ValveControlResult_Load(object sender, EventArgs e)
        {
            if (dgvDoc == null)
            {
                return;
            }
            dt.TableName = "表具阀门控制";
            dt.Columns.Add("序号", typeof(string));
            dt.Columns.Add("表具地址", typeof(string));
            dt.Columns.Add("命令类型", typeof(string));
            dt.Columns.Add("操作时间", typeof(string));
            dt.Columns.Add("操作结果", typeof(string));
            dt.Columns.Add("场强↓", typeof(string));
            dt.Columns.Add("场强↑", typeof(string));
            dt.Columns.Add("阀门操作", typeof(string));
            dt.Columns.Add("是否强制", typeof(string));
            dt.Columns.Add("失败原因", typeof(string));
            dgvDoc.DataSource = dt;
            dgvDoc.Columns["序号"].Width = 40;
            dgvDoc.Columns["序号"].Frozen = true;
            dgvDoc.Columns["表具地址"].Width = 100;
            dgvDoc.Columns["表具地址"].Frozen = true;
            dgvDoc.Columns["命令类型"].Width = 100;
            dgvDoc.Columns["操作时间"].Width = 150;
            dgvDoc.Columns["操作结果"].Width = 130;
            dgvDoc.Columns["场强↓"].Width = 60;
            dgvDoc.Columns["场强↑"].Width = 60;
            dgvDoc.Columns["阀门操作"].Width = 70;
            dgvDoc.Columns["是否强制"].Width = 70;
            dgvDoc.Columns["失败原因"].Width = 120;
            string strFile = Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath) + "\\DataForward\\ValveControl.xml";
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
        // TxBuf:命令字+节点地址+转发内容
        public string GetResultString(byte[] DataBuf, byte[] TxBuf)
        {
            string strInfo = null;
            if (DataBuf[0] != 0x03)
            {
                return null;
            }
            DataRow dr = dt.NewRow();
            dr["序号"] = dt.Rows.Count + 1;
            dr["表具地址"] = FrmMain.GetStringAddrFromByte(DataBuf, 1);
            dr["命令类型"] = FrmMain.GetStringFromCmd(DataBuf[0]);
            dr["操作时间"] = DateTime.Now.ToString("yyyy年MM月dd日 HH:mm:ss");
            if (DataBuf.Length == FrmMain.AddrLength + 1 + 1)
            {
                dr["操作结果"] = FrmMain.GetErrorInfo(DataBuf[1 + FrmMain.AddrLength]);
            }
            else
            {
                if (DataBuf[1 + FrmMain.AddrLength + 1] == 0xAA)
                {
                    dr["操作结果"] = "开关阀成功";
                }
                else if (DataBuf[1 + FrmMain.AddrLength + 1] == 0xAB)
                {
                    dr["操作结果"] = "开关阀失败";
                }
                else if (DataBuf[1 + FrmMain.AddrLength + 1] == 0xAD)
                {
                    dr["操作结果"] = "表端接收命令成功";
                }
            }
            if (TxBuf.Length >= 1 + FrmMain.AddrLength + 2)
            {
                if (TxBuf[1 + FrmMain.AddrLength] == 0)
                {
                    dr["是否强制"] = "正常";
                } 
                else if (TxBuf[1 + FrmMain.AddrLength] == 0x01)
                {
                    dr["是否强制"] = "强制";
                }
                if (TxBuf[2 + FrmMain.AddrLength] == 0)
                {
                    dr["阀门操作"] = "关阀";
                }
                else if (TxBuf[2 + FrmMain.AddrLength] == 0x01)
                {
                    dr["阀门操作"] = "开阀";
                }
            }
            if (DataBuf.Length >= 1 + FrmMain.AddrLength + 1 + 5)
            {
                int iPos = 1 + FrmMain.AddrLength + 1 + 1;
                string strFailReson = "";
                if ((DataBuf[iPos] & 0x01) == 0x01)
                {
                    strFailReson += "电池欠压,";
                }
                if ((DataBuf[iPos] & 0x02) == 0x02)
                {
                    strFailReson += "磁干扰,";
                }
                if ((DataBuf[iPos] & 0x04) == 0x04)
                {
                    strFailReson += "ADC正在工作,";
                }
                if ((DataBuf[iPos] & 0x08) == 0x08)
                {
                    strFailReson += "阀门正在运行中,";
                }
                if ((DataBuf[iPos] & 0x10) == 0x10)
                {
                    strFailReson += "阀门故障,";
                }
                if ((DataBuf[iPos] & 0x20) == 0x20)
                {
                    strFailReson += "RF正在工作,";
                }
                if ((DataBuf[iPos] & 0x40) == 0x40)
                {
                    strFailReson += "任务申请失败,";
                }
                if ((DataBuf[iPos] & 0x80) == 0x80)
                {
                    strFailReson += "等待按键开阀,";
                }
                iPos += 1;
                if ((DataBuf[iPos] & 0x01) == 0x01)
                {
                    strFailReson += "当前阀门已经到位,";
                }
                if ((DataBuf[iPos] & 0x02) == 0x02)
                {
                    strFailReson += "设备类型错误,";
                }
                if ((DataBuf[iPos] & 0x04) == 0x04)
                {
                    strFailReson += "time申请失败,";
                }
                if ((DataBuf[iPos] & 0x08) == 0x08)
                {
                    strFailReson += "系统欠费,";
                }
                dr["失败原因"] = strFailReson.TrimEnd(',');
                iPos++;
                dr["场强↓"] = "-" + DataBuf[iPos++].ToString("D") + "dBm";
                dr["场强↑"] = "-" + DataBuf[iPos++].ToString("D") + "dBm";
            }
            else
            {
                dr["失败原因"] = "";
                dr["场强↓"] = "";
                dr["场强↑"] = "";
            }
            if (DataBuf.Length > 1 + FrmMain.AddrLength + 1)
            {
                if (DataBuf[1 + FrmMain.AddrLength + 1] != 0xAB)
                {
                    strInfo = dr["操作结果"].ToString();
                }
                else if (dr["失败原因"].ToString().Length > 0)
                {
                    strInfo += ", 失败原因：" + dr["失败原因"].ToString();
                }
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
            string strFile = strPath + "\\ValveControl.xml";
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
                string strFile = strPath + "\\ValveControl.xml";
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
