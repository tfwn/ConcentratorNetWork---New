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
    public partial class ReadRealDataResult : UserControl,IGetResult
    {
        DataTable dt = new DataTable();

        public ReadRealDataResult()
        {
            InitializeComponent();
        }

        private void ReadRealDataResult_Load(object sender, EventArgs e)
        {
            if (dgvDoc == null)
            {
                return;
            }
            dt.TableName = "读取表计计量信息";
            dt.Columns.Add("序号", typeof(string));
            dt.Columns.Add("表具地址", typeof(string));
            dt.Columns.Add("命令类型", typeof(string));
            dt.Columns.Add("操作时间", typeof(string));
            dt.Columns.Add("操作结果", typeof(string));
            dt.Columns.Add("数据格式", typeof(string));
            dt.Columns.Add("场强↓", typeof(string));
            dt.Columns.Add("场强↑", typeof(string));
            dt.Columns.Add("报警信息", typeof(string));
            dt.Columns.Add("阀门状态", typeof(string));
            dt.Columns.Add("电压", typeof(string));
            dt.Columns.Add("温度", typeof(string));
            dt.Columns.Add("信噪比", typeof(string));
            dt.Columns.Add("信道", typeof(string));
            dt.Columns.Add("版本", typeof(string));
            dt.Columns.Add("正转用量", typeof(string));
            dt.Columns.Add("反转用量", typeof(string));
            dt.Columns.Add("起始序号", typeof(string));
            dt.Columns.Add("冻结时间", typeof(string));
            dt.Columns.Add("冻结方式", typeof(string));
            dt.Columns.Add("冻结数量", typeof(string));
            dt.Columns.Add("时间间隔", typeof(string));
            dt.Columns.Add("冻结数据", typeof(string));
            dgvDoc.DataSource = dt;
            dgvDoc.Columns["序号"].Width = 40;
            dgvDoc.Columns["序号"].Frozen = true;
            dgvDoc.Columns["表具地址"].Width = 100;
            dgvDoc.Columns["表具地址"].Frozen = true;
            dgvDoc.Columns["命令类型"].Width = 140;
            dgvDoc.Columns["操作时间"].Width = 150;
            dgvDoc.Columns["操作结果"].Width = 100;
            dgvDoc.Columns["数据格式"].Width = 80;
            dgvDoc.Columns["场强↓"].Width = 60;
            dgvDoc.Columns["场强↑"].Width = 60;
            dgvDoc.Columns["报警信息"].Width = 100;
            dgvDoc.Columns["阀门状态"].Width = 60;
            dgvDoc.Columns["电压"].Width = 60;
            dgvDoc.Columns["温度"].Width = 60;
            dgvDoc.Columns["信噪比"].Width = 60;
            dgvDoc.Columns["信道"].Width = 70;
            dgvDoc.Columns["版本"].Width = 60;
            dgvDoc.Columns["正转用量"].Width = 100;
            dgvDoc.Columns["反转用量"].Width = 100;
            dgvDoc.Columns["起始序号"].Width = 60;
            dgvDoc.Columns["冻结时间"].Width = 130;
            dgvDoc.Columns["冻结方式"].Width = 70;
            dgvDoc.Columns["冻结数量"].Width = 60;
            dgvDoc.Columns["时间间隔"].Width = 80;
            dgvDoc.Columns["冻结数据"].Width = 800;

            string strFile = Path.GetDirectoryName(System.Windows.Forms.Application.ExecutablePath) + "\\DataForward\\ReadRealData.xml";
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
        public string GetResultString(byte[] DataBuf, byte[] TxBuf)
        {
            string strInfo = null;
            if (DataBuf[0] != 0x01)
            {
                return null;
            }
//            D3 91 37 00 C0 02 4D FA 3F 02 00 00 00 00 60 09 19 21 68 00 30 11 
            //01 10 16 10 22 14 18 
            //AA 00 01 00 00 00 00 00 00 00 00 00 00 00 00 20 01 23 1B 0F 39 01 49 57 55 55 DE 16 00 03 

            DataRow dr = dt.NewRow();
            dr["序号"] = dt.Rows.Count + 1;
            dr["表具地址"] = FrmMain.GetStringAddrFromByte(DataBuf, 1);
            dr["命令类型"] = FrmMain.GetStringFromCmd(DataBuf[0]);
            dr["操作时间"] = DateTime.Now.ToString("yyyy年MM月dd日 HH:mm:ss");
            strInfo = FrmMain.GetErrorInfo(DataBuf[1 + FrmMain.AddrLength]);
            dr["操作结果"] = strInfo;
            if (DataBuf.Length > FrmMain.AddrLength + 1 + 1)
            {
                int iPos = 1 + FrmMain.AddrLength + 1;
                if (DataBuf[iPos] == 0)
                {
                    dr["数据格式"] = "实时数据";
                }
                else if (DataBuf[iPos] == 1)
                {
                    dr["数据格式"] = "冻结数据";
                }
                else
                {
                    dr["数据格式"] = "未知格式";
                }
                iPos += 1;
                strInfo += "\n 数据格式：" + dr["数据格式"].ToString();
                string[] strResult = FrmMain.ExplainMeterData(DataBuf, iPos, DataBuf.Length - iPos);
                int index = 0;
                if (strResult.Length == 11)
                {   // 正转用量+反转用量
                    dr["正转用量"] = strResult[index++];
                    dr["反转用量"] = strResult[index++];
                    strInfo += "\n 正转用量：" + dr["正转用量"].ToString();
                    strInfo += "\n 反转用量：" + dr["反转用量"].ToString();
                }
                else if (strResult.Length == 15)
                {   // 起始序号+冻结时间+冻结方式+冻结数量+时间间隔+冻结数据
                    dr["起始序号"] = strResult[index++];
                    dr["冻结时间"] = strResult[index++];
                    dr["冻结方式"] = strResult[index++];
                    dr["冻结数量"] = strResult[index++];
                    dr["时间间隔"] = strResult[index++];
                    dr["冻结数据"] = strResult[index++];
                    strInfo += "\n 起始序号：" + dr["起始序号"].ToString();
                    strInfo += "\n 冻结时间：" + dr["冻结时间"].ToString();
                    strInfo += "\n 冻结方式：" + dr["冻结方式"].ToString();
                    strInfo += "\n 冻结数量：" + dr["冻结数量"].ToString();
                    strInfo += "\n 时间间隔：" + dr["时间间隔"].ToString();
                    strInfo += "\n 正转用量：" + dr["正转用量"].ToString();
                    strInfo += "\n 冻结数据：" + dr["冻结数据"].ToString();
                }
                else if (strResult.Length == 12)
                {   // 起始序号+冻结时间+冻结数据
                    dr["起始序号"] = strResult[index++];
                    dr["冻结时间"] = strResult[index++];
                    dr["冻结数据"] = strResult[index++];
                    strInfo += "\n 起始序号：" + dr["起始序号"].ToString();
                    strInfo += "\n 冻结时间：" + dr["冻结时间"].ToString();
                    strInfo += "\n 冻结数据：" + dr["冻结数据"].ToString();
                }
                dr["报警信息"] = strResult[index++];
                dr["阀门状态"] = strResult[index++];
                dr["电压"] = strResult[index++];
                dr["温度"] = strResult[index++];
                dr["信噪比"] = strResult[index++];
                dr["信道"] = strResult[index++];
                dr["版本"] = strResult[index++];
                dr["场强↓"] = strResult[index++];
                dr["场强↑"] = strResult[index++];
                strInfo += "\n 报警信息：" + dr["报警信息"].ToString();
                strInfo += "\n 阀门状态：" + dr["阀门状态"].ToString();
                strInfo += "\n 电压：" + dr["电压"].ToString();
                strInfo += "\n 温度：" + dr["温度"].ToString();
                strInfo += "\n 信噪比：" + dr["信噪比"].ToString();
                strInfo += "\n 信道：" + dr["信道"].ToString();
                strInfo += "\n 版本：" + dr["版本"].ToString();
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
            string strFile = strPath + "\\ReadRealData.xml";
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
                string strFile = strPath + "\\ReadRealData.xml";
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
