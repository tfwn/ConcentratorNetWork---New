using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace SR6009_Concentrator_Tools.FunList
{
    public partial class ReadRealData : UserControl,IGetParas
    {
        public class TimeSlot
        {
            public static int TableNumber = 1;
            public static int TotalTableNumber = 1;
        }
        public ReadRealData()
        {
            InitializeComponent();
        }
        private void ReadRealData_Load(object sender, EventArgs e)
        {
            cbAdditionInfo.Checked = false;
            gpAdditon.Enabled = cbAdditionInfo.Checked;
            cmbDataFormat.SelectedIndex = 0;
            cmbTimeType.SelectedIndex = 0;
            dtpDate.Enabled = false;
            dtpTime.Enabled = false;
            dtpDate.Value = DateTime.Now;
            dtpTime.Value = DateTime.Now;
        }
        public int GetDataBuf(byte[] DataBuf, int Start, int TableNumber, int TotalTableNumber)
        {
            int iLen = 0;
            nudTimeSlot.Value = TableNumber;
            nudMeterCount.Value = TotalTableNumber;
            DataBuf[Start + iLen++] = (byte)cmbDataFormat.SelectedIndex;        // 定时定量数据
            if (cbAdditionInfo.Checked == true)
            {
                if (cmbTimeType.SelectedIndex == 0)
                {
                    dtpDate.Value = DateTime.Now;
                    dtpTime.Value = DateTime.Now;
                }
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpDate.Value.Year / 100));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpDate.Value.Year % 100));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpDate.Value.Month));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpDate.Value.Day));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpTime.Value.Hour));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpTime.Value.Minute));
                DataBuf[Start + iLen++] = FrmMain.Hex2Bcd((byte)(dtpTime.Value.Second));
                for (int iLoop = 0; iLoop < 12; iLoop++)
                {
                    DataBuf[Start + iLen++] = 0;
                }
                DataBuf[Start + iLen++] = (byte)(nudTimeSlot.Value % 256);
                DataBuf[Start + iLen++] = (byte)(nudTimeSlot.Value / 256);
                DataBuf[Start + iLen++] = (byte)(nudMeterCount.Value % 256);
                DataBuf[Start + iLen++] = (byte)(nudMeterCount.Value / 256);

            }

            return iLen;
        }
        public string GetResultString(byte[] DataBuf)
        {
            // 命令字(1)+节点地址(6)+转发结果(1)+数据格式编号(1)+数据格式+场强值(1)
            if (DataBuf[0] != 0x01)
            {
                return null;
            }
            if (DataBuf.Length <= FrmMain.AddrLength + 1 + 1 + 2)
            {
                return null;
            }
            int iPos = 1 + FrmMain.AddrLength + 1;
            string strInfo = "";
            if (DataBuf[iPos] == 0)
            {
                strInfo += "实时数据";
            }
            else if (DataBuf[iPos] == 1)
            {
                strInfo += "冻结数据";
            }
            else if (DataBuf[iPos] == 3)
            {
                strInfo += "定时数据";
            }
            else if (DataBuf[iPos] == 4)
            {
                strInfo += "定量数据";
            }
            else if (DataBuf[iPos] == 5)
            {
                strInfo += "报警数据";
            }
            iPos += 1;
            strInfo += " 表读数：" + FrmMain.Byte6ToUint64(DataBuf, iPos);
            return strInfo;
        }

        private void cbAdditionInfo_CheckedChanged(object sender, EventArgs e)
        {
            gpAdditon.Enabled = cbAdditionInfo.Checked;

        }
        #region 定时器
        private int _50MsTimer = 0;                                         // 50毫秒定时器
        private bool flag = false;
        private void FunListTimer_Tick(object sender, EventArgs e)
        {

            if (_50MsTimer++ > 50 / FunListTimer.Interval)
            {
                _50MsTimer = 0;
                nudMeterCount.Value = TimeSlot.TotalTableNumber;
                nudTimeSlot.Value = TimeSlot.TableNumber;
            }
        }
        #endregion
        private void cmbTimeType_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (cmbTimeType.SelectedIndex == 0)
            {
                dtpDate.Enabled = false;
                dtpTime.Enabled = false;
                dtpDate.Value = DateTime.Now;
                dtpTime.Value = DateTime.Now;  
            }
            else
            {
                dtpDate.Enabled = true;
                dtpTime.Enabled = true;
            }
        }
    }
}
