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
    public partial class ValveControl : UserControl, IGetParas
    {
        string strValveCtrl = "";
        public ValveControl()
        {
            InitializeComponent();
            strValveCtrl = Common.XmlHelper.GetNodeDefValue(FrmMain.strConfigFile, "/Config/Parameter/ValveCtrl", "0,0");
            string[] strTemp = strValveCtrl.Split(',');
            cmbValveCtrl.SelectedIndex = Convert.ToInt16(strTemp[0]);
            cmbValveOption.SelectedIndex = Convert.ToInt16(strTemp[1]);
        }
        public int GetDataBuf(byte[] DataBuf, int Start, int TableNumber, int TotalTableNumber)
        {
            int iLen = 0;
            DataBuf[Start + iLen++] = (byte)("正常" == cmbValveOption.Text ? 0x00 : 0x01);
            DataBuf[Start + iLen++] = (byte)("开阀" == cmbValveCtrl.Text ? 0x01 : 0x00);
            string strNewValveCtrl = cmbValveCtrl.SelectedIndex.ToString("D") + "," + cmbValveOption.SelectedIndex.ToString("D");
            if (strNewValveCtrl != strValveCtrl)
            {
                Common.XmlHelper.SetNodeValue(FrmMain.strConfigFile, "/Config/Parameter", "ValveCtrl", strNewValveCtrl);
            }
            return iLen;
        }
        public string GetResultString(byte[] DataBuf)
        {
            // 命令字(1)+节点地址(6)+转发结果(1)+开关成功标志(1)+失败原因(2)+场强值(1)
            if (DataBuf[0] != 0x03)
            {
                return null;
            }
            if (DataBuf.Length <= FrmMain.AddrLength + 1 + 1 + 2)
            {
                return null;
            }
            int iPos = 1 + FrmMain.AddrLength + 1;
            string strInfo = cmbValveCtrl.Text + "(" + cmbValveOption.Text + ")指令 ";
            if (DataBuf[iPos] == 0xAA)
            {
                strInfo += "开关阀成功";
            }
            else if (DataBuf[iPos] == 0xAD)
            {
                strInfo += "表端接收命令成功";
            }
            else if (DataBuf[iPos] == 0xAB)
            {
                iPos += 1;
                strInfo = "开关阀失败 原因：";
                if ((DataBuf[iPos] & 0x01) == 0x01)
                {
                    strInfo += "电池欠压,";
                }
                else if ((DataBuf[iPos] & 0x02) == 0x02)
                {
                    strInfo += "磁干扰中,";
                }
                else if ((DataBuf[iPos] & 0x04) == 0x04)
                {
                    strInfo += "ADC正在工作,";
                }
                else if ((DataBuf[iPos] & 0x08) == 0x08)
                {
                    strInfo += "阀门正在运行中,";
                }
                else if ((DataBuf[iPos] & 0x10) == 0x10)
                {
                    strInfo += "阀门故障,";
                }
                else if ((DataBuf[iPos] & 0x20) == 0x20)
                {
                    strInfo += "RF正在工作,";
                }
                else if ((DataBuf[iPos] & 0x40) == 0x40)
                {
                    strInfo += "任务申请失败,";
                }
                else if ((DataBuf[iPos] & 0x80) == 0x80)
                {
                    strInfo += "等待按键开阀,";
                }
                iPos += 1;
                if ((DataBuf[iPos] & 0x01) == 0x01)
                {
                    strInfo += "当前阀门已经到位,";
                }
                else if ((DataBuf[iPos] & 0x02) == 0x02)
                {
                    strInfo += "设备类型错误,";
                }
                else if ((DataBuf[iPos] & 0x04) == 0x04)
                {
                    strInfo += "time申请失败,";
                }
                else if ((DataBuf[iPos] & 0x08) == 0x08)
                {
                    strInfo += "系统欠费,";
                }
                strInfo = strInfo.TrimEnd(',');
            }
            return strInfo;
        }
    }
}
