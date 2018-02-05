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
    public partial class SetFunOption : UserControl, IGetParas
    {
        string strFunOption = "";

        public SetFunOption()
        {
            InitializeComponent();
            strFunOption = Common.XmlHelper.GetNodeDefValue(FrmMain.strConfigFile, "/Config/Parameter/FunOption", "1,0,0,1,1,0,1,1,1,0,0,0,1,0,0,0");
            string[] strOpt = strFunOption.Split(',');
            for (int iLoop = 0; iLoop < strOpt.Length; iLoop++)
            {
                ComboBox cmbOption = (ComboBox)(gpbOption.Controls.Find("cmbOption" + iLoop, false))[0];
                cmbOption.SelectedIndex = Convert.ToInt16(strOpt[iLoop]);
            }
        }
        public int GetDataBuf(byte[] DataBuf, int Start, int TableNumber, int TotalTableNumber)
        {
            string strNewFunOption = "";
            int iLoop = 0;
            for (; iLoop < 16; iLoop++ )
            {
                ComboBox cmbOption = (ComboBox)(gpbOption.Controls.Find("cmbOption" + iLoop, false))[0];
                strNewFunOption += cmbOption.SelectedIndex.ToString("D") + ",";
                DataBuf[Start + iLoop] = (byte)(cmbOption.SelectedIndex);
            }
            strNewFunOption = strNewFunOption.TrimEnd(',');
            if (strNewFunOption != strFunOption)
            {
                Common.XmlHelper.SetNodeValue(FrmMain.strConfigFile, "/Config/Parameter", "FunOption", strNewFunOption);
            }
            return iLoop;
        }
        public string GetResultString(byte[] DataBuf)
        {
            // 命令字(1)+节点地址(6)+转发结果(1)+操作状态+场强值(1)
            if (DataBuf[0] != 0x08)
            {
                return null;
            }
            if (DataBuf.Length <= FrmMain.AddrLength + 1 + 1 + 2)
            {
                return null;
            }
            int iPos = 1 + FrmMain.AddrLength + 1;
            string strInfo = FrmMain.GetErrorInfo(DataBuf[iPos]);
            return strInfo;
        }
    }
}
