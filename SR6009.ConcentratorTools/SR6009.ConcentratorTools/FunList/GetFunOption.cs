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
    public partial class GetFunOption : UserControl, IGetParas
    {
        DataTable dt = new DataTable();
        public GetFunOption()
        {
            InitializeComponent();
        }
        public int GetDataBuf(byte[] DataBuf, int Start, int TableNumber, int TotalTableNumber)
        {
            return 0;
        }
        public string GetResultString(byte[] DataBuf)
        {
            // 命令字(1)+节点地址(6)+转发结果(1)+开关成功标志(1)+失败原因(2)+场强值(1)
            if (DataBuf[0] != 0x0B)
            {
                return null;
            }
            if (DataBuf.Length <= FrmMain.AddrLength + 1 + 1 + 2)
            {
                return null;
            }
            int iPos = 1 + FrmMain.AddrLength + 1;
            string strInfo = "";
            strInfo += "\n 磁扰关阀：" + ((DataBuf[iPos] & 0x01) == 0x01 ? "开启" : "关闭");
            strInfo += "\n 数据加密：" + ((DataBuf[iPos] & 0x02) == 0x02 ? "开启" : "关闭");
            strInfo += "\n 防拆检测：" + ((DataBuf[iPos] & 0x04) == 0x04 ? "开启" : "关闭");
            strInfo += "\n 垂直安装：" + ((DataBuf[iPos] & 0x08) == 0x08 ? "开启" : "关闭");
            strInfo += "\n 上报功能：" + ((DataBuf[iPos] & 0x10) == 0x10 ? "开启" : "关闭");
            strInfo += "\n 冻结上报：" + ((DataBuf[iPos] & 0x20) == 0x20 ? "开启" : "关闭");
            strInfo += "\n 欠费关阀：" + ((DataBuf[iPos] & 0x40) == 0x40 ? "开启" : "关闭");
            strInfo += "\n 预付费用：" + ((DataBuf[iPos] & 0x80) == 0x80 ? "开启" : "关闭");
            iPos += 1;
            strInfo += "\n 阀门防锈：" + ((DataBuf[iPos] & 0x02) == 0x02 ? "开启" : "关闭");
            strInfo += "\n 掉电关阀：" + ((DataBuf[iPos] & 0x04) == 0x04 ? "开启" : "关闭");
            strInfo += "\n 休眠策略：" + ((DataBuf[iPos] & 0x08) == 0x08 ? "开启" : "关闭");
            strInfo += "\n 自动关阀：" + ((DataBuf[iPos] & 0x10) == 0x10 ? "开启" : "关闭");
            strInfo += "\n 泄露检测：" + ((DataBuf[iPos] & 0x20) == 0x20 ? "开启" : "关闭");
            strInfo += "\n 流速控制：" + ((DataBuf[iPos] & 0x80) == 0x80 ? "开启" : "关闭");
            return strInfo;
        }
    }
}
