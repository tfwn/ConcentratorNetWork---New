using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;
using UsbLibrary;
using Common;
using SR6009_Concentrator_Tools.FunList;
using System.Net;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Diagnostics;
using Microsoft.Office.Core;

namespace SR6009_Concentrator_Tools
{
    public partial class FrmMain : Form
    {
        private enum enCmd
        {
            读表具数据 = 0x01,
            读冻结数据_6009 = 0x02,
            读表具冻结数据 = 0x05,

            复合命令字1 = 0x09,

            集中器版本信息 = 0x40,
            读集中器地址 = 0x41,
            写集中器地址 = 0x42,
            读集中器时钟 = 0x43,
            写集中器时钟 = 0x44,
            读Gprs参数 = 0x45,
            写Gprs参数 = 0x46,
            读Gprs信号强度 = 0x47,
            集中器初始化 = 0x48,
            读集中器工作模式参数 = 0x49,
            写集中器工作模式参数 = 0x4A,
            集中器请求时间 = 0x4B,
            集中器重新启动 = 0x4C,
            集中器数据转发 = 0x4D,
            写集中器第二扫描信道 = 0x4E,
            读集中器第二扫描信道 = 0x4F,

            读表具数量 = 0x50,
            读表具档案信息 = 0x51,
            写表具档案信息 = 0x52,
            删除表具档案信息 = 0x53,
            修改表具档案信息 = 0x54,
            读表具自定义路由信息 = 0x55,
            写表具自定义路由信息 = 0x56,
            批量读设备自定义路由信息 = 0x57,
            批量写设备自定义路由信息 = 0x58,

            定时定量数据主动上传 = 0x61,
            冻结数据主动上传 = 0x62,
            读集中器中的定时定量数据 = 0x63,
            读集中器中的冻结数据 = 0x64,
            批量读取集中器中的定时定量数据 = 0x65,
            批量读取集中器中的冻结数据 = 0x66,

            读集中器中的下发命令数据 = 0x67,
            批量读取集中器中的下发命令数据 = 0x68,
            批量写集中器中的下发命令数据 = 0x69,

            集中器程序升级 = 0xF1,
            集中器监控控制 = 0xF2,
            存储器检查 = 0xF3,
            指令空闲 = 0xFF,
        };
        private struct ProtocolStruct
        {
            public int PacketLength;                                        // 接收到消息长度
            public byte PacketProperty;                                     // 报文标识
            public byte TaskIDByte;                                         // 任务号字节
            public byte CommandIDByte;                                      // 命令字节
            public byte DeviceTypeByte;                                     // 设备类型字节
            public byte LifeCycleByte;                                      // 生命周期字节
            public byte RouteLevelByte;                                     // 路径级数信息
            public byte RouteCurPos;                                        // 路由当前位置
            public byte[] RoutePathList;                                    // 传输路径列表
            public byte[] DataBuf;                                          // 数据域字节
            public byte DownSignalStrengthByte;                             // 下行信号强度
            public byte UpSignalStrengthByte;                               // 上行信号强度
            public byte Crc8Byte;                                           // 校验字
            public int Crc16Byte;                                           // 校验字
            public byte EndByte;                                            // 结束符
            public bool isSuccess;                                          // 是否成功
        };
        private string[] strMeterType = new string[] {
            "冷水表",
            "热水表",
            "燃气表",
            "电能表",
            "中继器",
        };
        DataTable Document = new DataTable();
        private const string strNoPath = "无路由";
        public static byte AddrLength = 6;                                  // 地址长度
        private int CommDelayTime = 1300;                                   // 通信延时时间
        private const int MaxNodeNumOnePath = 5;// 7;                       // 路径中包含最大节点数(包含集中器和目标节点)
        private int _50MsTimer = 0;                                         // 50毫秒定时器
        private int _500MsTimer = 0;                                        // 500毫秒定时器
        private int[] PathLevel = new int[2] { 2, 2 };                      // 每条路径的级数(包括集中器和目标节点)
        private const byte XOR_CRC8 = 0x00;                                 // 校验码为 crc8 校验
        private const byte XOR_CRC16 = 0x01;                                // 校验码为 crc16 校验
        private const byte UpDir = 0x80;                                    // 上行数据
        private const byte AckFrm = 0x40;                                   // 应答帧
        private const byte EnCrypt = 0x20;                                  // 加密帧
        private const byte NeedAck = 0x10| XOR_CRC8;                        // 需要应答
        private const byte SyncWord0 = 0xD3;                                // 同步字0
        private const byte SyncWord1 = 0x91;                                // 同步字1
        private const int MaxNodeNum = 1024;                                // 最大表具数量
        private enCmd Command = enCmd.指令空闲;                             // 当前执行指令
        private enCmd LastCommand = enCmd.指令空闲;                         // 上一个执行的指令
        private int ReadPos = 0;                                            // 读取档案时的位置指示
        private int WritePos = 0;                                           // 写入档案时的位置指示
        private string strLocalAddr = "";                                   // 本机地址
        private byte bFrameSn = 0;                                          // 任务号 
        private int ReceiveWDT = 0;                                         // 接收缓冲器看门狗
        private int PortBufRdPos = 0;                                       // 接收缓冲器读位置
        private int PortBufWrPos = 0;                                       // 接收缓冲器写位置
        private byte[] PortRxBuf = new Byte[2000];                          // 接收缓冲器定义
        private delegate void SerialDataRecievedEventHandler(object sender, SerialDataReceivedEventArgs e);
        public static string strConfigFile = System.Windows.Forms.Application.StartupPath + @"\Config.xml";
        public string strCurDataFwdAddr = null;                             // 当前数据转发的地址
        public static string[] strDataFwdNodeAddrList;                      // 数据转发时表具地址列表
        private int iLastDataFwdNodeCount = 0;                              // 上一个数据转发队列数量
        private string strCurNodeAddr = "";                                 // 当前操作的节点地址
        private string strDataType = "定量数据";                            // 当前数据类型
        string strColumnIndex = "null";                                     // 列序号定义

        #region 窗口控制
        public FrmMain()
        {
            InitializeComponent();
        }
        private void FrmMain_Load(object sender, EventArgs e)
        {
            this.Text = "6009集中器调试工具 V2.10.171108";
            AddStringToCommBox(false, " 6009集中器调试工具\n <<<================= 上海桑锐电子科技股份有限公司 =================>>>", null, Color.DarkGreen);
            this.Width = 1360;
            GetLocalIP();
            cmbTimeType.SelectedIndex = 0;
            cmbNodeType.Items.AddRange(strMeterType);
            cmbOperateCmd.SelectedIndex = 0;
            CreateUsbThread();

            Document.TableName = "表具档案和抄表数据";
            Document.Columns.Add("序号", typeof(string));
            Document.Columns.Add("表具地址", typeof(string));
            Document.Columns.Add("类型", typeof(string));
            Document.Columns.Add("结果", typeof(string));
            Document.Columns.Add("路径0", typeof(string));
            Document.Columns.Add("路径1", typeof(string));
            Document.Columns.Add("场强↓", typeof(string));
            Document.Columns.Add("场强↑", typeof(string));
            Document.Columns.Add("报警信息", typeof(string));
            Document.Columns.Add("阀门状态", typeof(string));
            Document.Columns.Add("电压", typeof(string));
            Document.Columns.Add("温度", typeof(string));
            Document.Columns.Add("信噪比", typeof(string));
            Document.Columns.Add("信道", typeof(string));
            Document.Columns.Add("版本", typeof(string));
            Document.Columns.Add("读取时间", typeof(string));
            Document.Columns.Add("正转用量", typeof(string));
            Document.Columns.Add("反转用量", typeof(string));
            Document.Columns.Add("起始序号", typeof(string));
            Document.Columns.Add("冻结时间", typeof(string));
            Document.Columns.Add("冻结方式", typeof(string));
            Document.Columns.Add("冻结数量", typeof(string));
            Document.Columns.Add("时间间隔", typeof(string));
            Document.Columns.Add("冻结数据", typeof(string));

            Document.Columns.Add("下发命令状态", typeof(string));
            Document.Columns.Add("信道号", typeof(string));
            Document.Columns.Add("自动信道设置", typeof(string));
            Document.Columns.Add("阀控", typeof(string));
            Document.Columns.Add("实时抄表", typeof(string));
            Document.Columns.Add("结算价格", typeof(string));
            Document.Columns.Add("表具状态", typeof(string));
            Document.Columns.Add("保留位", typeof(string));

            dgvDoc.DataSource = Document;
            dgvDoc.Columns["序号"].Width = 40;
            dgvDoc.Columns["序号"].Frozen = true;         //水平滚动时不移动
            dgvDoc.Columns["表具地址"].Width = 100;
            dgvDoc.Columns["表具地址"].Frozen = true;
            dgvDoc.Columns["类型"].Width = 60;
            dgvDoc.Columns["结果"].Width = 100;
            dgvDoc.Columns["路径0"].Width = 100;
            dgvDoc.Columns["路径1"].Width = 100;
            dgvDoc.Columns["场强↓"].Width = 60;
            dgvDoc.Columns["场强↑"].Width = 60;
            dgvDoc.Columns["报警信息"].Width = 100;
            dgvDoc.Columns["阀门状态"].Width = 60;
            dgvDoc.Columns["电压"].Width = 50;
            dgvDoc.Columns["温度"].Width = 50;
            dgvDoc.Columns["信噪比"].Width = 50;
            dgvDoc.Columns["信道"].Width = 70;
            dgvDoc.Columns["版本"].Width = 60;
            dgvDoc.Columns["读取时间"].Width = 130;
            dgvDoc.Columns["正转用量"].Width = 80;
            dgvDoc.Columns["反转用量"].Width = 80;
            dgvDoc.Columns["起始序号"].Width = 60;
            dgvDoc.Columns["冻结时间"].Width = 130;
            dgvDoc.Columns["冻结方式"].Width = 70;
            dgvDoc.Columns["冻结数量"].Width = 60;
            dgvDoc.Columns["时间间隔"].Width = 80;
            dgvDoc.Columns["冻结数据"].Width = 800;


            dgvDoc.Columns["下发命令状态"].Width = 150;
            dgvDoc.Columns["信道号"].Width = 80;
            dgvDoc.Columns["自动信道设置"].Width = 150;
            dgvDoc.Columns["阀控"].Width = 60;
            dgvDoc.Columns["实时抄表"].Width = 100;
            dgvDoc.Columns["结算价格"].Width = 60;
            dgvDoc.Columns["表具状态"].Width = 120;
            dgvDoc.Columns["保留位"].Width = 800;

            strColumnIndex = XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/ColumnIndex", "");
            if (strColumnIndex != "")
            {
                string[] strIndex = strColumnIndex.Split('^');
                for (int iIndex = 0; iIndex < strIndex.Length; iIndex++)
                {
                    dgvDoc.Columns[strIndex[iIndex]].DisplayIndex = iIndex;
                }
            }
        }
        private void GetLocalIP()  
        {  
            try  
            {
                string name = Dns.GetHostName();
                IPAddress[] ipadrlist = Dns.GetHostAddresses(name);
                foreach (IPAddress ipa in ipadrlist)
                {
                    if (ipa.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                    {
                        string[] strIpSplit = ipa.ToString().Split('.');
                        strLocalAddr = "";
                        for (int iLoop = 0; iLoop < strIpSplit.Length; iLoop++ )
                        {
                            strLocalAddr += strIpSplit[iLoop].PadLeft(3, '0');
                        }
                        strLocalAddr = strLocalAddr.PadLeft(AddrLength * 2, '0');
                        strLocalAddr = strLocalAddr.Substring(0, AddrLength * 2);
                        tsLocalAddress.Text = "本机地址：" + strLocalAddr;
                        return;
                    }
                }
            }  
            catch  
            {
                strLocalAddr = "111111111111";
                tsLocalAddress.Text = "本机地址：" + strLocalAddr;
            }  
        } 
        private void dgvDoc_ColumnDisplayIndexChanged(object sender, DataGridViewColumnEventArgs e)
        {
            if (strColumnIndex == "null" || e.Column.Name == "序号" || e.Column.Name == "表具地址")
            {
                return;
            }
            string strColumnNewIndex = "";
            for (int iIndex = 0; iIndex < dgvDoc.Columns.Count; iIndex++)
            {
                for (int i = 0; i < dgvDoc.Columns.Count; i++)
                {
                    if (iIndex == dgvDoc.Columns[i].DisplayIndex)
                    {
                        strColumnNewIndex += dgvDoc.Columns[i].Name.ToString() + "^";
                        break;
                    }
                }
            }
            strColumnNewIndex = strColumnNewIndex.TrimEnd('^');
            if (strColumnNewIndex != strColumnIndex)
            {
                strColumnIndex = strColumnNewIndex;
                XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "ColumnIndex", strColumnIndex);
            }
        }
        private void FrmMain_SizeChanged(object sender, EventArgs e)
        {
            if (scon2.Panel2.Width > 30)
            {
                if (tbPageRec.Parent != tbPageRecord)
                {
                    tbPageDocument.Controls.Remove(tbPageRec);
                    tbPageRecord.Controls.Add(tbPageRec);
                }
            }
            else
            {
                if (tbPageRec.Parent != tbPageDocument)
                {
                    tbPageRecord.Controls.Remove(tbPageRec);
                    tbPageDocument.Controls.Add(tbPageRec);
                }
            }
            if (this.Size.Height > this.MinimumSize.Height)
            {
                gpbGprsInfo.Size = new System.Drawing.Size(352, this.Size.Height - 768 + 279);
                gpbDataFwd.Size = new System.Drawing.Size(352, this.Size.Height - 768 + 490);
                gpbFunList.Size = new System.Drawing.Size(329, this.Size.Height - 768 + 405);
            }
            else
            {
                gpbGprsInfo.Size = new System.Drawing.Size(352, 279);
                gpbDataFwd.Size = new System.Drawing.Size(352, 490);
                gpbFunList.Size = new System.Drawing.Size(329, 405);
            }
        }
        private void scon2_splitterMoved(object sender, SplitterEventArgs e)
        {
            if (scon2.Panel2.Width > 30)
            {
                if (tbPageRec.Parent != tbPageRecord)
                {
                    tbPageDocument.Controls.Remove(tbPageRec);
                    tbPageRecord.Controls.Add(tbPageRec);
                }
            }
            else
            {
                if (tbPageRec.Parent != tbPageDocument)
                {
                    tbPageRecord.Controls.Remove(tbPageRec);
                    tbPageDocument.Controls.Add(tbPageRec);
                }
            }
        }
        #endregion

        #region 端口操作
        private void cmbPort_Click(object sender, EventArgs e)
        {
            cmbPort.Items.Clear();
            foreach (string portName in SerialPort.GetPortNames())
            {
                cmbPort.Items.Add(portName);
            }
            if (true == _isConnect)
            {
                cmbPort.Items.Add("USB");
            }
        }
        private void cmbPort_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (cmbPort.Text == "USB")
            {
                cmbComType.Enabled = false;
                cmbComType.Text = "USB通信";
            }
            else
            {
                cmbComType.Enabled = true;
            }
        }
        private void btPortCtrl_Click(object sender, EventArgs e)
        {
            if (cmbPort.SelectedIndex < 0 || cmbPort.Text == "")
            {
                MessageBox.Show("请先选择通信端口！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (cmbComType.Text == "")
            {
                MessageBox.Show("请选择通信类型！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if ((cmbComType.Text == "USB通信" && cmbPort.Text != "USB") || (cmbComType.Text != "USB通信" && cmbPort.Text == "USB"))
            {
                MessageBox.Show("当前使用端口和通信类型不匹配！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (cmbComType.Text == "无线通信")
            {
                CommDelayTime = 6000;
            }
            else
            {
                CommDelayTime = 3000;
            }
            if (cmbPort.Text == "USB")
            {
                if (btPortCtrl.Text == "打开端口")
                {
                    if (true == _isConnect)
                    {
                        btPortCtrl.Text = "关闭端口";
                        cmbPort.Enabled = false;
                        cmbComType.Enabled = false;
                        AddStringToCommBox(true, "打开" + cmbPort.Text + "端口成功，通信类型为" + cmbComType.Text + "！", null, Color.Black);
                        AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                        lbCurState.Text = "设备状态：空闲";
                        tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：打开";
                    }
                    else
                    {
                        btPortCtrl.Text = "打开端口";
                        cmbPort.Enabled = true;
                        cmbComType.Enabled = true;
                        AddStringToCommBox(true, "打开" + cmbPort.Text + "端口失败！", null, Color.Red);
                        AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                        lbCurState.Text = "设备状态：通信端口关闭";
                        tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：关闭";
                    }
                }
                else
                {
                    btPortCtrl.Text = "打开端口";
                    cmbPort.Enabled = true;
                    cmbComType.Enabled = true;
                    AddStringToCommBox(true, "关闭" + cmbPort.Text + "端口成功！", null, Color.Black);
                    AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                    lbCurState.Text = "设备状态：通信端口关闭";
                    tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：关闭";
                }
                return;
            }
            else
            {
                if (btPortCtrl.Text == "打开端口")
                {
                    if (true == port_Ctrl(true))
                    {
                        btPortCtrl.Text = "关闭端口" ;
                        cmbPort.Enabled = false;
                        cmbComType.Enabled = false;
                        AddStringToCommBox(true, "打开" + cmbPort.Text + "端口成功，通信类型为" + cmbComType.Text + "！", null, Color.Black);
                        AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                        lbCurState.Text = "设备状态：空闲";
                        tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：打开";
                    }
                    else
                    {
                        btPortCtrl.Text = "打开端口";
                        cmbPort.Enabled = true;
                        cmbComType.Enabled = true;
                        AddStringToCommBox(true, "打开" + cmbPort.Text + "端口失败！", null, Color.Red);
                        AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                        lbCurState.Text = "设备状态：通信端口关闭";
                        tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：关闭";
                    }
                }
                else
                {
                    port_Ctrl(false);
                    btPortCtrl.Text = "打开端口";
                    cmbPort.Enabled = true;
                    cmbComType.Enabled = true;
                    AddStringToCommBox(true, "关闭" + cmbPort.Text + "端口成功！", null, Color.Black);
                    AddStringToCommBox(false, "\n****************************************", null, Color.Black);
                    lbCurState.Text = "设备状态：通信端口关闭";
                    tsComPortInfo.Text = "通信端口：" + cmbPort.Text + " 通信类型：" + cmbComType.Text + " 状态：关闭";
                }
            }
        }
        private bool port_Ctrl(bool ctrl)
        {
            string strBaudrate;

            if (true == ctrl)
            {
                if (cmbComType.Text == "串口通信")
                {
                    strBaudrate = Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/System/UartBaudrate", "115200");
                }
                else if (cmbComType.Text == "无线通信")
                {
                    strBaudrate = Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/System/RfBaudrate", "9600");
                }
                else// if (cmbComType.Text == "485通信")
                {
                    strBaudrate = Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/System/485Baudrate", "9600");
                }
                if (serialPort.IsOpen == false ||
                    serialPort.BaudRate != Convert.ToInt32(strBaudrate) ||
                    serialPort.PortName != cmbPort.Text)
                {
                    try
                    {
                        serialPort.Close();
                        serialPort.BaudRate = Convert.ToInt32(strBaudrate);
                        serialPort.PortName = cmbPort.Text;
                        serialPort.Open();
                        return true;
                    }
                    catch (System.Exception ex)
                    {
                        MessageBox.Show("打开通信端口时出现问题:" + ex.Message, "发生错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        return false;
                    }
                }
            }
            else
            {
                try
                {
                    serialPort.DiscardInBuffer();
                    serialPort.DiscardOutBuffer();
                    serialPort.Close();
                    return true;
                }
                catch (System.Exception ex)
                {
                    MessageBox.Show("关闭通信端口时出现问题:" + ex.Message, "发生错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return false;
                }
            }
            return true;
        }
        private void serialPort_DataTransmit(byte[] buf, int len)
        {
            try
            {
                serialPort.Write(buf, 0, len);
            }
            catch (System.Exception ex)
            {
                Command = enCmd.指令空闲;
                MessageBox.Show("串口通信出现异常:" + ex.Message, "出错啦", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
        private void serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int iRead;

            if (InvokeRequired)
            {
                try
                {
                    Invoke(new SerialDataRecievedEventHandler(serialPort_DataReceived), new object[] { sender, e });
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
            else
            {
                try
                {
                    while (serialPort.BytesToRead > 0)
                    {
                        iRead = serialPort.ReadByte();
                        PortRxBuf[PortBufWrPos] = (byte)iRead;
                        PortBufWrPos = (PortBufWrPos + 1) % PortRxBuf.Length;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
        }
        #endregion

        #region USB通信部分
        protected Thread ConnectUsbThread = null;
        private bool _isConnect = false;
        public bool isConnect
        {
            set
            {
                _isConnect = value;
                SetConnectStatusText(value);
            }
            get { return _isConnect; }
        }
        protected void SetConnectStatusText(bool isSuccess)
        {
            if (this.InvokeRequired)
            {
                SetConnectStatusTextCallBack st = new SetConnectStatusTextCallBack(SetConnectStatusText);
                this.Invoke(st, new object[] { isSuccess });
            }
            else
            {
                if (isSuccess == false)
                {
                    if (cmbPort.Text == "USB" && btPortCtrl.Text == "关闭端口")
                    {
                        MessageBox.Show("USB通信出现异常！", "USB端错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        btPortCtrl.Text = "打开端口";
                        cmbPort.Enabled = true;
                        cmbComType.Enabled = true;
                    }
                }
            }
        }
        protected delegate void SetConnectStatusTextCallBack(bool isSuccess);
        void CreateUsbThread()
        {
            ConnectUsbThread = new Thread(ConnectUsbWork);
            ConnectUsbThread.IsBackground = true;
            ConnectUsbThread.Priority = ThreadPriority.Normal;
            ConnectUsbThread.Start();
        }
        void ConnectUsbWork()
        {
            while (true)
            {
                ConnectUsb();
                Thread.Sleep(2000);
            }
        }
        private delegate void ConnectUsbCallBack(); //连接USB
        protected void ConnectUsb()
        {
            if (this.InvokeRequired)//等待异步
            {
                ConnectUsbCallBack fc = new ConnectUsbCallBack(ConnectUsb);
                this.Invoke(fc); //通过代理调用刷新方法
            }
            else
            {
                try
                {
                    SetErrorText("");
                    this.usbHidPort.ProductId = Convert.ToInt32("5750", 16);
                    this.usbHidPort.VendorId = Convert.ToInt32("0483", 16);
                    this.usbHidPort.CheckDevicePresent();
                }
                catch (Exception ex)
                {
                    SetErrorText(ex.ToString());
                }
            }
        }
        private void usbHidPort_OnSpecifiedDeviceArrived(object sender, EventArgs e)
        {
            isConnect = true;
        }
        private void usbHidPort_OnSpecifiedDeviceRemoved(object sender, EventArgs e)
        {
            isConnect = false;
        }
        private void usbHidPort_OnDataRecieved(object sender, DataRecievedEventArgs args)
        {
            int loop;

            if (InvokeRequired)
            {
                try
                {
                    Invoke(new DataRecievedEventHandler(usbHidPort_OnDataRecieved), new object[] { sender, args });
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            }
            else
            {
                for (loop = 0; loop < 64; loop++)
                {
                    PortRxBuf[PortBufWrPos] = args.data[loop + 1];
                    PortBufWrPos = (PortBufWrPos + 1) % PortRxBuf.Length;
                }
            }
        }
        public void usbHidPort_OnDataTransmit(byte[] buf, int len)
        {
            byte[] usbBuf = new byte[65];
            byte loop = 0;

            if (isConnect)
            {
                loop = (byte)((len + 1) / 64);
                if (((len + 1) % 64) != 0)
                {
                    loop++;
                }
                if (this.usbHidPort.SpecifiedDevice != null)
                {
                    for (byte i = 0; i < loop; i++)
                    {
                        Array.Clear(usbBuf, 0, usbBuf.Length);
                        for (int j = 0; j < 64; j++)
                        {
                            if (i * 64 + j >= buf.Length)
                            {
                                break;
                            }
                            usbBuf[j + 1] = buf[i * 64 + j];
                        }
                        this.usbHidPort.SpecifiedDevice.SendData(usbBuf);
                    }
                }
            }
            else
            {
                //MessageBox.Show(WindowHandler.GetLangValue("USBNOTCONNECTED"),
                //             Util.SystemTitle,
                //              MessageBoxButtons.OK,
                //             MessageBoxIcon.Error
                //              );
                //MessageBoxEx.Show(WindowHandler.GetLangValue("USBNOTCONNECTED"),
                //       Util.SystemTitle, MessageBoxIcon.Error);
            }
        }
        private delegate void SetErrorTextCallBack(string MsgText);
        protected void SetErrorText(string MsgText)
        {
            if (this.InvokeRequired)
            {
                SetErrorTextCallBack st = new SetErrorTextCallBack(SetErrorText);
                this.Invoke(st, new object[] { MsgText });
            }
            else
            {
                //this.tsConnectStatus.Text = MsgText;
            }
        }
        #endregion

        #region 定时器
        private bool flag = false;
        private void timer_Tick(object sender, EventArgs e)
        {
            int len, sum, loop;

            if (Command != enCmd.指令空闲)
            {
                if (prgBar.Value < prgBar.Maximum)
                {
                    prgBar.Value++;
                }
                else
                {
                    if (flag == false && (
                        Command == enCmd.批量读取集中器中的冻结数据 ||
                        Command == enCmd.批量读取集中器中的定时定量数据 ||
                        Command == enCmd.批量读设备自定义路由信息 ||
                        Command == enCmd.批量写设备自定义路由信息 ||
                        Command == enCmd.读表具档案信息 ||
                        Command == enCmd.写表具档案信息 ||
                        Command == enCmd.集中器数据转发))
                    {
                        flag = true;
                        LastCommand = Command;
                        if (DialogResult.Retry == MessageBox.Show("通信失败，继续重试还是取消任务？", "通信中断", MessageBoxButtons.RetryCancel, MessageBoxIcon.Question))
                        {
                            flag = false;
                            if (enCmd.批量读取集中器中的冻结数据 == LastCommand)
                            {
                                ReadAllFreezeDataProc();
                            }
                            else if (enCmd.批量读取集中器中的定时定量数据 == LastCommand)
                            {
                                ReadAllRealDataProc();
                            }
                            else if (enCmd.批量读设备自定义路由信息 == LastCommand)
                            {
                                ReadRouteFromDev();
                            }
                            else if (enCmd.批量写设备自定义路由信息 == LastCommand)
                            {
                                WriteRouteToDevProc();
                            }
                            else if (enCmd.写表具档案信息 == LastCommand)
                            {
                                WriteDocToDev();
                            }
                            else if (enCmd.读表具档案信息 == LastCommand)
                            {
                                ReadDocFromDev();
                            }
                            else if (enCmd.集中器数据转发 == LastCommand)
                            {
                                RunCmdProc();
                            }
                            else if (enCmd.批量写集中器中的下发命令数据 == LastCommand)
                            {
                                RunLssuedCmdProc();
                            }
                            return;
                        }
                    }
                    if (Command == enCmd.集中器数据转发)
                    {
                        btRunCmd.Text = "执行命令";
                        gpbFunList.Enabled = true;
                        cbRunNodeAddr.Enabled = true;
                        cmbOperateCmd.Enabled = true;
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    AddStringToCommBox(false, "\n执行结果：失败", null, Color.Red);
                }
            }

            if (_50MsTimer++ > 50 / timer.Interval)
            {
                _50MsTimer = 0;
                if (ReceiveWDT > 0)
                {
                    ReceiveWDT--;
                    if (ReceiveWDT == 0)
                    {
                        PortBufRdPos = (UInt16)((PortBufRdPos + 1) % PortRxBuf.Length);
                    }
                }
                if (strDataFwdNodeAddrList != null && strDataFwdNodeAddrList.Length > 0)
                {
                    btRunCmd.Enabled = true;
                    if (strDataFwdNodeAddrList.Length != iLastDataFwdNodeCount)
                    {
                        iLastDataFwdNodeCount = strDataFwdNodeAddrList.Length;
                        lbNodeAddrList.Text = "表具地址列表[" + strDataFwdNodeAddrList.Length.ToString("D") + "]";
                        cbRunNodeAddr.Items.Clear();
                        cbRunNodeAddr.Items.AddRange(strDataFwdNodeAddrList);
                        cbRunNodeAddr.SelectedIndex = 0;

                        lbCmdNodeAddrList.Text = "表具地址列表[" + strDataFwdNodeAddrList.Length.ToString("D") + "]";
                        cbRunCmdNodeAddr.Items.Clear();
                        cbRunCmdNodeAddr.Items.AddRange(strDataFwdNodeAddrList);
                        cbRunCmdNodeAddr.SelectedIndex = 0;

                        for (int iRow = 0; iRow < dgvDoc.Rows.Count; iRow++)
                        {
                            if (strDataFwdNodeAddrList[0] == dgvDoc.Rows[iRow].Cells["表具地址"].Value.ToString())
                            {
                                ReadRealData.TimeSlot.TableNumber = iRow+1;
                                ReadRealData.TimeSlot.TotalTableNumber = dgvDoc.Rows.Count;
                                break;
                            }
                        }
                    }
                }
                else if (cbRunNodeAddr.Text.Length > 0)
                {

                }
                else if (cbRunCmdNodeAddr.Text.Length > 0)
                {

                }
                else
                {
                    btRunCmd.Enabled = false;
                    cbRunNodeAddr.Items.Clear();
                    cbRunCmdNodeAddr.Items.Clear();
                    lbNodeAddrList.Text = "表具地址列表[0]";
                    lbCmdNodeAddrList.Text = "表具地址列表[0]";
                    ReadRealData.TimeSlot.TableNumber = 1;
                    ReadRealData.TimeSlot.TotalTableNumber = 1;
                }
                while (true)
                {
                    len = (PortBufWrPos >= PortBufRdPos) ? (PortBufWrPos - PortBufRdPos) : (PortRxBuf.Length - PortBufRdPos + PortBufWrPos);
                    if (len < 4)
                    {
                        break;
                    }
                    if (cmbPort.Text == "" || btPortCtrl.Text == "打开端口")
                    {
                        PortBufRdPos = PortBufWrPos = 0;
                        break;
                    }

                    if (PortRxBuf[PortBufRdPos % PortRxBuf.Length] != SyncWord0 || PortRxBuf[(PortBufRdPos + 1) % PortRxBuf.Length] != SyncWord1)
                    {
                        PortBufRdPos = (UInt16)((PortBufRdPos + 1) % PortRxBuf.Length);
                        continue;
                    }
                    sum = PortRxBuf[(PortBufRdPos + 2) % PortRxBuf.Length] + (PortRxBuf[(PortBufRdPos + 3) % PortRxBuf.Length] & 0x03) * 256 + 2;
                    if (ReceiveWDT == 0)
                    {
                        ReceiveWDT = 50;
                    }
                    if (sum > 1000)
                    {
                        PortBufRdPos = (UInt16)((PortBufRdPos + 1) % PortRxBuf.Length);
                        continue;
                    }
                    if (cmbComType.Text != "无线通信")
                    {
                        sum += 2;
                    }
                    if (len < sum)
                    {
                        break;
                    }
                    byte[] rxBuf = new Byte[sum];
                    ReceiveWDT = 0;
                    for (loop = 0; loop < sum; loop++)
                    {
                        rxBuf[loop] = PortRxBuf[(PortBufRdPos + loop) % PortRxBuf.Length];
                    }
                    PortBufRdPos = (PortBufRdPos + loop) % PortRxBuf.Length;
                    ExplainPacket(rxBuf);
                }
            }
            if (_500MsTimer++ > 500 / timer.Interval)
            {
                _500MsTimer = 0;
                lbCurTime.Text = DateTime.Now.ToString(" yyyy年MM月dd日 HH:mm:ss  ");        
            }
        }
        #endregion

        #region 共用函数
        private bool DeviceStatus(bool bCheckPort, bool bCheckCmd, bool bCheckConcAddr)
        {
            if (true == bCheckPort && btPortCtrl.Text == "打开端口")
            {
                MessageBox.Show("请先打开通信端口!", "警告", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return false;
            }
            if (true == bCheckCmd && enCmd.指令空闲 != Command)
            {
                MessageBox.Show("当前指令运行中,请稍后再试!", "警告", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return false;
            }
            if (true == bCheckConcAddr && tbConcAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请先读出集中器编号，然后再试！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return false;
            }
            return true;
        }
        public static string GetStringAddrFromByte(byte[] DataBuf, int iStart)
        {
            string strNodeAddr = "";
            for (int iLoop = 0; iLoop < AddrLength; iLoop++)
            {
                strNodeAddr += DataBuf[iStart + iLoop].ToString("X2");
            }
            return strNodeAddr;
        }
        private int GetByteAddrFromString(string strSource, byte[] DataBuf, int iStart)
        {
            int iPos = iStart;
            for (int iLoop = 0; iLoop < strSource.Length; )
            {
                DataBuf[iPos++] = Convert.ToByte(strSource.Substring(iLoop, 2), 16);
                iLoop += 2;
            }
            return (iPos - iStart);
        }
        private string GetDevTypeFromByte(byte TypeId)
        {
            switch (TypeId)
            {
                case 0x10: return "冷水表";
                case 0x20: return "热水表";
                case 0x30: return "燃气表";
                case 0x40: return "电能表";
                case 0xFD: return "中继器";
                default: return "未知类型";
            }
        }
        private byte GetDevTypeFromString(string strDevType)
        {
            switch (strDevType)
            {
                case "冷水表": return 0x10;
                case "热水表": return 0x20;
                case "燃气表": return 0x30;
                case "电能表": return 0x40;
                case "中继器": return 0xFD;
                default: return 0xFF;
            }
        }
        public static string GetErrorInfo(byte ErrCode)
        {
            switch (ErrCode)
            {
                case 0xAA: return "操作成功";
                case 0xAB: return "操作失败";
                case 0xAC: return "通讯失败";
                case 0xAD: return "命令下达成功";
                case 0xAE: return "数据格式错误";
                case 0xAF: return "时间异常";
                case 0xBA: return "对象不存在";
                case 0xBB: return "对象重复";
                case 0xBC: return "对象已满";
                case 0xBD: return "参数错误";
                case 0xCC: return "超时错误";
                case 0xCD: return "单轮运行超时错误";
                case 0xCE: return "正在执行";
                case 0xCF: return "操作已处理";
                case 0xD0: return "已应答";
                case 0xD1: return "抄读表数据错误";
                case 0xD2: return "没有此项功能";
                default: return "未知错误";
            }
        }
        private byte CalCRC8(byte[] dataBuf, int pos, int dataLen)
        {
            byte i, cCrc = 0;
            for (int m = 0; m < dataLen; m++)
            {
                cCrc ^= dataBuf[m + pos];
                for (i = 0; i < 8; i++)
                {
                    if ((cCrc & 0x01) != 0)
                    {
                        cCrc >>= 1;
                        cCrc ^= 0x8c;
                    }
                    else
                    {
                        cCrc >>= 1;
                    }
                }
            }
            return cCrc;
        }
        private int CalCRC16(byte[] dataBuf, int pos, int dataLen)
        {
            int i, cCrc = 0xffff;
            for (int m = 0; m < dataLen; m++)
            {
                cCrc ^= dataBuf[m + pos];
                for (i = 0; i < 8; i++)
                {
                    if ((cCrc & 0x0001) != 0)
                    {
                        cCrc >>= 1;
                        cCrc ^= 0x8408;
                    }
                    else
                    {
                        cCrc >>= 1;
                    }
                }
            }
            cCrc ^= 0xFFFF;
            return cCrc;
        }
        public static byte Hex2Bcd(byte hexVal)
        {
            byte a, b;
            a = (byte)(hexVal / 10);
            b = (byte)(hexVal % 10);
            return (byte)(a << 4 | b);
        }
        public static byte Bcd2Hex(byte bcdVal)
        {
            byte a, b;
            a = (byte)(bcdVal >> 4 & 0x0F);
            b = (byte)(bcdVal & 0x0F);
            return (byte)(a * 10 + b);
        }
        public static string Byte6ToUint64(byte[] SrcBuf, int StartIndex)
        {
            UInt64 integer = 0;
            UInt32 fraction = 0;

            for (int iLoop = 0; iLoop < 4; iLoop++)
            {
                integer <<= 8;
                integer += SrcBuf[StartIndex + 4 - iLoop - 1];
            }
            fraction = (UInt32)(SrcBuf[StartIndex + 4] + SrcBuf[StartIndex + 5] * 256);
            return (integer.ToString("D") + "." + fraction.ToString("D"));
        }
        private void tbAddress_KeyPress(object sender, KeyPressEventArgs e, int iLen, string strFilter)
        {
            TextBox tbAddress = (TextBox)sender;

            if (tbAddress.ReadOnly == true)
            {
                return;
            }
            if (strFilter.IndexOf(e.KeyChar) < 0)
            {
                e.Handled = true;
                return;
            }
            if (e.KeyChar == '\r')
            {
                tbAddress.Text = tbAddress.Text.PadLeft(iLen, '0');
                e.Handled = true;
            }
            if (tbAddress.Text.Length >= iLen && e.KeyChar != '\b')
            {
                if (tbAddress.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        #endregion

        #region 协议解析
        private byte[] ConstructTxBuffer(enCmd Cmd, byte Option, byte[] Path, byte[] DataField)
        {
            byte bCrc;
            int crc16;
            int iLen = 0;
            try
            {
                byte[] buf = new byte[1500];
                // 同步字
                buf[iLen++] = SyncWord0;
                buf[iLen++] = SyncWord1;
                // 包长度,后面填充
                iLen += 2;
                // 报文标识
                buf[iLen++] = Option;
                // 帧序号
                buf[iLen++] = bFrameSn++;
                // 命令字
                buf[iLen++] = (byte)Cmd;
                Command = Cmd;
                // 设备类型
                buf[iLen++] = 0xFA;
                // 生命周期
                buf[iLen++] = 0x3F;
                // 传输路径
                if (Path == null)
                {
                    buf[iLen++] = 0x02;
                    iLen += GetByteAddrFromString(strLocalAddr, buf, iLen);
                    iLen += GetByteAddrFromString(tbConcAddr.Text, buf, iLen);
                }
                else
                {
                    Array.Copy(Path, 0, buf, iLen, Path.Length);
                    iLen += Path.Length;
                }
                // 数据域
                if (DataField != null)
                {
                    Array.Copy(DataField, 0, buf, iLen, DataField.Length);
                    iLen += DataField.Length;
                }
                // 上行信号强度
                buf[iLen++] = 0;
                // 下行信号强度
                buf[iLen++] = 0;
                if (0x1 == (Option & 0x1))
                {
                    // 包长度
                    buf[2] = (byte)(iLen + 1);
                    buf[3] = (byte)((iLen + 1) >> 8);
                    // crc16校验,低字节在前
                    crc16 = CalCRC16(buf, 2, iLen - 2);
                    buf[iLen++] = (byte)((crc16) & 0xFF);
                    buf[iLen++] = (byte)((crc16 >> 8) & 0xFF);
                }
                else
                {
                    // 包长度
                    buf[2] = (byte)(iLen);
                    buf[3] = (byte)(iLen >> 8);
                    // 校验字
                    bCrc = CalCRC8(buf, 2, iLen - 2);
                    buf[iLen++] = bCrc;
                }
                // 结束符
                buf[iLen++] = 0x16;
                // 其他参数
                buf[iLen++] = 0x00;
                buf[iLen++] = 9 + 16;
                buf[iLen++] = 3;

                // 调整数据大小
                Array.Resize(ref buf, iLen);
                return buf;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }
            return null;
        }
        public void DataTransmit(byte[] dataBuf)
        {
            if (cmbComType.Text == "USB通信")
            {
                usbHidPort_OnDataTransmit(dataBuf, dataBuf.Length);
            }
            else
            {
                serialPort_DataTransmit(dataBuf, dataBuf.Length);
            }
        }
        private ProtocolStruct ExtractRxBuffer(byte[] Buffer)
        {
            ProtocolStruct proStruct = new ProtocolStruct();
            int iLoop, packPos, startPos;
            int calCrc;

            try
            {
                if (Buffer[0] == SyncWord0 && Buffer[1] == SyncWord1)
                {
                    startPos = 2;
                }
                else
                {
                    startPos = 0;
                }
                packPos = startPos;
                // 包长度
                proStruct.PacketLength = Buffer[packPos++] + (Buffer[packPos++] & 0x03) * 256;
                //报文标识
                proStruct.PacketProperty = Buffer[packPos++];
                // 任务号
                proStruct.TaskIDByte = Buffer[packPos++];
                // 命令字
                proStruct.CommandIDByte = Buffer[packPos++];
                // 设备类型
                proStruct.DeviceTypeByte = Buffer[packPos++];
                // 生命周期
                proStruct.LifeCycleByte = Buffer[packPos++];
                // 路径条数
                proStruct.RouteLevelByte = (Byte)(Buffer[packPos] & 0x0F);
                // 当前位置
                proStruct.RouteCurPos = (byte)(Buffer[packPos] >> 4 & 0x0F);
                // 传输路径
                packPos++;
                proStruct.RoutePathList = new byte[proStruct.RouteLevelByte * AddrLength];
                for (iLoop = 0; iLoop < proStruct.RoutePathList.Length; iLoop++)
                {
                    proStruct.RoutePathList[iLoop] = Buffer[packPos++];
                }
                if (0x1 == (proStruct.PacketProperty & 0x1))
                {//crc16
                    // 数据域
                    proStruct.DataBuf = new byte[proStruct.PacketLength - 5 - packPos + startPos];
                }
                else
                {//crc8
                    // 数据域
                    proStruct.DataBuf = new byte[proStruct.PacketLength - 4 - packPos + startPos];
                }
                for (iLoop = 0; iLoop < proStruct.DataBuf.Length; iLoop++)
                {
                    proStruct.DataBuf[iLoop] = Buffer[packPos++];
                }
                // 下行信号强度
                proStruct.DownSignalStrengthByte = Buffer[packPos++];
                // 上行信号强度
                proStruct.UpSignalStrengthByte = Buffer[packPos++];
                if (0x1 == (proStruct.PacketProperty & 0x1))
                {//crc16 低字节在前
                    // 校验字
                    proStruct.Crc16Byte = (Buffer[packPos++] & 0xff) | ((Buffer[packPos++] << 8) & 0xFF00);
                    // 结束字
                    proStruct.EndByte = Buffer[packPos++];
                    // 是否成功
                    calCrc = CalCRC16(Buffer, startPos, proStruct.PacketLength - 3);
                    proStruct.isSuccess = (calCrc == proStruct.Crc16Byte) ? true : false;
                }
                else
                {//crc8
                    // 校验字
                    proStruct.Crc8Byte = Buffer[packPos++];
                    // 结束字
                    proStruct.EndByte = Buffer[packPos++];
                    // 是否成功
                    calCrc = CalCRC8(Buffer, startPos, proStruct.PacketLength - 2);
                    proStruct.isSuccess = (calCrc == proStruct.Crc8Byte) ? true : false;
                }
                return proStruct;
            }
            catch
            {
                proStruct.isSuccess = false;
                return proStruct;
            }
        }
        private void ExplainPacket(byte[] RecBuf)
        {
            ProtocolStruct rxStruct = ExtractRxBuffer(RecBuf);            
            if (false == rxStruct.isSuccess)
            {
                AddStringToCommBox(true, "接收：", RecBuf, Color.Black);
                AddStringToCommBox(false, "<校验错误>", null, Color.Red);
                return;
            }
            else if (rxStruct.CommandIDByte == (byte)enCmd.集中器监控控制)
            {
                ExplainGprsMonitor(rxStruct);
                return;
            }
            AddStringToCommBox(true, "接收：", RecBuf, Color.Black);
            switch (Command)
            {
                case enCmd.读表具数据:
                    ExplainReadMeterData(rxStruct);
                    break;
                case enCmd.读集中器地址: 
                    ExplainReadConcAddr(rxStruct);
                    break;
                case enCmd.写集中器地址:
                    ExplainWriteConcAddr(rxStruct);
                    break;
                case enCmd.集中器重新启动:
                    ExplainRestartDev(rxStruct);
                    break;
                case enCmd.存储器检查:
                    ExplainMemCheck(rxStruct);
                    break;
                case enCmd.集中器版本信息:
                    ExplainDevVerInfo(rxStruct);
                    break;
                case enCmd.读集中器时钟:
                    ExplainReadRtc(rxStruct);
                    break;
                case enCmd.写集中器时钟:
                    ExplainSetRtc(rxStruct);
                    break;
                case enCmd.读集中器工作模式参数:
                    ExplainReadConcParam(rxStruct);
                    break;
                case enCmd.写集中器工作模式参数:
                    ExplainWriteConcParam(rxStruct);
                    break;
                case enCmd.读Gprs参数:
                    ExplainReadGprsParam(rxStruct);
                    break;
                case enCmd.写Gprs参数:
                    ExplainSetGprsParam(rxStruct);
                    break;
                case enCmd.读Gprs信号强度:
                    ExplainReadGprsInfo(rxStruct);
                    break;
                case enCmd.写集中器第二扫描信道:
                    ExplainWSecondChannel(rxStruct);
                    break;
                case enCmd.读集中器第二扫描信道:
                    ExplainRSecondChannel(rxStruct);
                    break;
                case enCmd.读表具档案信息:
                    ExplainReadDocFromDev(rxStruct);
                    break;
                case enCmd.写表具档案信息:
                    ExplainWriteDocToDev(rxStruct);
                    ExplainCreatNewNode(rxStruct);
                    break;
                case enCmd.批量读设备自定义路由信息:
                    ExplainReadRouteFromDev(rxStruct);
                    break;
                case enCmd.批量写设备自定义路由信息:
                    ExplainWriteRouteToDev(rxStruct);
                    break;
                case enCmd.集中器初始化:
                    ExplainFormatDoc(rxStruct);
                    ExplainFormatData(rxStruct);
                    break;
                case enCmd.读表具数量:
                    ExplainReadDevNodeCount(rxStruct);
                    break;
                case enCmd.删除表具档案信息:
                    ExplainDelNode(rxStruct);
                    break;
                case enCmd.修改表具档案信息:
                    ExplainModifyNode(rxStruct);
                    break;
                case enCmd.读表具自定义路由信息:
                    ExplainReadRoute(rxStruct);
                    break;
                case enCmd.写表具自定义路由信息:
                    ExplainWriteRoute(rxStruct);
                    break;
                case enCmd.集中器数据转发:
                    ExplainRunCmd(rxStruct);
                    break;
                case enCmd.读集中器中的定时定量数据:
                    ExplainReadRealData(rxStruct);
                    break;
                case enCmd.读集中器中的冻结数据:
                    ExplainReadFreezeData(rxStruct);
                    break;
                case enCmd.批量读取集中器中的定时定量数据:
                    ExplainReadAllRealData(rxStruct);
                    break;
                case enCmd.批量读取集中器中的冻结数据:
                    ExplainReadAllFreezeData(rxStruct);
                    break;
                case enCmd.读集中器中的下发命令数据:
                    ExplainReadCmdData(rxStruct);
                    break;
                case enCmd.批量读取集中器中的下发命令数据:
                    ExplainReadAllCmdData(rxStruct);
                    break;
                case enCmd.批量写集中器中的下发命令数据:
                    ExplainBatchWriteMeterCmd(rxStruct);
                    break;
                    
                default:
                    //ExplainOtherData(rxStruct);
                    break;
            }
        }
        #endregion

        #region 通讯信息
        private void ProgressBarCtrl(int iValue, int iMinValue, int iMaxValue)
        {
            prgBar.Minimum = iMinValue;
            prgBar.Maximum = iMaxValue;
            if (iValue >= iMinValue && iValue <= iMaxValue)
            {
                prgBar.Value = iValue;
            }
        }
        private void AddStringToCommBox(bool bNeedTime, string strInfo, byte[] buf, Color strColor, bool bClear = false)
        {
            int iLoop, iStart;

            if (bClear == true && tsmiAutoClear.Checked == true)
            {
                rtbCommMsg.Clear();
            }
            if (rtbCommMsg.Text.Length > rtbCommMsg.MaxLength - 100)
            {
                rtbCommMsg.Clear();
            }
            iStart = rtbCommMsg.Text.Length;
            if (true == bNeedTime)
            {
                rtbCommMsg.AppendText("\n[" + DateTime.Now.ToString("yyyy/MM/dd HH:mm:ss:fff") + "]");
            }
            if (null != buf)
            { 
                for (iLoop = 0; iLoop < buf.Length; iLoop++)
                {
                    strInfo += buf[iLoop].ToString("X2") + " ";
                }
                strInfo.Trim();
            }
            rtbCommMsg.AppendText(strInfo);
            rtbCommMsg.Select(iStart, rtbCommMsg.Text.Length);
            rtbCommMsg.SelectionColor = strColor;
            if (tsmiAutoScroll.Checked == true)
            {
                rtbCommMsg.ScrollToCaret();
            }
        }
        private void cntMenuStripCommInfo_Opening(object sender, CancelEventArgs e)
        {
            tsmiAutoScroll.Enabled = true;
            if ("1" == Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/CommonMsgAutoScroll", "1"))
            {
                tsmiAutoScroll.Checked = true;
            }
            else
            {
                tsmiAutoScroll.Checked = false;
            }
            if ("1" == Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/CommonMsgAutoClear", "0"))
            {
                tsmiAutoClear.Checked = true;
            }
            else
            {
                tsmiAutoClear.Checked = false;
            }
            if (rtbCommMsg.Text.Length == 0)
            {
                tsmiClearAll.Enabled = false;
                tsmiSaveRecord.Enabled = false;
            }
            else
            {
                tsmiClearAll.Enabled = true;
                tsmiSaveRecord.Enabled = true;
            }
        }
        private void tsmiClearAll_Click(object sender, EventArgs e)
        {
            rtbCommMsg.Clear();
        }
        private void tsmiAutoScroll_Click(object sender, EventArgs e)
        {
            Common.XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "CommonMsgAutoScroll", (tsmiAutoScroll.Checked == true ? "1" : "0"));
        }
        private void tsmiAutoClear_Click(object sender, EventArgs e)
        {
            Common.XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "CommonMsgAutoClear", (tsmiAutoClear.Checked == true ? "1" : "0"));
        }
        private void tsmiSaveRecord_Click(object sender, EventArgs e)
        {
            string strDirectory = "";
            string strFileName;

            if (rtbCommMsg.Text.Length == 0)
            {
                MessageBox.Show("没有通讯数据可以保存！", "错误");
                return;
            }
            strDirectory = Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/SaveCommMsgPath", System.Windows.Forms.Application.StartupPath);
            saveFileDlg.Filter = "*.txt(文本文件)|*.txt";
            saveFileDlg.DefaultExt = "txt";
            saveFileDlg.FileName = "通信记录_" + tbConcAddr.Text + "_" + DateTime.Now.ToString("yyMMddHHmm");
            if (saveFileDlg.ShowDialog() == DialogResult.Cancel)
            {
                return;
            }
            strFileName = saveFileDlg.FileName;
            if (strFileName.Length == 0)
            {
                return;
            }
            if (strDirectory != Path.GetDirectoryName(strFileName))
            {
                strDirectory = Path.GetDirectoryName(strFileName);
                Common.XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "SaveCommMsgPath", strDirectory);
            }
            try
            {
                StreamWriter sw = new StreamWriter(strFileName, true, System.Text.Encoding.UTF8);
                sw.WriteLine("\n******以下记录保存时间是" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "******");
                string strTemp = rtbCommMsg.Text.Replace("\n", "\r\n");
                sw.Write(strTemp);
                sw.Close();
            }
            catch (SystemException ex)
            {
                Console.WriteLine(ex.ToString());
            }
        }
        #endregion

        #region 参数管理选项卡
        #region 读取集中器编号
        private void btReadConcAddr_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, false))
            {
                return;
            }
            int iLen = 0;
            byte[] path = new byte[AddrLength * 2 + 1];
            path[iLen++] = 0x02;
            iLen += GetByteAddrFromString(strLocalAddr, path, iLen);
            for (int iLoop = 0; iLoop < AddrLength; iLoop++)
            {
                path[iLen++] = 0xD5;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.读集中器地址, NeedAck, path, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读集中器编号----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读集中器编号";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainReadConcAddr(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器地址 )
                {
                    tbConcAddr.Text = GetStringAddrFromByte(rxStruct.DataBuf, 0);
                    AddStringToCommBox(false, "\n执行结果：成功  集中器地址为：" + tbConcAddr.Text, null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    scon1.Panel2.Enabled = true;
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取集中器编号出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 设置集中器编号
        private void btSetConcAddr_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbNewConcAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请输入新的集中器编号后再试！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("请确认是否要修改集中器地址吗？\n注意：修改集中器地址要通过串口或USB接口进行。", "修改确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            byte[] dataBuf = new byte[AddrLength];
            GetByteAddrFromString(tbNewConcAddr.Text, dataBuf, 0);
            byte[] txBuf = ConstructTxBuffer(enCmd.写集中器地址, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------设置集中器编号----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：设置集中器编号";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainWriteConcAddr(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写集中器地址)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    { 
                        tbConcAddr.Text = tbNewConcAddr.Text;
                        AddStringToCommBox(false, "\n执行结果：成功  新的集中器编号为：" + tbConcAddr.Text, null, Color.DarkBlue);
                        AddStringToCommBox(false, " 集中器将在10秒钟内重新启动！", null, Color.Red);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                        AddStringToCommBox(false, " 只能通过调试串口或USB接口修改集中器地址！", null, Color.Blue);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("设置集中器编号出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 重新启动集中器
        private void btRestartDev_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否要重新启动集中器？", "操作确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.集中器重新启动, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------重新启动集中器----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：重新启动集中器";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainRestartDev(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.集中器重新启动)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, "\n执行结果：成功  集中器将在10秒钟内重新启动", null, Color.DarkBlue);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                    }
                    lbCurState.Text = "设备状态：空闲";
                    Command = enCmd.指令空闲;
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("集中器重新启动出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 存储器检查
        private void btMemCheck_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("存储器检查要耗费较长时间，确定执行吗？", "等待确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.存储器检查, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------存储器检查----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：存储器检查";
            ProgressBarCtrl(0, 0, 60000 / timer.Interval);
        }
        private void ExplainMemCheck(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.存储器检查)
                {
                    string strInfo = "";
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        strInfo = rxStruct.DataBuf[1] == 0 ? "64K字节" : "128K字节";
                        AddStringToCommBox(false, "\n执行结果：成功  ", null, Color.DarkBlue);
                        AddStringToCommBox(false, "存储器容量为" + strInfo, null, Color.Red);
                        MessageBox.Show("检测完毕，存储器容量为" + strInfo, "检测结果", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                        strInfo = rxStruct.DataBuf[5].ToString("X2") + rxStruct.DataBuf[4].ToString("X2") + rxStruct.DataBuf[3].ToString("X2") + rxStruct.DataBuf[2].ToString("X2");
                        AddStringToCommBox(false, "  在检测到0x" + strInfo + "位置处发生错误!", null, Color.Red);
                        MessageBox.Show("检测错误，在检测到0x" + strInfo + "位置处发生错误!", "检测结果", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    lbCurState.Text = "设备状态：空闲";
                    Command = enCmd.指令空闲;
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("存储器检查出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 读取集中器版本信息
        private void btReadDevVerInfo_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.集中器版本信息, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取集中器版本信息----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取集中器版本信息";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainDevVerInfo(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.集中器版本信息)
                {
                    lbVerInfo.Text = "软件:" + rxStruct.DataBuf[0].ToString("X") + "." + rxStruct.DataBuf[1].ToString("X");
                    lbVerInfo.Text += " 硬件:" + rxStruct.DataBuf[2].ToString("X") + "." + rxStruct.DataBuf[3].ToString("X");
                    lbVerInfo.Text += " 协议:" + rxStruct.DataBuf[4].ToString("X") + "." + rxStruct.DataBuf[5].ToString("X");
                    AddStringToCommBox(false, "\n执行结果：成功  集中器版本信息为  " + lbVerInfo.Text, null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取集中器版本信息出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 读取集中器时钟
        private void btReadRtc_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.读集中器时钟, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取集中器时钟----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取集中器时钟";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainReadRtc(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器时钟)
                {
                    DateTime rdTime = new DateTime(Bcd2Hex(rxStruct.DataBuf[0]) * 100 + Bcd2Hex(rxStruct.DataBuf[1]), Bcd2Hex(rxStruct.DataBuf[2]), Bcd2Hex(rxStruct.DataBuf[3]),
                                               Bcd2Hex(rxStruct.DataBuf[4]), Bcd2Hex(rxStruct.DataBuf[5]), Bcd2Hex(rxStruct.DataBuf[6]), 0);
                    dtpDate.Value = rdTime;
                    dtpTime.Value = rdTime;
                    AddStringToCommBox(false, "\n执行结果：成功  集中器时钟为：" + rdTime.ToString(), null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取集中器时钟信息出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 设置集中器时钟
        private void btSetRtc_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("确认要设置集中器的时钟吗？", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            if ("系统当前时间" == cmbTimeType.Text)
            {
                string strTime = DateTime.Now.ToString("yyyyMMddHHmmssfff");
                dtpDate.Value = DateTime.Now;
                dtpTime.Value = DateTime.Now;
            }
            byte[] dataBuf = new byte[7];
            dataBuf[0] = Hex2Bcd((byte)(dtpDate.Value.Year / 100));
            dataBuf[1] = Hex2Bcd((byte)(dtpDate.Value.Year % 100));
            dataBuf[2] = Hex2Bcd((byte)(dtpDate.Value.Month));
            dataBuf[3] = Hex2Bcd((byte)(dtpDate.Value.Day));
            dataBuf[4] = Hex2Bcd((byte)(dtpTime.Value.Hour));
            dataBuf[5] = Hex2Bcd((byte)(dtpTime.Value.Minute));
            dataBuf[6] = Hex2Bcd((byte)(dtpTime.Value.Second));
            byte[] txBuf = ConstructTxBuffer(enCmd.写集中器时钟, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------设置集中器时钟----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：设置集中器时钟";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainSetRtc(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写集中器时钟)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, "\n执行结果：成功  注意集中器可能会启动数据补抄或数据上传任务", null, Color.DarkBlue);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("设置集中器时钟信息出错：" + ex.Message);
            }
            return;
        } 
        private void cmbTimeType_SelectedIndexChanged(object sender, EventArgs e)
        {
            if ("系统当前时间" == cmbTimeType.Text)
            {
                dtpDate.Enabled = false;
                dtpTime.Enabled = false;
                string strTime = DateTime.Now.ToString("yyyyMMddHHmmssfff");
                dtpDate.Value = DateTime.Now;
                dtpTime.Value = DateTime.Now;
            }
            else if("自定义时间" == cmbTimeType.Text)
            {
                dtpDate.Enabled = true;
                dtpTime.Enabled = true;
            }
        }
        #endregion
        #region 读取集中器工作参数
        private void btReadConcParam_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.读集中器工作模式参数, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取集中器工作参数----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取集中器工作参数";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainReadConcParam(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器工作模式参数)
                {
                    AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                    if (rxStruct.DataBuf[0] == 0)
                    {
                        rdbRealTimeMode.Checked = true;
                        AddStringToCommBox(false, "\n  数据类型：定时定量模式", null, Color.DarkBlue);
                    }
                    else if (rxStruct.DataBuf[0] == 1)
                    {
                        rdbFreezeMode.Checked = true;
                        AddStringToCommBox(false, "\n  数据类型：冻结数据模式", null, Color.DarkBlue);
                    }
                    else
                    {
                        rdbRealTimeMode.Checked = false;
                        rdbFreezeMode.Checked = false;
                        AddStringToCommBox(false, "\n  数据类型：错误的数据类型", null, Color.Red);
                    }
                    if ((rxStruct.DataBuf[1] & 0x20) == 0x20)
                    {
                        rdbEncryptData.Checked = true;
                        AddStringToCommBox(false, "\n  数据加密：开启", null, Color.DarkBlue);
                    }
                    else
                    {
                        rdbDisCryptData.Checked = true;
                        AddStringToCommBox(false, "\n  数据加密：关闭", null, Color.DarkBlue);
                    }
                    nudUploadStartHour.Value = Bcd2Hex(rxStruct.DataBuf[2]);
                    if ((rxStruct.DataBuf[1] & 0x40) == 0x40)
                    {
                        rdbDataUploadEnable.Checked = true;
                        AddStringToCommBox(false, "\n  主动上传：开启  时间参数：" + nudUploadStartHour.Value.ToString() + "时", null, Color.DarkBlue);
                    }
                    else
                    {
                        rdbDataUploadDisable.Checked = true;
                        AddStringToCommBox(false, "\n  主动上传：关闭  时间参数：" + nudUploadStartHour.Value.ToString() + "时", null, Color.DarkBlue);
                    }
                    if ((rxStruct.DataBuf[1] & 0x10) == 0x00)
                    {
                        upRealTimeMode.Checked = true;
                        AddStringToCommBox(false, "\n  上传类型：定时定量", null, Color.DarkBlue);
                    }
                    else if ((rxStruct.DataBuf[1] & 0x10) == 0x10)
                    {
                        upFreezeMode.Checked = true;
                        AddStringToCommBox(false, "\n  上传类型：冻结", null, Color.DarkBlue);
                    }
                    if ((rxStruct.DataBuf[1] & 0x80) == 0x80)
                    {
                        rdbDataReplenishEnable.Checked = true;
                        AddStringToCommBox(false, "\n  数据补抄：开启  ", null, Color.DarkBlue);
                    }
                    else
                    {
                        rdbDataReplenishDisable.Checked = true;
                        AddStringToCommBox(false, "\n  数据补抄：禁能  ", null, Color.DarkBlue);
                    }


                    UInt32 bitMask = (UInt32)((rxStruct.DataBuf[3] << 24) | (rxStruct.DataBuf[4] << 16) | ((rxStruct.DataBuf[5] << 8)) | rxStruct.DataBuf[6]);
                    string strData = "";
                    for (int iLoop = 1; iLoop < 32; iLoop++)
                    {
                        if ((bitMask & (1 << 32 - iLoop - 1)) == 0)
                        {
                            lvReadDate.Items[iLoop - 1].Checked = false;
                        }
                        else
                        {
                            lvReadDate.Items[iLoop - 1].Checked = true;
                            strData += iLoop.ToString() + ",";
                        }
                    }
                    nudReplenishStartHour.Value = Bcd2Hex(rxStruct.DataBuf[7]);
                    nudReplenishRetry.Value = Bcd2Hex(rxStruct.DataBuf[8]);
                    AddStringToCommBox(false, "补抄次数：" + nudReplenishRetry.Value.ToString() + "次  " + "补抄时间：" + nudReplenishStartHour.Value.ToString() + "时", null, Color.DarkBlue);
                    if (strData == "")
                    {
                        strData = "未设置";
                    }
                    AddStringToCommBox(false, "\n  补抄日期：" + strData.TrimEnd(','), null, Color.DarkBlue);

                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读集中器工作模式参数出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 设置集中器工作参数
        private void btSetConcParam_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("请确认是否要修改集中器的工作参数？\n注意：数据类型只能通过串口或USB接口修改。", "修改确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            byte[] dataBuf = new byte[9];
            if (rdbRealTimeMode.Checked == true)
            {
                dataBuf[0] = 0x00;
            }
            else if (rdbFreezeMode.Checked == true)
            {
                dataBuf[0] = 0x01;
            }
            else
            {
                MessageBox.Show("数据类型设置有错误，请重新设置！");
                return;
            }
            dataBuf[1] = 0;
            if (rdbEncryptData.Checked == true)
            {
                dataBuf[1] |= 0x20;
            }
            else if (rdbDisCryptData.Checked == true)
            {
            }
            else
            {
                MessageBox.Show("数据加密设置有错误，请重新设置！");
                return;
            }
            if (rdbDataUploadEnable.Checked == true)
            {
                dataBuf[1] |= 0x40;
            }
            else if (rdbDataUploadDisable.Checked == true)
            {
            }
            else
            {
                MessageBox.Show("数据上传设置有错误，请重新设置！");
                return;
            }
            if (upRealTimeMode.Checked == true)
            {
                dataBuf[1] |= 0x00;
            }
            else if (upFreezeMode.Checked == true)
            {
                dataBuf[1] |= 0x10;
            }
            else
            {
                MessageBox.Show("数据上传类型设置有错误，请重新设置！");
                return;
            }
            if (rdbDataReplenishEnable.Checked == true)
            {
                dataBuf[1] |= 0x80;
            }
            else if (rdbDataReplenishDisable.Checked == true)
            {
            }
            else
            {
                MessageBox.Show("数据补抄设置有错误，请重新设置！");
                return;
            }
            dataBuf[2] = Hex2Bcd((byte)nudUploadStartHour.Value);
            UInt32 bitMask = 0;
            for (int iLoop = 1; iLoop < 32; iLoop++ )
            {
                bitMask <<= 1;
                if (lvReadDate.Items[iLoop - 1].Checked == true)
                {
                    bitMask |= 1;
                }
            }
            dataBuf[3] = (byte)(bitMask >> 24);
            dataBuf[4] = (byte)(bitMask >> 16);
            dataBuf[5] = (byte)(bitMask >> 8);
            dataBuf[6] = (byte)bitMask;
            dataBuf[7] = Hex2Bcd((byte)nudReplenishStartHour.Value);
            dataBuf[8] = (byte)nudReplenishRetry.Value;
            
            byte[] txBuf = ConstructTxBuffer(enCmd.写集中器工作模式参数, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------设置集中器工作参数----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：写集中器工作模式参数";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainWriteConcParam(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写集中器工作模式参数)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                        AddStringToCommBox(false, " 只能通过调试串口或USB接口修改数据类型！", null, Color.Blue);
                    }
                    lbCurState.Text = "设备状态：空闲";
                    Command = enCmd.指令空闲;
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("写集中器工作模式参数出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 其他参数操作
        private void dataType_Click(object sender, EventArgs e)
        {
            RadioButton rbt = (RadioButton)sender;
            if (DialogResult.Cancel == MessageBox.Show("注意，修改数据类型要和通讯的子节点数据类型相匹配，否则会造成不可预料的结果！\n修改数据类型只能通过调试串口和USB接口操作，并在更改成功后自动重启集中器！", "特别注意", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                if (rbt.Name == "rdbRealTimeMode")
                {
                    rdbRealTimeMode.Checked = false;
                    rdbFreezeMode.Checked = true;
                }
                else
                {
                    rdbRealTimeMode.Checked = true;
                    rdbFreezeMode.Checked = false;
                }
                return;
            }        
        }
        private void tbConcAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
            if (tbConcAddr.Text.Length == AddrLength * 2)
            {
                scon1.Panel2.Enabled = true;
            }
            else
            {
                scon1.Panel2.Enabled = false;
            }
        }
        private void tbConcAddr_Leave(object sender, EventArgs e)
        {
            if (tbConcAddr.Text != "")
            {
                tbConcAddr.Text = tbConcAddr.Text.PadLeft(AddrLength * 2, '0');
                scon1.Panel2.Enabled = true;
            }
            else
            {
                scon1.Panel2.Enabled = false;
            }
        }
        private void tbNewConcAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
        }
        private void tbNewConcAddr_Leave(object sender, EventArgs e)
        {
            if (tbNewConcAddr.Text != "")
            {
                tbNewConcAddr.Text = tbNewConcAddr.Text.PadLeft(AddrLength * 2, '0');
            }
        }
        #endregion
        #endregion

        #region GPRS参数选项卡
        #region 读取GPRS参数
        private void btReadGprsParam_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.读Gprs参数, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取GPRS参数----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取GPRS参数";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainReadGprsParam(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读Gprs参数)
                {
                    AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                    int iLen = 0;
                    string strInfo = "\n  " + lbServerIP0.Text + "：";
                    tbServerIp00.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp00.Text + ".";
                    tbServerIp01.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp01.Text + ".";
                    tbServerIp02.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp02.Text + ".";
                    tbServerIp03.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp03.Text + "  ";
                    tbServerPort0.Text = (rxStruct.DataBuf[iLen++] + rxStruct.DataBuf[iLen++] * 256).ToString("D");
                    strInfo += lbPort0.Text + "：" + tbServerPort0.Text;
                    AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    strInfo = "\n  " + lbServerIP1.Text + "：";
                    tbServerIp10.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp10.Text + ".";
                    tbServerIp11.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp11.Text + ".";
                    tbServerIp12.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp12.Text + ".";
                    tbServerIp13.Text = rxStruct.DataBuf[iLen++].ToString("D3");
                    strInfo += tbServerIp13.Text + "  ";
                    tbServerPort1.Text = (rxStruct.DataBuf[iLen++] + rxStruct.DataBuf[iLen++] * 256).ToString("D");
                    strInfo += lbPort1.Text + "：" + tbServerPort1.Text;
                    AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    nudHeatBeat.Value = rxStruct.DataBuf[iLen++] * 10;
                    strInfo = "\n  " + lbHeatBeat.Text + "：" + nudHeatBeat.Value.ToString() + "秒  ";
                    int count = rxStruct.DataBuf[iLen++];
                    tbApn.Text = System.Text.Encoding.Default.GetString(rxStruct.DataBuf, iLen, count);
                    if (tbApn.Text.Length > 0)
                    {
                        strInfo += lbApn.Text + "：" + tbApn.Text + "  ";
                    }
                    else
                    {
                        strInfo += lbApn.Text + "：未设置  ";
                    }
                    iLen += count;
                    count = rxStruct.DataBuf[iLen++];
                    tbUsername.Text = System.Text.Encoding.Default.GetString(rxStruct.DataBuf, iLen, count);
                    if (tbUsername.Text.Length > 0)
                    {
                        strInfo += lbUsername.Text + "：" + tbUsername.Text + "  ";
                    }
                    else
                    {
                        strInfo += lbUsername.Text + "：未设置  ";
                    }
                    iLen += count;
                    count = rxStruct.DataBuf[iLen++];
                    tbPassword.Text = System.Text.Encoding.Default.GetString(rxStruct.DataBuf, iLen, count);
                    if (tbPassword.Text.Length > 0)
                    {
                        strInfo += lbPassword.Text + "：" + tbPassword.Text;
                    }
                    else
                    {
                        strInfo += lbPassword.Text + "：未设置";
                    }
                    AddStringToCommBox(false, strInfo, null, Color.DarkBlue);

                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读GPRS参数出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 设置GPRS参数
        private void btSetGprsParam_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("确认要写入这些GPRS参数吗？", "确认信息", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            try
            {
                byte[] dataBuf = new byte[100];
                int iLen = 0;
                dataBuf[iLen++] = Convert.ToByte(tbServerIp00.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp01.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp02.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp03.Text);
                dataBuf[iLen++] = (byte)Convert.ToUInt16(tbServerPort0.Text);
                dataBuf[iLen++] = (byte)(Convert.ToUInt16(tbServerPort0.Text) >> 8);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp10.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp11.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp12.Text);
                dataBuf[iLen++] = Convert.ToByte(tbServerIp13.Text);
                dataBuf[iLen++] = (byte)Convert.ToUInt16(tbServerPort1.Text);
                dataBuf[iLen++] = (byte)(Convert.ToUInt16(tbServerPort1.Text) >> 8);
                dataBuf[iLen++] = (byte)(nudHeatBeat.Value / 10);
                byte[] apn = System.Text.Encoding.Default.GetBytes(tbApn.Text);
                dataBuf[iLen++] = (byte)apn.Length;
                Array.Copy(apn, 0, dataBuf, iLen, apn.Length);
                iLen += apn.Length;
                byte[] username = System.Text.Encoding.Default.GetBytes(tbUsername.Text);
                dataBuf[iLen++] = (byte)username.Length;
                Array.Copy(username, 0, dataBuf, iLen, username.Length);
                iLen += username.Length;
                byte[] password = System.Text.Encoding.Default.GetBytes(tbPassword.Text);
                dataBuf[iLen++] = (byte)password.Length;
                Array.Copy(password, 0, dataBuf, iLen, password.Length);
                iLen += password.Length;
                Array.Resize(ref dataBuf, iLen);
                byte[] txBuf = ConstructTxBuffer(enCmd.写Gprs参数, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------设置GPRS参数----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：设置GPRS参数";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("设置GPRS参数出错：" + ex.Message, "错误");
            }
        }
        private void ExplainSetGprsParam(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写Gprs参数)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                        AddStringToCommBox(false, " 集中器可能会重新启动，并以新的IP地址连接服务器！", null, Color.Red);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                    }
                    lbCurState.Text = "设备状态：空闲";
                    Command = enCmd.指令空闲;
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("设置GPRS参数出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 读取GPRS模块信息
        private void btReadGprsInfo_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] txBuf = ConstructTxBuffer(enCmd.读Gprs信号强度, NeedAck, null, null);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读GPRS模块信息----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读GPRS模块信息";
            ProgressBarCtrl(0, 0, 15000 / timer.Interval);
        }
        private void ExplainReadGprsInfo(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读Gprs信号强度)
                {
                    if (rxStruct.DataBuf[0] == 99)
                    {
                        pgbGrpsRssi.Enabled = false;
                        pgbGrpsRssi.Value = 0;
                    }
                    else if (rxStruct.DataBuf[0] <= pgbGrpsRssi.Maximum)
                    {
                        pgbGrpsRssi.Enabled = true;
                        pgbGrpsRssi.Value = rxStruct.DataBuf[0];
                    }
                    lbConnectStatus.Text = rxStruct.DataBuf[1] == 0x01 ? "联机状态：在线" : "联机状态：离线";
                    if (rxStruct.DataBuf[2] > 0)
                    {
                        lbIMSI.Text = "IMSI编码：" + System.Text.Encoding.Default.GetString(rxStruct.DataBuf, 3, rxStruct.DataBuf[2]);
                    }
                    else
                    {
                        lbIMSI.Text = "IMSI编码：未知";
                    }
                    int pos = 2 + rxStruct.DataBuf[2] + 1;
                    if (rxStruct.DataBuf[pos] < 30)
                    {
                        lbGprsModle.Text = "模块型号：" + System.Text.Encoding.Default.GetString(rxStruct.DataBuf, pos + 1, rxStruct.DataBuf[pos]);
                    }
                    else
                    {
                        lbGprsModle.Text = "模块型号：未知";
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    AddStringToCommBox(false, lbGprsModle.Text + " " + lbConnectStatus.Text + " 信号强度：-" + rxStruct.DataBuf[0].ToString("D") + "dBm " + lbIMSI.Text, null, Color.Red);

                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读GPRS模块信息出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 其他GPRS参数
        private void ExplainGprsMonitor(ProtocolStruct rxStruct)
        {
            if (rxStruct.DataBuf[0] == 0)
            {
                if (rtbGprsMsg.Text.Length > rtbGprsMsg.MaxLength - 100)
                {
                    rtbGprsMsg.Clear();
                }
                rtbGprsMsg.AppendText(System.Text.Encoding.Default.GetString(rxStruct.DataBuf, 1, rxStruct.DataBuf.Length - 1));
                rtbGprsMsg.ScrollToCaret();
            }
        }
        private void tbServerIp_KeyPress(object sender, KeyPressEventArgs e)
        {
            TextBox tb = (TextBox)sender;

            if ("0123456789\b\r\x03\x16\x18".IndexOf(e.KeyChar) < 0)
            {
                e.Handled = true;
                return;
            }
            if (e.KeyChar == '\r')
            {
                SendKeys.Send("{TAB}");
                return;
            }
            if (tb.Text.Length >= 3 && e.KeyChar != '\b')
            {
                if (tb.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        private void tbServerPort_KeyPress(object sender, KeyPressEventArgs e)
        {
            TextBox tb = (TextBox)sender;

            if ("0123456789\b\r\x03\x16\x18".IndexOf(e.KeyChar) < 0)
            {
                e.Handled = true;
                return;
            }
            if (e.KeyChar == '\r')
            {
                SendKeys.Send("{TAB}");
                return;
            }
            if (tb.Text.Length >= 5 && e.KeyChar != '\b')
            {
                if (tb.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        private void tbApnInfo_KeyPress(object sender, KeyPressEventArgs e)
        {
            TextBox tb = (TextBox)sender;

            if (e.KeyChar == '\r')
            {
                SendKeys.Send("{TAB}");
                return;
            }
            if (tb.Text.Length >= 12 && e.KeyChar != '\b')
            {
                if (tb.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        #endregion
        #endregion

        #region 档案和路由选项卡
        #region 从文件中读取档案
        private void btReadDoc_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(false, true, false))
            {
                return;
            }
            if (Document.Rows.Count > 0)
            {
                if (DialogResult.Cancel == MessageBox.Show("新导入的档案将会清除当前的数据，是否继续?", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
                {
                    return;
                }
            }

            string strNodeDoc = XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/DocumentFile", System.Windows.Forms.Application.StartupPath);
            openFileDlg.InitialDirectory = Path.GetDirectoryName(strNodeDoc);
            openFileDlg.Filter = "*.txt(文本文件)|*.txt";
            openFileDlg.DefaultExt = "txt";
            openFileDlg.FileName = "";
            if (openFileDlg.ShowDialog() != DialogResult.OK)
            {
                return;
            }
            string strFileName = openFileDlg.FileName;
            if (strFileName.Length == 0)
            {
                return;
            }
            if (strNodeDoc != strFileName)
            {
                XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "DocumentFile", strFileName);
            }
            Document.Clear();

            int iLine = 0;
            string strRead;
            StreamReader sr = new StreamReader(strFileName, Encoding.Default);
            while ((strRead = sr.ReadLine()) != null && iLine <= MaxNodeNum)
            {
                iLine += 1;
                string strNodeAddr = "";
                string strNodeType = strMeterType[0];
                string strRoute0 = strNoPath;
                string strRoute1 = strNoPath;
                string[] strDoc = strRead.Split(':');
                if (strDoc.Length > 0)
                {
                    string[] strAddrType = strDoc[0].Split(',');
                    if (strAddrType.Length > 0)
                    {
                        strNodeAddr = strAddrType[0];
                    }
                    if (strAddrType.Length > 1 && strAddrType[1] != "")
                    {
                        strNodeType = strAddrType[1];
                    }
                }
                if (strDoc.Length > 1 && strDoc[1] != "")
                {
                    string[] strRoute = strDoc[1].Split(';');
                    if (strRoute.Length > 0 && strRoute[0] != "")
                    {
                        strRoute0 = strRoute[0];
                    }
                    if (strRoute.Length > 1 && strRoute[1] != "")
                    {
                        strRoute1 = strRoute[1];
                    }
                }
                try
                {        
                    // 检查表具地址是否正确
                    if (strNodeAddr.Length != AddrLength * 2)
                    {
                        MessageBox.Show("在第" + iLine.ToString() + "行的档案长度错误，\r\n读取档案信息失败。", "长度错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        sr.Close();
                        return;
                    }             
                    System.Text.RegularExpressions.Regex rx = new System.Text.RegularExpressions.Regex("^\\d{12}$");
                    if (rx.IsMatch(strNodeAddr))
                    {
                        DataRow[] repDt = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (repDt.Length > 0)
                        {
                            sr.Close();
                            int index = Array.IndexOf(Document.Select(), repDt[0]);
                            MessageBox.Show("档案中第" + (index + 1).ToString() + "行和" + iLine.ToString() + "行档案重复，\r\n读取档案信息失败。", "档案重复", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                            return;
                        }
                    }
                    else
                    {
                        MessageBox.Show("在第" + iLine.ToString() + "行\r\n读取档案信息失败，档案地址格式非法。", "格式错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        sr.Close();
                        return;
                    }
                    // 检查表具类型是否正确
                    if (Array.IndexOf(strMeterType, strNodeType) < 0)
                    {
                        sr.Close();
                        MessageBox.Show("地址为" + strNodeAddr + "的表具类型错误,\r\n读取档案信息失败。", "类型错误", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        return;
                    }
                    // 添加至档案中
                    DataRow dr = Document.NewRow();
                    dr["序号"] = Document.Rows.Count + 1;
                    dr["表具地址"] = strNodeAddr;
                    dr["类型"] = strNodeType;
                    dr["路径0"] = strRoute0;
                    dr["路径1"] = strRoute1;
                    Document.Rows.Add(dr);
                }
                catch (System.Exception ex)
                {
                    sr.Close();
                    MessageBox.Show("在第" + iLine.ToString() + "行" + ex.Message + "\r\n读取档案信息失败。", "错误信息", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
                }
            }
            sr.Close();
            MessageBox.Show("从文件中成功导入" + Document.Rows.Count.ToString() + "个表具档案!", "读取档案信息成功", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
        #endregion
        #region 保存档案到文件
        private void btWriteDoc_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(false, true, false))
            {
                return;
            }
            if (Document.Rows.Count == 0)
            {
                MessageBox.Show("没有档案可以保存！", "错误", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            string strExportFilename = XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/DocumentFile", System.Windows.Forms.Application.StartupPath);
            saveFileDlg.InitialDirectory = Path.GetDirectoryName(strExportFilename);
            saveFileDlg.Filter = "*.txt(文本文件)|*.txt";
            saveFileDlg.DefaultExt = "txt";
            saveFileDlg.FileName = "";
            if (DialogResult.OK != saveFileDlg.ShowDialog())
            {
                return;
            }
            string strFileName = saveFileDlg.FileName;
            if (strFileName.Length == 0)
            {
                return;
            }
            if (strExportFilename != strFileName)
            {
                XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "DocumentFile", strFileName);
            }
            StreamWriter sw = new StreamWriter(strFileName, false, System.Text.Encoding.GetEncoding("Unicode"));
            try
            {
                int iLine = 0;
                for (; iLine < Document.Rows.Count; iLine++)
                {
                    bool flag = false;
                    string strTemp = Document.Rows[iLine]["表具地址"].ToString() + "," + Document.Rows[iLine]["类型"].ToString() + ":";
                    if (Document.Rows[iLine]["路径0"].ToString().Length >= AddrLength * 2)
                    {
                        strTemp += Document.Rows[iLine]["路径0"].ToString();
                        flag = true;
                    }
                    if (Document.Rows[iLine]["路径1"].ToString().Length >= AddrLength * 2)
                    {
                        if (true == flag)
                        {
                            strTemp += ";";
                        }
                        strTemp += Document.Rows[iLine]["路径1"].ToString();
                    }
                    sw.WriteLine(strTemp);
                }
                sw.Close();
                MessageBox.Show("保存档案信息成功！", "成功", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            catch (System.Exception ex)
            {
                sw.Close();
                MessageBox.Show("保存档案信息时失败，" + ex.Message, "保存失败", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
        }
        #endregion
        #region 从设备中读取档案
        private void ReadDocFromDev()
        {
            byte[] dataBuf = new byte[100];
            int iLen = 0;

            dataBuf[iLen++] = 0xFF;
            dataBuf[iLen++] = (byte)ReadPos;
            dataBuf[iLen++] = (byte)(ReadPos >> 8);
            dataBuf[iLen++] = (byte)("无线通信" == cmbComType.Text ? 20 : 50);
            Array.Resize(ref dataBuf, iLen);
            byte[] txBuf = ConstructTxBuffer(enCmd.读表具档案信息, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------从设备中读取档案----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：从设备中读取档案";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void btReadDocFromDev_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (Document.Rows.Count > 0)
            {
                if (DialogResult.Cancel == MessageBox.Show("本操作会清除原有的档案信息，是否继续?", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
                {
                    return;
                }
            }
            try
            {
                ReadPos = 0;
                ReadDocFromDev();
                Document.Clear();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("从设备中读取档案出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadDocFromDev(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读表具档案信息)
                {
                    int totalNodeCount = rxStruct.DataBuf[0] + rxStruct.DataBuf[1] * 256;
                    int ackNodeCount = rxStruct.DataBuf[2];
                    int iLen = 3;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < ackNodeCount; iLoop++ )
                    {
                        DataRow dr = Document.NewRow();
                        dr["序号"] = Document.Rows.Count + 1;
                        dr["表具地址"] = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        dr["类型"] = GetDevTypeFromByte(rxStruct.DataBuf[iLen++]);
                        iLen += 1;
                        dr["结果"] = "";
                        Document.Rows.Add(dr);
                        if (iLoop % 3 == 0)
                        {
                            strInfo += "\n";
                        }
                        strInfo += "  " + dr["表具地址"] + "-" + dr["类型"];
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    AddStringToCommBox(false, " 本次读取位置：" + ReadPos.ToString("D") + " 应答数量：" + ackNodeCount.ToString("D") + " 总数量：" + totalNodeCount.ToString("D"), null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }
                    if (ReadPos + ackNodeCount < totalNodeCount)
                    {
                        ReadPos += ackNodeCount;
                        ReadDocFromDev();
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("共读出" + totalNodeCount.ToString("D") + "个档案信息", "读取成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("从设备中读取档案出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 保存档案到设备
        private void WriteDocToDev()
        {
            byte[] dataBuf = new byte[1200];
            int iLen = 0;
            dataBuf[iLen++] = 0;
            dataBuf[iLen++] = 0;
            int iCount, iNum;
            iNum = "无线通信" == cmbComType.Text ? 12 : 60;
            for (iCount = 0; iCount < iNum; )
            {
                DataRow dr = Document.Rows[WritePos + iCount];
                iLen += GetByteAddrFromString(dr["表具地址"].ToString(), dataBuf, iLen);
                dataBuf[iLen++] = GetDevTypeFromString(dr["类型"].ToString());
                dataBuf[iLen++] = 0x01;
                iCount += 1;
                if (WritePos + iCount >= Document.Rows.Count)
                {
                    dataBuf[1] = 1;
                    break;
                }
            }
            dataBuf[0] = (byte)iCount;
            Array.Resize(ref dataBuf, iLen);
            byte[] txBuf = ConstructTxBuffer(enCmd.写表具档案信息, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------保存档案到设备----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：保存档案到设备";
            int time = (Document.Rows.Count + iNum - 1) / iNum;
            ProgressBarCtrl(WritePos / iNum, 0, (Document.Rows.Count / iNum + 1) * CommDelayTime / timer.Interval);
        }
        private void btWriteDocToDev_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (Document.Rows.Count == 0)
            {
                MessageBox.Show("没有可以保存的档案。", "没有档案", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否将全部档案导出到集中器中？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            try
            {
                WritePos = 0;
                WriteDocToDev();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("保存档案到设备出错：" + ex.Message, "错误");
            }
        }
        private void ExplainWriteDocToDev(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写表具档案信息 && WritePos != -1)
                {
                    int setNodeCount = rxStruct.DataBuf[0];
                    int iLen = 1;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < setNodeCount; iLoop++)
                    {
                        string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                        DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (dr.Length > 0)
                        {
                            dr[0]["结果"] = "写档案:" + strResult;
                        }
                        strInfo += "\n " + strNodeAddr + "-" + strResult;
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    AddStringToCommBox(false, " 本次写入位置：" + WritePos.ToString("D") + " 写入数量：" + setNodeCount.ToString("D"), null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }
                    if (WritePos + setNodeCount < Document.Rows.Count)
                    {
                        WritePos += setNodeCount;
                        WriteDocToDev();
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("共写入" + Document.Rows.Count.ToString("D") + "个档案信息", "写入成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("保存档案到设备出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 从设备中读取全部路由
        private void ReadRouteFromDev()
        {
            byte[] dataBuf = new byte[3];
            dataBuf[0] = (byte)ReadPos;
            dataBuf[1] = (byte)(ReadPos >> 8);
            dataBuf[2] = (byte)("无线通信" == cmbComType.Text ? 3 : 10);
            byte[] txBuf = ConstructTxBuffer(enCmd.批量读设备自定义路由信息, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------从设备中读取全部路由----------------->>>", null, Color.DarkGreen);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：从设备中读取全部路由";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void btReadRouteFromDev_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("读出的路由信息会自动和现有节点档案匹配，如果没有此档案则增加这个档案，是否继续?", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            try
            {
                ReadPos = 0;
                ReadRouteFromDev();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("从设备中读取全部路由出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadRouteFromDev(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量读设备自定义路由信息)
                {
                    int totalNodeCount = rxStruct.DataBuf[0] + rxStruct.DataBuf[1] * 256;
                    int ackNodeCount = rxStruct.DataBuf[2];
                    int iLen = 3;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < ackNodeCount; iLoop++)
                    {
                        string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        string strNodeType = GetDevTypeFromByte(rxStruct.DataBuf[iLen++]);
                        int iCount = rxStruct.DataBuf[iLen++];
                        string strPath0 = "";
                        for (int i = 0; i < iCount; i++)
                        {
                            strPath0 += GetStringAddrFromByte(rxStruct.DataBuf, iLen) + ",";
                            iLen += AddrLength;
                        }
                        strPath0 = strPath0.TrimEnd(',');
                        if (strPath0 == "")
                        {
                            strPath0 = strNoPath;
                        }
                        iCount = rxStruct.DataBuf[iLen++];
                        string strPath1 = "";
                        for (int i = 0; i < iCount; i++)
                        {
                            strPath1 += GetStringAddrFromByte(rxStruct.DataBuf, iLen) + ",";
                            iLen += AddrLength;
                        }
                        strPath1 = strPath1.TrimEnd(',');
                        if (strPath1 == "")
                        {
                            strPath1 = strNoPath;
                        }
                        // 查找档案中对应的表具
                        DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (dr.Length > 0)
                        {
                            dr[0]["类型"] = strNodeType;
                            dr[0]["路径0"] = strPath0;
                            dr[0]["路径1"] = strPath1;
                        }
                        else
                        {
                            DataRow newDr = Document.NewRow();
                            newDr["表具地址"] = strNodeAddr;
                            newDr["类型"] = strNodeType;
                            newDr["路径0"] = strPath0;
                            newDr["路径1"] = strPath1;
                            Document.Rows.Add(newDr);
                        }
                        strInfo += "\n " + strNodeAddr + "," + strNodeType + ":" + strPath0 + ";" + strPath1;
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    AddStringToCommBox(false, " 本次读取位置：" + ReadPos.ToString("D") + " 应答数量：" + ackNodeCount.ToString("D") + " 总数量：" + totalNodeCount.ToString("D"), null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }
                    if (ReadPos + ackNodeCount < totalNodeCount)
                    {
                        ReadPos += ackNodeCount;
                        ReadRouteFromDev();
                    }
                    else
                    {
                        dgvDoc.Columns["序号"].Visible = true;
                        dgvDoc.Columns["类型"].Visible = true;
                        dgvDoc.Columns["表具地址"].Visible = true;
                        dgvDoc.Columns["路径0"].Visible = true;
                        dgvDoc.Columns["路径1"].Visible = true;


                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        dgvDoc_SelectionChanged(null, null);
                        MessageBox.Show("共读出" + totalNodeCount.ToString("D") + "个路由信息", "读取成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("从设备中读取路由信息出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 保存全部路由到设备
        private void WriteRouteToDevProc()
        {
            byte[] dataBuf = new byte[1200];
            int iLen = 0;
            int iNum = "无线通信" == cmbComType.Text ? 150 : 700;
            for (; WritePos < Document.Rows.Count; WritePos++)
            {
                DataRow dr = Document.Rows[WritePos];
                iLen += GetByteAddrFromString(dr["表具地址"].ToString(), dataBuf, iLen);
                string strPath0 = dr["路径0"].ToString();
                if (strPath0 == "" || strPath0 == strNoPath)
                {
                    dataBuf[iLen++] = 0;
                }
                else
                {
                    string[] strTemp = strPath0.Split(',');
                    dataBuf[iLen++] = (byte)(strTemp.Length);
                    for (int i = 0; i < strTemp.Length; i++)
                    {
                        iLen += GetByteAddrFromString(strTemp[i], dataBuf, iLen);
                    }
                }
                string strPath1 = dr["路径1"].ToString();
                if (strPath1 == "" || strPath1 == strNoPath)
                {
                    dataBuf[iLen++] = 0;
                }
                else
                {
                    string[] strTemp = strPath1.Split(',');
                    dataBuf[iLen++] = (byte)(strTemp.Length);
                    for (int i = 0; i < strTemp.Length; i++)
                    {
                        iLen += GetByteAddrFromString(strTemp[i], dataBuf, iLen);
                    }
                }
                if (iLen > iNum)
                {
                    break;
                }
            }
            if (iLen > 0)
            {
                Array.Resize(ref dataBuf, iLen);
                byte[] txBuf = ConstructTxBuffer(enCmd.批量写设备自定义路由信息, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------保存全部路由到设备----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：保存全部路由到设备";
                ProgressBarCtrl(0, 0, (CommDelayTime + 2000) / timer.Interval);
            }
            else
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("共写入" + Document.Rows.Count.ToString("D") + "个档案信息", "写入成功");
            }
        }
        private void btWriteRouteToDev_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (Document.Rows.Count == 0)
            {
                MessageBox.Show("没有可以保存的路由档案。", "没有档案", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否将全部路由信息导出到集中器中？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            try
            {
                WritePos = 0;
                WriteRouteToDevProc();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("保存全部路由到设备出错：" + ex.Message, "错误");
            }
        }
        private void ExplainWriteRouteToDev(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量写设备自定义路由信息)
                {
                    int setNodeCount = rxStruct.DataBuf.Length / 7;
                    int iLen = 0;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < setNodeCount; iLoop++)
                    {
                        string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                        DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (dr.Length > 0)
                        {
                            dr[0]["结果"] = "写路由信息：" + strResult;
                        }
                        strInfo += "\n " + strNodeAddr + "-" + strResult;
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }

                    WriteRouteToDevProc();
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("保存全部路由到设备出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 档案初始化
        private void btFormatDoc_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("档案初始化会清除集中器中所有的表具档案、中继档案和路由信息，是否继续?", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            try
            {
                byte[] dataBuf = new byte[1] { 0 };
                byte[] txBuf = ConstructTxBuffer(enCmd.集中器初始化, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------档案初始化----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：档案初始化";
                ProgressBarCtrl(0, 0, 1000 * 5 / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("档案初始化时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainFormatDoc(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.集中器初始化 && rxStruct.DataBuf[0] == 0)
                {
                    AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[1]), null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("档案初始化出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 数据初始化
        private void btFormatData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("数据初始化会清除集中器中所有表具抄读到的数据，但是不会清除表具档案、中继档案和路由信息，并且需要耗费较长的时间，是否继续?", "确认请求", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
            {
                return;
            }
            try
            {
                byte[] dataBuf = new byte[1] { 1 };
                byte[] txBuf = ConstructTxBuffer(enCmd.集中器初始化, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------数据初始化----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：数据初始化";
                ProgressBarCtrl(0, 0, 1000 * 60 / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("数据初始化时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainFormatData(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.集中器初始化 && rxStruct.DataBuf[0] == 1)
                {
                    AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[1]), null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("数据初始化出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 读取设备中档案数量
        private void btReadDevNodeCount_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] dataBuf = new byte[1] { 0xFF };
            byte[] txBuf = ConstructTxBuffer(enCmd.读表具数量, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取设备中档案数量----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取设备中档案数量";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainReadDevNodeCount(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读表具数量 && rxStruct.DataBuf[0] == 0xFF)
                {
                    int iCount = rxStruct.DataBuf[1] + rxStruct.DataBuf[2] * 256;
                    btReadDevNodeCount.Text = "设备中档案数量(" + iCount.ToString("D") + ")";
                    AddStringToCommBox(false, "\n执行结果：成功  集中器中档案的数量为：" + iCount.ToString("D"), null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取设备中档案数量出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 新建档案
        private void btCreatNewNode_Click(object sender, EventArgs e)
        {
            if (cbSyncWithConc.Checked == true)
            {
                if (false == DeviceStatus(true, true, true))
                {
                    return;
                }
            }
            else
            {
                if (false == DeviceStatus(false, true, false))
                {
                    return;
                }
            }
            if (tbCurNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在当前表具地址中输入合法的新建档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (cmbNodeType.Text == "")
            {
                MessageBox.Show("请在当前表具类型中选择新建的表具类型！", "类型错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("确认是否要新建该档案？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            DataRow[] repDr = Document.Select("表具地址='" + tbCurNodeAddr.Text + "'");
            if (repDr.Length > 0)
            {
                if (false == cbSyncWithConc.Checked)
                {
                    MessageBox.Show("新添加的档案已经存在，添加失败！", "档案重复");
                    return;
                }
                else
                {
                    if (DialogResult.No == MessageBox.Show("新添加的档案已经存在，继续向集中器中添加档案吗？", "档案重复", MessageBoxButtons.YesNo, MessageBoxIcon.Question))
                    {
                        return;
                    }
                }
            }
            else
            {
                DataRow newDr = Document.NewRow();
                newDr["序号"] = Document.Rows.Count + 1;
                newDr["表具地址"] = tbCurNodeAddr.Text;
                newDr["类型"] = cmbNodeType.Text;
                Document.Rows.Add(newDr);
            }
            if (true == cbSyncWithConc.Checked)
            {
                try
                {
                    byte[] dataBuf = new byte[10];
                    WritePos = -1;
                    dataBuf[0] = 1;
                    dataBuf[1] = 1;
                    GetByteAddrFromString(tbCurNodeAddr.Text, dataBuf, 2);
                    dataBuf[8] = GetDevTypeFromString(cmbNodeType.Text);
                    dataBuf[9] = 1;
                    byte[] txBuf = ConstructTxBuffer(enCmd.写表具档案信息, NeedAck, null, dataBuf);
                    DataTransmit(txBuf);
                    AddStringToCommBox(false, "\n<<<-----------------在集中器中新建档案----------------->>>", null, Color.DarkGreen, true);
                    AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                    lbCurState.Text = "设备状态：在集中器中新建档案";
                    ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
                }
                catch (Exception ex)
                {
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    MessageBox.Show("在集中器中新建档案时出错：" + ex.Message, "错误");
                }
            }
        }
        private void ExplainCreatNewNode(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写表具档案信息 && WritePos == -1)
                {
                    int setNodeCount = rxStruct.DataBuf[0];
                    int iLen = 1;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < setNodeCount; iLoop++)
                    {
                        string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                        DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (dr.Length > 0)
                        {
                            dr[0]["结果"] = "新建档案:" + strResult;
                        }
                        strInfo += "  " + strNodeAddr + "-" + strResult;
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("新建档案到集中器出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 删除档案
        private void btDelNode_Click(object sender, EventArgs e)
        {
            if (cbSyncWithConc.Checked == true)
            {
                if (false == DeviceStatus(true, true, true))
                {
                    return;
                }
            }
            else
            {
                if (false == DeviceStatus(false, true, false))
                {
                    return;
                }
            }
            if (tbCurNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在当前表具地址中输入合法的待删除档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否要删除该档案？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            string strNodeAddr = tbCurNodeAddr.Text;
            DataRow[] dr = Document.Select("表具地址='" + tbCurNodeAddr.Text + "'");
            if (dr.Length == 0)
            {
                if (false == cbSyncWithConc.Checked)
                {
                    MessageBox.Show("待删除的档案不存在，删除失败！", "无此档案");
                    return;
                }
                else
                {
                    if (DialogResult.No == MessageBox.Show("待删除的档案不存在，继续在集中器中删除该档案吗？", "无此档案", MessageBoxButtons.YesNo, MessageBoxIcon.Question))
                    {
                        return;
                    }
                }
            }
            else
            {
                Document.Rows.Remove(dr[0]);
                for (int iLoop = 0; iLoop < Document.Rows.Count; iLoop++)
                {
                    Document.Rows[iLoop]["序号"] = iLoop + 1;
                }
            }
            if (true == cbSyncWithConc.Checked)
            {
                try
                {
                    byte[] dataBuf = new byte[AddrLength];
                    GetByteAddrFromString(strNodeAddr, dataBuf, 0);
                    byte[] txBuf = ConstructTxBuffer(enCmd.删除表具档案信息, NeedAck, null, dataBuf);
                    DataTransmit(txBuf);
                    AddStringToCommBox(false, "\n<<<-----------------在集中器中删除档案----------------->>>", null, Color.DarkGreen, true);
                    AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                    lbCurState.Text = "设备状态：在集中器中删除档案";
                    ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
                }
                catch (Exception ex)
                {
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    MessageBox.Show("在集中器中删除档案时出错：" + ex.Message, "错误");
                }
            }
        }
        private void ExplainDelNode(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.删除表具档案信息)
                {
                    string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, 0);
                    string strResult = GetErrorInfo(rxStruct.DataBuf[AddrLength]);
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    AddStringToCommBox(false, "  " + strNodeAddr + "-" + strResult, null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("删除表具档案信息出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 修改档案
        private void btModifyNode_Click(object sender, EventArgs e)
        {
            if (cbSyncWithConc.Checked == true)
            {
                if (false == DeviceStatus(true, true, true))
                {
                    return;
                }
            }
            else
            {
                if (false == DeviceStatus(false, true, false))
                {
                    return;
                }
            }
            if (tbCurNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在当前表具地址中输入合法的待修改档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (tbNewNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在新表具地址中输入合法的修改后档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否要修改该档案数据信息？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            DataRow[] repDr = Document.Select("表具地址='" + tbNewConcAddr.Text + "'");
            if (repDr.Length > 0)
            {
                MessageBox.Show("修改的新档案地址已经存在，请重新确定新档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (cmbNodeType.Text == "")
            {
                MessageBox.Show("请在当前表具类型中选择新建的表具类型！", "类型错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            repDr = Document.Select("表具地址='" + tbCurNodeAddr.Text + "'");
            if (repDr.Length == 0)
            {
                if (false == cbSyncWithConc.Checked)
                {
                    MessageBox.Show("待修改的档案不存在，修改失败！", "无此档案");
                    return;
                }
                else
                {
                    if (DialogResult.No == MessageBox.Show("待修改的档案不存在，继续在集中器中修改该档案吗？", "无此档案", MessageBoxButtons.YesNo, MessageBoxIcon.Question))
                    {
                        return;
                    }
                }
            }
            else if (repDr.Length == 1)
            {
                repDr[0]["表具地址"] = tbNewNodeAddr.Text;
                repDr[0]["类型"] = cmbNodeType.Text;
            }
            else
            {
                MessageBox.Show("待修改的档案已存在，修改失败！", "档案重复");
                return;
            }
            if (true == cbSyncWithConc.Checked)
            {
                try
                {
                    byte[] dataBuf = new byte[13];
                    GetByteAddrFromString(tbCurNodeAddr.Text, dataBuf, 0);
                    GetByteAddrFromString(tbNewNodeAddr.Text, dataBuf, AddrLength);
                    dataBuf[AddrLength * 2] = GetDevTypeFromString(cmbNodeType.Text);
                    byte[] txBuf = ConstructTxBuffer(enCmd.修改表具档案信息, NeedAck, null, dataBuf);
                    DataTransmit(txBuf);
                    AddStringToCommBox(false, "\n<<<-----------------在集中器中修改档案----------------->>>", null, Color.DarkGreen, true);
                    AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                    lbCurState.Text = "设备状态：在集中器中修改档案";
                    ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
                }
                catch (Exception ex)
                {
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    MessageBox.Show("在集中器中修改档案时出错：" + ex.Message, "错误");
                }
            }
        }
        private void ExplainModifyNode(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.修改表具档案信息)
                {
                    if (rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                    }
                    else
                    {
                        AddStringToCommBox(false, "\n执行结果：失败  失败原因：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.Red);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("在集中器中修改档案时出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 从设备中读取当前表具路由
        private void btReadNodeRoute_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbCurNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在当前表具地址中输入合法的新建档案地址！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            try
            {
                byte[] dataBuf = new byte[AddrLength];
                GetByteAddrFromString(tbCurNodeAddr.Text, dataBuf, 0);
                byte[] txBuf = ConstructTxBuffer(enCmd.读表具自定义路由信息, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------读当前表具路径信息----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：读当前表具路径信息";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读当前表具路径信息出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadRoute(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读表具自定义路由信息)
                {                    
                    int iLen = 0;
                    string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                    string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                    if (strNodeAddr != tbCurNodeAddr.Text)
                    {
                        return;
                    }
                    iLen += AddrLength;
                    string strNodeType = GetDevTypeFromByte(rxStruct.DataBuf[iLen++]);
                    string strInfo = "";
                    int iCount = rxStruct.DataBuf[iLen++];
                    string strPath0 = "";
                    for (int i = 0; i < iCount; i++)
                    {
                        strPath0 += GetStringAddrFromByte(rxStruct.DataBuf, iLen) + ",";
                        iLen += AddrLength;
                    }
                    strPath0 = strPath0.TrimEnd(',');
                    if (strPath0 == "")
                    {
                        strPath0 = strNoPath;
                    }
                    iCount = rxStruct.DataBuf[iLen++];
                    string strPath1 = "";
                    for (int i = 0; i < iCount; i++)
                    {
                        strPath1 += GetStringAddrFromByte(rxStruct.DataBuf, iLen) + ",";
                        iLen += AddrLength;
                    }
                    strPath1 = strPath1.TrimEnd(',');
                    if (strPath1 == "")
                    {
                        strPath1 = strNoPath;
                    }
                    // 查找档案中对应的表具
                    DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                    if (dr.Length > 0)
                    {
                        dr[0]["类型"] = strNodeType;
                        dr[0]["路径0"] = strPath0;
                        dr[0]["路径1"] = strPath1;
                    }
                    strInfo += "\n " + strNodeAddr + "," + strNodeType + ":" + strPath0 + ";" + strPath1;
                    AddStringToCommBox(false, "\n执行结果：" + strResult, null, Color.DarkBlue);
                    if (strInfo.Length > 0 && rxStruct.DataBuf[0] == 0xAA)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    if (0xAA == rxStruct.DataBuf[0])
                    {
                        dgvDoc_SelectionChanged(null, null);
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("从设备中读取路由信息出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 保存当前表具路由到设备
        private void btWriteNodeRoute_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbCurNodeAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请在档案中选择要保存路由信息的表具！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否保存当前路由信息到集中器中？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            try
            {
                int iLen = 0;
                byte[] dataBuf = new byte[100];
                iLen += GetByteAddrFromString(tbCurNodeAddr.Text, dataBuf, iLen);
                DataRow[] dr = Document.Select("表具地址='" + tbCurNodeAddr.Text + "'");
                if (dr.Length == 0)
                {
                    MessageBox.Show("在档案中没有找到该表具信息！", "档案错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
                string strPath0 = dr[0]["路径0"].ToString();
                if (strPath0 == "" || strPath0 == strNoPath)
                {
                    dataBuf[iLen++] = 0;
                }
                else
                {
                    string[] strTemp = strPath0.Split(',');
                    dataBuf[iLen++] = (byte)(strTemp.Length);
                    for (int i = 0; i < strTemp.Length; i++)
                    {
                        iLen += GetByteAddrFromString(strTemp[i], dataBuf, iLen);
                    }
                }
                string strPath1 = dr[0]["路径1"].ToString();
                if (strPath1 == "" || strPath1 == strNoPath)
                {
                    dataBuf[iLen++] = 0;
                }
                else
                {
                    string[] strTemp = strPath1.Split(',');
                    dataBuf[iLen++] = (byte)(strTemp.Length);
                    for (int i = 0; i < strTemp.Length; i++)
                    {
                        iLen += GetByteAddrFromString(strTemp[i], dataBuf, iLen);
                    }
                }
                Array.Resize(ref dataBuf, iLen);
                byte[] txBuf = ConstructTxBuffer(enCmd.写表具自定义路由信息, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------写当前表具路径信息----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：写当前表具路径信息";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("写当前表具路径信息出错：" + ex.Message, "错误");
            }
        }
        private void ExplainWriteRoute(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写表具自定义路由信息)
                {
                    int iLen = 0;
                    string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                    iLen += AddrLength;
                    string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                    if (strNodeAddr != tbCurNodeAddr.Text)
                    {
                        return;
                    }
                    DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                    if (dr.Length > 0)
                    {
                        dr[0]["结果"] = "写路由信息：" + strResult;
                    }
                    AddStringToCommBox(false, "\n执行结果：" + strResult, null, Color.DarkBlue);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("写表具自定义路由信息出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 路径测试
        byte curTestPathNo = 0;
        private void btTestPath0_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            try
            {
                int iLevel = 0;
                byte[] path;
                if (PathLevel[0] <= 2)
                {
                    path = new byte[AddrLength * 3 + 1];
                    path[iLevel++] = 0x03;
                    iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                    iLevel += GetByteAddrFromString(tbConcAddr.Text, path, iLevel);
                    iLevel += GetByteAddrFromString(tbCurNodeAddr.Text, path, iLevel);
                }
                else
                {
                    path = new byte[AddrLength * (PathLevel[0] + 1) + 1];
                    path[iLevel++] = (byte)(PathLevel[0] + 1);
                    iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                    for (int iRow = 0; iRow < PathLevel[0]; iRow++)
                    {
                        TextBox tbNodeAddr = (TextBox)(gpbPath0.Controls.Find("tbNodeAddr0" + iRow, false))[0];
                        if (tbNodeAddr.Text.Length == AddrLength * 2)
                        {
                            iLevel += GetByteAddrFromString(tbNodeAddr.Text, path, iLevel);
                        }
                    }
                }
                if (path[0] > 5)
                {
                    AddStringToCommBox(false, "\n 路径过长，请保存当前路径并使用“数据转发”功能直接测试路径。 ", null, Color.Red, true);
                    return;
                }

                byte[] dataBuf = new byte[1] { 0 };   // 定时定量数据
                byte[] txBuf = ConstructTxBuffer(enCmd.读表具数据, NeedAck, path, dataBuf);      // zxp 要注意加密时这个加密位是由集中器加还是由电脑加,由集中器加
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------路径0测试----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：路径0测试中";
                ProgressBarCtrl(0, 0, (iLevel / AddrLength) * CommDelayTime / timer.Interval);
                curTestPathNo = 0;
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("路径0测试时出错：" + ex.Message, "错误");
            }
        }
        private void btTestPath1_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            try
            {
                int iLevel = 0;
                byte[] path;
                if (PathLevel[1] <= 2)
                {
                    path = new byte[AddrLength * 3 + 1];
                    path[iLevel++] = 0x03;
                    iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                    iLevel += GetByteAddrFromString(tbConcAddr.Text, path, iLevel);
                    iLevel += GetByteAddrFromString(tbCurNodeAddr.Text, path, iLevel);
                }
                else
                {
                    path = new byte[AddrLength * (PathLevel[1] + 1) + 1];
                    path[iLevel++] = (byte)(PathLevel[1] + 1);
                    iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                    for (int iRow = 0; iRow < PathLevel[1]; iRow++)
                    {
                        TextBox tbNodeAddr = (TextBox)(gpbPath1.Controls.Find("tbNodeAddr1" + iRow, false))[0];
                        if (tbNodeAddr.Text.Length == AddrLength * 2)
                        {
                            iLevel += GetByteAddrFromString(tbNodeAddr.Text, path, iLevel);
                        }
                    }
                }
                byte[] dataBuf = new byte[1] { 0 };   // 定时定量数据
                byte[] txBuf = ConstructTxBuffer(enCmd.读表具数据, NeedAck, path, dataBuf);      // zxp 要注意加密时这个加密位是由集中器加还是由电脑加,由集中器加
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------路径1测试----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：路径1测试中";
                ProgressBarCtrl(0, 0, (iLevel / AddrLength) * CommDelayTime / timer.Interval);
                curTestPathNo = 1;
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("路径0测试时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadMeterData(ProtocolStruct rxStruct)
        {
            int iLoop;
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读表具数据)
                {
                    string strDstAddr = GetStringAddrFromByte(rxStruct.RoutePathList, rxStruct.RoutePathList.Length - AddrLength);
                    if (strDstAddr != strLocalAddr)
                    {
                        return;
                    }
                    int iLevel = 0;
                    byte[] path;
                    if (PathLevel[curTestPathNo] <= 2)
                    {
                        path = new byte[AddrLength * 3];
                        iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                        iLevel += GetByteAddrFromString(tbConcAddr.Text, path, iLevel);
                        iLevel += GetByteAddrFromString(tbCurNodeAddr.Text, path, iLevel);
                    }
                    else
                    {
                        path = new byte[AddrLength * (PathLevel[curTestPathNo] + 1)];
                        iLevel += GetByteAddrFromString(strLocalAddr, path, iLevel);
                        for (int iRow = 0; iRow < PathLevel[curTestPathNo]; iRow++)
                        {
                            TextBox tbNodeAddr;
                            if (0 == curTestPathNo)
                            {
                                tbNodeAddr = (TextBox)(gpbPath0.Controls.Find("tbNodeAddr0" + iRow, false))[0];
                            }
                            else
                            {
                                tbNodeAddr = (TextBox)(gpbPath1.Controls.Find("tbNodeAddr1" + iRow, false))[0];
                            }
                            if (tbNodeAddr.Text.Length == AddrLength * 2)
                            {
                                iLevel += GetByteAddrFromString(tbNodeAddr.Text, path, iLevel);
                            }
                        }
                    }
                    byte[] ackPath = new byte[rxStruct.RouteLevelByte * AddrLength];
                    for (iLoop = 0; iLoop < rxStruct.RouteLevelByte; iLoop++)
                    {
                        Array.Copy(rxStruct.RoutePathList, (rxStruct.RouteLevelByte - iLoop - 1) * AddrLength, ackPath, iLoop * AddrLength, AddrLength);
                    }
                    for (iLoop = 0; iLoop < ackPath.Length; iLoop++)
                    {
                        if (ackPath[iLoop] != path[iLoop])
                        {
                            break;
                        }
                    }
                    string strResult = "";
                    if (iLoop < ackPath.Length)
                    {
                        strResult = "路径" + curTestPathNo.ToString("D") + "测试失败!";
                    }
                    else
                    {
                        strResult = "路径" + curTestPathNo.ToString("D") + "测试成功!";
                    }
                    AddStringToCommBox(false, "\n" + strResult, null, Color.Red);
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    MessageBox.Show(strResult, "测试结果");
                    return;
                }
            }
            catch (Exception)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                //MessageBox.Show("数据初始化出错：" + ex.Message);
            }
            return;
        } 
        #endregion
        #region 自定义路由控件功能
        private const int iTextBoxNodeAddrPosX = 32;
        private const int iTextBoxNodeAddrPosY = 18;
        private const int iButtonInsertNodePosX = 12;
        private const int iButtonInsertNodePosY = 9;
        private const int iButtonDelNodePosX = 127;
        private const int iButtonDelNodePosY = 21;
        private const int iPictureBoxTopLinePosX = 18;
        private const int iPictureBoxTopLinePosY = 14;
        private const int iPictureBoxLeftLinePosX = 18;
        private const int iPictureBoxLeftLinePosY = 6;
        private const int iPictureBoxRightLinePosX = 116;
        private const int iPictureBoxRightLinePosY = 22;
        private const int iControlsInterval = 24;
        private const int iLabelCreatePathPosX = 10;
        private const int iLabelCreatePathPosY = 22;
        private const int iButtonCreatePathPosX = 35;
        private const int iButtonCreatePathPosY = 45;
        private object AddControlItems(int iCol, int iRow, int iPathLevel, string strText = "", bool bEdit = false)
        {
            GroupBox gpbPath = (GroupBox)(gpbNodeEdit.Controls.Find("gpbPath" + iCol, false))[0];
            // 添加创建编辑框
            if (iPathLevel == 0)
            {
                gpbPath.Controls.Clear();
                Label lbNoUserDefPath = new Label();
                if (strText == "NULL")
                {
                    lbNoUserDefPath.Text = "按\"从设备中读取当前表具路由\"键获取自定义路径！";
                    lbNoUserDefPath.Location = new System.Drawing.Point(iLabelCreatePathPosX, iLabelCreatePathPosY);
                    lbNoUserDefPath.AutoSize = false;
                    lbNoUserDefPath.Size = new System.Drawing.Size(120, 50);
                    lbNoUserDefPath.RightToLeft = System.Windows.Forms.RightToLeft.No;
                    gpbPath.Controls.Add(lbNoUserDefPath);
                    return null;
                }
                else
                {
                    lbNoUserDefPath.Text = "无路径";
                    lbNoUserDefPath.Location = new System.Drawing.Point(iLabelCreatePathPosX, iLabelCreatePathPosY);
                    lbNoUserDefPath.AutoSize = true;
                    lbNoUserDefPath.RightToLeft = System.Windows.Forms.RightToLeft.No;
                    gpbPath.Controls.Add(lbNoUserDefPath);

                    Button btCreatUserDefPath = new Button();
                    btCreatUserDefPath.Name = "btCreatPath" + iCol;
                    btCreatUserDefPath.Text = "新建路径";
                    btCreatUserDefPath.Location = new System.Drawing.Point(iButtonCreatePathPosX, iButtonCreatePathPosY);
                    btCreatUserDefPath.AutoSize = true;
                    btCreatUserDefPath.Enabled = true;
                    btCreatUserDefPath.Click += new System.EventHandler(this.CreateNodeProc);
                    gpbPath.Controls.Add(btCreatUserDefPath);
                    return btCreatUserDefPath;
                }
            }
            // 添加文本编辑框
            if (iRow == 0)
            {
                gpbPath.Controls.Clear();
            }
            TextBox tbNodeAddr = new TextBox();
            tbNodeAddr.Name = "tbNodeAddr" + iCol + iRow;
            if (iRow == 0)
            {
                tbNodeAddr.Text = tbConcAddr.Text;
                tbNodeAddr.ReadOnly = true;
            }
            else if (iRow == iPathLevel - 1)
            {
                tbNodeAddr.Text = tbCurNodeAddr.Text;
                tbNodeAddr.ReadOnly = true;
            }
            else
            {
                tbNodeAddr.Text = strText;
                tbNodeAddr.ReadOnly = false;// (bEdit == true) ? false : true;
            }
            tbNodeAddr.Location = new System.Drawing.Point(iTextBoxNodeAddrPosX, iTextBoxNodeAddrPosY + iRow * iControlsInterval);
            tbNodeAddr.Size = new System.Drawing.Size(88, 21);
            tbNodeAddr.RightToLeft = System.Windows.Forms.RightToLeft.No;
            tbNodeAddr.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            //tbNodeAddr.AutoCompleteSource = AutoCompleteSource.CustomSource;
            //tbNodeAddr.AutoCompleteCustomSource = AutoCompleteStringArray;
            //tbNodeAddr.AutoCompleteMode = AutoCompleteMode.Suggest;
            tbNodeAddr.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.tbNodeAddr_KeyPress);
            tbNodeAddr.Leave += new System.EventHandler(this.tbNodeAddr_Leave);
            gpbPath.Controls.Add(tbNodeAddr);
            //if (bEdit == false)
            //{
            //    return tbNodeAddr;
            //}
            // 添加插入按钮和横线
            if (iRow == 0)
            {
                PictureBox pbLeftLine = new PictureBox();
                pbLeftLine.Name = "pbLeftLine" + iCol + iRow;
                pbLeftLine.Size = new System.Drawing.Size(15, 15);
                pbLeftLine.Location = new System.Drawing.Point(iPictureBoxTopLinePosX, iPictureBoxTopLinePosY + iRow * iControlsInterval);
                pbLeftLine.Image = global::SR6009_Concentrator_Tools.Properties.Resources.LineTop;
                pbLeftLine.SendToBack();
                gpbPath.Controls.Add(pbLeftLine);
            }
            else
            {
                Button btNewInsertNode = new Button();
                btNewInsertNode.Name = "btInsertNode" + iCol + iRow;
                btNewInsertNode.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Center;
                btNewInsertNode.Image = global::SR6009_Concentrator_Tools.Properties.Resources.Add;
                btNewInsertNode.Location = new System.Drawing.Point(iButtonInsertNodePosX, iButtonInsertNodePosY + iRow * iControlsInterval);
                btNewInsertNode.Size = new System.Drawing.Size(15, 15);
                btNewInsertNode.UseVisualStyleBackColor = true;
                btNewInsertNode.Click += new System.EventHandler(this.InsertNodeProc);
                gpbPath.Controls.Add(btNewInsertNode);

                PictureBox pbLeftLine = new PictureBox();
                pbLeftLine.Name = "pbLeftLine" + iCol + iRow;
                pbLeftLine.Size = new System.Drawing.Size(15, 25);
                pbLeftLine.Location = new System.Drawing.Point(iPictureBoxLeftLinePosX, iPictureBoxLeftLinePosY + iRow * iControlsInterval);
                pbLeftLine.Image = global::SR6009_Concentrator_Tools.Properties.Resources.LineLeft;
                gpbPath.Controls.Add(pbLeftLine);
            }
            // 添加删除按钮和横线
            if (iRow > 0 && iRow < PathLevel[iCol] - 1)
            {
                Button btDelNode = new Button();
                btDelNode.Name = "btDelNode" + iCol + iRow;
                btDelNode.Image = global::SR6009_Concentrator_Tools.Properties.Resources.Sub;
                btDelNode.Location = new System.Drawing.Point(iButtonDelNodePosX, iButtonDelNodePosY + iRow * iControlsInterval);
                btDelNode.Size = new System.Drawing.Size(15, 15);
                btDelNode.TextImageRelation = System.Windows.Forms.TextImageRelation.ImageAboveText;
                btDelNode.UseVisualStyleBackColor = true;
                btDelNode.Click += new System.EventHandler(this.DelNodeProc);
                gpbPath.Controls.Add(btDelNode);

                PictureBox pbRightLine = new PictureBox();
                pbRightLine.Name = "pbRightLine" + iCol + iRow;
                pbRightLine.Size = new System.Drawing.Size(15, 15);
                pbRightLine.Location = new System.Drawing.Point(iPictureBoxRightLinePosX, iPictureBoxRightLinePosY + iRow * iControlsInterval);
                pbRightLine.Image = global::SR6009_Concentrator_Tools.Properties.Resources.LineRight;
                gpbPath.Controls.Add(pbRightLine);
            }
            return tbNodeAddr;
        }
        private void CreateNodeProc(object sender, EventArgs e)
        {
            int iCol, iRow;
            Button btCreatPath = (Button)sender;

            string strBtnName = btCreatPath.Name;
            iCol = int.Parse(strBtnName.Substring(strBtnName.Length - 1, 1));
            PathLevel[iCol] = 3;
            for (iRow = 0; iRow < PathLevel[iCol]; iRow++)
            {
                if (iRow == 0 || iRow == PathLevel[iCol] - 1)
                {
                    AddControlItems(iCol, iRow, PathLevel[iCol], "", true);
                }
                else
                {
                    TextBox tbNodeAddr = (TextBox)AddControlItems(iCol, iRow, PathLevel[iCol], "", true);
                    tbNodeAddr.Focus();
                }
            }
        }
        private void InsertNodeProc(object sender, EventArgs e)
        {
            int iCol, iRow, iLoop;
            Button btInsert = (Button)sender;

            string strBtnName = btInsert.Name;
            iRow = int.Parse(strBtnName.Substring(strBtnName.Length - 1, 1));
            iCol = int.Parse(strBtnName.Substring(strBtnName.Length - 2, 1));
            if (PathLevel[iCol] >= MaxNodeNumOnePath)
            {
                return;
            }
            else
            {
                GroupBox gpbPath = (GroupBox)this.gpbNodeEdit.Controls.Find("gpbPath" + iCol, false)[0];
                // 现有节点移动位置,更改名字
                for (iLoop = PathLevel[iCol]; iLoop > iRow; iLoop--)
                {
                    TextBox tbNodeAddr = (TextBox)gpbPath.Controls.Find("tbNodeAddr" + iCol + (iLoop - 1), false)[0];
                    tbNodeAddr.Name = "tbNodeAddr" + iCol + iLoop;
                    tbNodeAddr.Location = new System.Drawing.Point(iTextBoxNodeAddrPosX, iTextBoxNodeAddrPosY + iLoop * iControlsInterval);
                    Button btInsertNode = (Button)gpbPath.Controls.Find("btInsertNode" + iCol + (iLoop - 1), false)[0];
                    btInsertNode.Name = "btInsertNode" + iCol + iLoop;
                    btInsertNode.Location = new System.Drawing.Point(iButtonInsertNodePosX, iButtonInsertNodePosY + iLoop * iControlsInterval);
                    PictureBox pbLeftLine = (PictureBox)gpbPath.Controls.Find("pbLeftLine" + iCol + (iLoop - 1), false)[0];
                    pbLeftLine.Name = "pbLeftLine" + iCol + iLoop;
                    pbLeftLine.Location = new System.Drawing.Point(iPictureBoxLeftLinePosX, iPictureBoxLeftLinePosY + iLoop * iControlsInterval);
                    if (iLoop < PathLevel[iCol])
                    {
                        Button btDelNode = (Button)gpbPath.Controls.Find("btDelNode" + iCol + (iLoop - 1), false)[0];
                        btDelNode.Name = "btDelNode" + iCol + iLoop;
                        btDelNode.Location = new System.Drawing.Point(iButtonDelNodePosX, iButtonDelNodePosY + iLoop * iControlsInterval);
                        PictureBox pbRightLine = (PictureBox)gpbPath.Controls.Find("pbRightLine" + iCol + (iLoop - 1), false)[0];
                        pbRightLine.Name = "pbRightLine" + iCol + iLoop;
                        pbRightLine.Location = new System.Drawing.Point(iPictureBoxRightLinePosX, iPictureBoxRightLinePosY + iLoop * iControlsInterval);
                    }
                }
                // 增加新节点
                PathLevel[iCol] += 1;
                TextBox tbNewNodeAddr = (TextBox)AddControlItems(iCol, iRow, PathLevel[iCol], "", true);
                tbNewNodeAddr.Focus();
                if (PathLevel[iCol] >= MaxNodeNumOnePath)
                {
                    for (iLoop = 1; iLoop < MaxNodeNumOnePath; iLoop++)
                    {
                        Button btInsertNode = (Button)gpbPath.Controls.Find("btInsertNode" + iCol + iLoop, false)[0];
                        btInsertNode.Enabled = false;
                    }
                }
            }
        }
        private void DelNodeProc(object sender, EventArgs e)
        {
            int iCol, iRow, iLoop;
            Button btDel = (Button)sender;

            string strBtnName = btDel.Name;
            iRow = int.Parse(strBtnName.Substring(strBtnName.Length - 1, 1));
            iCol = int.Parse(strBtnName.Substring(strBtnName.Length - 2, 1));
            GroupBox gpbPath = (GroupBox)this.gpbNodeEdit.Controls.Find("gpbPath" + iCol, false)[0];
            if (PathLevel[iCol] <= 3 && iRow == 1)
            {
                PathLevel[iCol] = 0;
                AddControlItems(iCol, 0, PathLevel[iCol], "", true);
            }
            else
            {
                // 移除选定的控件
                gpbPath.Controls.Remove((TextBox)gpbPath.Controls.Find("tbNodeAddr" + iCol + iRow, false)[0]);
                gpbPath.Controls.Remove((Button)gpbPath.Controls.Find("btDelNode" + iCol + iRow, false)[0]);
                gpbPath.Controls.Remove((Button)gpbPath.Controls.Find("btInsertNode" + iCol + iRow, false)[0]);
                gpbPath.Controls.Remove((PictureBox)gpbPath.Controls.Find("pbLeftLine" + iCol + iRow, false)[0]);
                gpbPath.Controls.Remove((PictureBox)gpbPath.Controls.Find("pbRightLine" + iCol + iRow, false)[0]);
                // 下面的控件上移一个位置并改名
                for (iLoop = iRow; iLoop < PathLevel[iCol] - 1; iLoop++)
                {
                    TextBox tbNodeAddr = (TextBox)gpbPath.Controls.Find("tbNodeAddr" + iCol + (iLoop + 1), false)[0];
                    tbNodeAddr.Name = "tbNodeAddr" + iCol + iLoop;
                    tbNodeAddr.Location = new System.Drawing.Point(iTextBoxNodeAddrPosX, iTextBoxNodeAddrPosY + iLoop * iControlsInterval);
                    Button btInsertNode = (Button)gpbPath.Controls.Find("btInsertNode" + iCol + (iLoop + 1), false)[0];
                    btInsertNode.Name = "btInsertNode" + iCol + iLoop;
                    btInsertNode.Location = new System.Drawing.Point(iButtonInsertNodePosX, iButtonInsertNodePosY + iLoop * iControlsInterval);
                    PictureBox pbLeftLine = (PictureBox)gpbPath.Controls.Find("pbLeftLine" + iCol + (iLoop + 1), false)[0];
                    pbLeftLine.Name = "pbLeftLine" + iCol + iLoop;
                    pbLeftLine.Location = new System.Drawing.Point(iPictureBoxLeftLinePosX, iPictureBoxLeftLinePosY + iLoop * iControlsInterval);
                    if (iLoop < PathLevel[iCol] - 2)
                    {
                        Button btDelNode = (Button)gpbPath.Controls.Find("btDelNode" + iCol + (iLoop + 1), false)[0];
                        btDelNode.Name = "btDelNode" + iCol + iLoop;
                        btDelNode.Location = new System.Drawing.Point(iButtonDelNodePosX, iButtonDelNodePosY + iLoop * iControlsInterval);
                        PictureBox pbRightLine = (PictureBox)gpbPath.Controls.Find("pbRightLine" + iCol + (iLoop + 1), false)[0];
                        pbRightLine.Name = "pbRightLine" + iCol + iLoop;
                        pbRightLine.Location = new System.Drawing.Point(iPictureBoxRightLinePosX, iPictureBoxRightLinePosY + iLoop * iControlsInterval);
                    }
                }
                PathLevel[iCol] -= 1;
                if (PathLevel[iCol] < MaxNodeNumOnePath)
                {
                    for (iLoop = 1; iLoop < PathLevel[iCol]; iLoop++)
                    {
                        Button btInsertNode = (Button)gpbPath.Controls.Find("btInsertNode" + iCol + iLoop, false)[0];
                        btInsertNode.Enabled = true;
                    }
                }
            }
            // 更新数据表
            if (PathLevel[iCol] < 3)
            {
                //tbNodeDoc.Rows[iCurrentNode]["UserDefPath" + iCol] = "";
            }
            else
            {
                TextBox tbNodeAddr = (TextBox)gpbPath.Controls.Find("tbNodeAddr" + iCol + 1, false)[0];
                string strPath = tbNodeAddr.Text;
                for (iLoop = 2; iLoop < PathLevel[iCol]; iLoop++)
                {
                    tbNodeAddr = (TextBox)gpbPath.Controls.Find("tbNodeAddr" + iCol + iLoop, false)[0];
                    strPath += "-" + tbNodeAddr.Text;
                }
                //tbPcNodeDoc.Rows[iCurrentPcNode]["UserDefPath" + iCol] = strPath;
            }
            RefreshDocument();
            //int iUserDefPaths = 0;
            //for (iLoop = 0; iLoop < PathLevel.Length; iLoop++)
            //{
            //    if (PathLevel[iLoop] > 2)
            //    {
            //        iUserDefPaths++;
            //    }
            //}
            //tbPcNodeDoc.Rows[iCurrentPcNode]["UserDefPath"] = iUserDefPaths;
        }
        private void tbNodeAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            TextBox tbNodeAddr = (TextBox)sender;
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
            if (e.KeyChar == '\r' && tbNodeAddr.Text.Length == AddrLength * 2)
            {
                RefreshDocument(); //运行两次
            }
        }
        private void tbNodeAddr_Leave(object sender, EventArgs e)
        {
            TextBox tbNodeAddr = (TextBox)sender;
            tbNodeAddr.Text = tbNodeAddr.Text.PadLeft(AddrLength * 2, '0');
            if (tbNodeAddr.Text.Length == AddrLength * 2)
            {
                RefreshDocument();//运行两次 还有其他地方
            }
        }
        #endregion
        #region 其他档案功能
        private void dgvDoc_SelectionChanged(object sender, EventArgs e)
        {
            if (dgvDoc.SelectedRows.Count > 0)
            {
                tbCurNodeAddr.Text = dgvDoc.SelectedRows[0].Cells["表具地址"].Value.ToString();
                tbCurAddr.Text = tbCurNodeAddr.Text;
                tbCurCmdAddr.Text = tbCurNodeAddr.Text;
                string strNodeType = dgvDoc.SelectedRows[0].Cells["类型"].Value.ToString();
                int index = Array.IndexOf(strMeterType, strNodeType);
                cmbNodeType.Text = "";
                if (index >= 0)
                {
                    cmbNodeType.SelectedIndex = index;
                }
                int iRow, iCol;
                for (iCol = 0; iCol < PathLevel.Length; iCol++)
                {
                    string strPath = dgvDoc.SelectedRows[0].Cells["路径" + iCol].Value.ToString();
                    PathLevel[iCol] = 2;
                    if (strPath.Length == 0)
                    {
                        AddControlItems(iCol, 0, 0, "NULL");
                    }
                    else if (strPath == strNoPath)
                    {
                        AddControlItems(iCol, 0, 0);
                    }
                    else
                    {
                        string[] strPathList = strPath.Split(',');
                        PathLevel[iCol] += strPathList.Length;
                        for (iRow = 0; iRow < PathLevel[iCol]; iRow++)
                        {
                            if (iRow == 0 || iRow == PathLevel[iCol] - 1)
                            {
                                AddControlItems(iCol, iRow, PathLevel[iCol]);
                            }
                            else
                            {
                                AddControlItems(iCol, iRow, PathLevel[iCol], strPathList[iRow - 1], false);
                            }
                        }
                        GroupBox gpbPath = (GroupBox)gpbNodeEdit.Controls.Find("gpbPath" + iCol, false)[0];
                        if (PathLevel[iCol] >= MaxNodeNumOnePath)
                        {
                            for (iRow = 1; iRow < MaxNodeNumOnePath; iRow++)
                            {
                                Button btInsertNode = (Button)gpbPath.Controls.Find("btInsertNode" + iCol + iRow, false)[0];
                                btInsertNode.Enabled = false;
                            }
                        }
                    }
                }
            }
            else
            {
                int iCol;
                for (iCol = 0; iCol < PathLevel.Length; iCol++)
                {
                    PathLevel[iCol] = 2;
                    AddControlItems(iCol, 0, 0);
                }
                tbCurAddr.Text = "";
                tbCurCmdAddr.Text = "";
            }
        }
        private void tbCurNodeAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
            if (e.KeyChar == '\r' && tbCurNodeAddr.Text.Length == AddrLength * 2)
            {
                dgvDoc.ClearSelection();
                foreach (DataGridViewRow row in dgvDoc.Rows)
                {
                    if (row.Cells["表具地址"].Value.ToString() == tbCurNodeAddr.Text)
                    {
                        row.Selected = true;
                        break;
                    }
                }  //zxp 跳出去，没找到 怎么判断 不应该在indexchange那个地方增加

            }
        }
        private void tbCurNodeAddr_Leave(object sender, EventArgs e)
        {
            if (tbCurNodeAddr.Text != "")
            {
                tbCurNodeAddr.Text = tbCurNodeAddr.Text.PadLeft(AddrLength * 2, '0');
            }
            dgvDoc.ClearSelection();
            foreach (DataGridViewRow row in dgvDoc.Rows)
            {
                if (row.Cells["表具地址"].Value.ToString() == tbCurNodeAddr.Text)
                {
                    row.Selected = true;
                    break;
                }
            }
        }
        private void tbNewNodeAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
        }
        private void tbNewNodeAddr_Leave(object sender, EventArgs e)
        {
            if (tbNewNodeAddr.Text != "")
            {
                tbNewNodeAddr.Text = tbNewNodeAddr.Text.PadLeft(AddrLength * 2, '0');
            }
        }
        private void RefreshDocument()
        {
            int iRow, iCol;
            bool bInvalidPath;

            for (iCol = 0; iCol < PathLevel.Length; iCol++)
            {
                GroupBox gpbPath = (GroupBox)(gpbNodeEdit.Controls.Find("gpbPath" + iCol, false))[0];
                string strPath = "";
                bInvalidPath = true;
                if (PathLevel[iCol] > 2)
                {
                    for (iRow = 1; iRow < PathLevel[iCol] - 1; iRow++)
                    {
                        TextBox tbNodeAddr = (TextBox)(gpbPath.Controls.Find("tbNodeAddr" + iCol + iRow, false))[0];
                        if (tbNodeAddr.Text.Length != AddrLength * 2)
                        {
                            bInvalidPath = false;
                            break;
                        }
                        strPath += tbNodeAddr.Text + ",";
                    }
                    if (strPath != "" && bInvalidPath == true)
                    {
                        strPath = strPath.TrimEnd(',');
                    }
                }
                else
                {
                    strPath = strNoPath;
                }
                DataRow[] dr = Document.Select("表具地址='" + tbCurNodeAddr.Text + "'");
                if (dr.Length > 0)
                {
                    if (iCol == 0)
                    {
                        dr[0]["路径0"] = strPath;
                    }
                    else
                    {
                        dr[0]["路径1"] = strPath;
                    }
                }
            }
        }
        #endregion
        #endregion

        #region 数据抄读选项卡
        #region 单个表具定时定量数据
        private void btReadRealData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbCurAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请先输入表具地址。", "表具地址错误");
                return;
            }
            try
            {
                byte[] dataBuf = new byte[AddrLength];
                strCurNodeAddr = tbCurAddr.Text;
                GetByteAddrFromString(strCurNodeAddr, dataBuf, 0);
                byte[] txBuf = ConstructTxBuffer(enCmd.读集中器中的定时定量数据, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------读取单个表具定时定量数据----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：读取单个表具定时定量数据";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取单个表具定时定量数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadRealData(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器中的定时定量数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    string strInfo = "";
                    int iStartPos = 0;
                    byte bResult = rxStruct.DataBuf[iStartPos++];
                    string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iStartPos);
                    iStartPos += AddrLength;
                    DataRow newDr = Document.NewRow();
                    newDr["表具地址"] = strAddr;
                    newDr["结果"] = GetErrorInfo(bResult);
                    if (0xAA == bResult)
                    {
                        string strRdTime = rxStruct.DataBuf[iStartPos++].ToString("X2") + rxStruct.DataBuf[iStartPos++].ToString("X2") + "年" +
                                           rxStruct.DataBuf[iStartPos++].ToString("X2") + "月" + rxStruct.DataBuf[iStartPos++].ToString("X2") + "日" +
                                           rxStruct.DataBuf[iStartPos++].ToString("X2") + "时";
                        string[] strExpInfo = ExplainMeterData(rxStruct.DataBuf, iStartPos, rxStruct.DataBuf.Length - (AddrLength + 1 + 5));
                        int index = 0;
                        newDr["读取时间"] = strRdTime;
                        // 正转用量+反转用量
                        newDr["正转用量"] = strExpInfo[index++];
                        newDr["反转用量"] = strExpInfo[index++];
                        newDr["报警信息"] = strExpInfo[index++];
                        newDr["阀门状态"] = strExpInfo[index++];
                        newDr["电压"] = strExpInfo[index++];
                        newDr["温度"] = strExpInfo[index++];
                        newDr["信噪比"] = strExpInfo[index++];
                        newDr["信道"] = strExpInfo[index++];
                        newDr["版本"] = strExpInfo[index++];
                        newDr["场强↓"] = strExpInfo[index++];
                        newDr["场强↑"] = strExpInfo[index++];
                        strInfo += "\n\t读取时间：" + newDr["读取时间"].ToString();
                        strInfo += "\n\t正转用量：" + newDr["正转用量"].ToString();
                        strInfo += "\n\t反转用量：" + newDr["反转用量"].ToString();
                        strInfo += "\n\t报警信息：" + newDr["报警信息"].ToString();
                        strInfo += "\n\t阀门状态：" + newDr["阀门状态"].ToString();
                        strInfo += "\n\t电压：" + newDr["电压"].ToString();
                        strInfo += "\n\t温度：" + newDr["温度"].ToString();
                        strInfo += "\n\t信噪比：" + newDr["信噪比"].ToString();
                        strInfo += "\n\t信道：" + newDr["信道"].ToString();
                        strInfo += "\n\t版本：" + newDr["版本"].ToString();
                        strInfo += "\n\t场强↓：" + newDr["场强↓"].ToString();
                        strInfo += "\n\t场强↑：" + newDr["场强↑"].ToString();
                    }
                    DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                    if (findDr.Length > 0)
                    {
                        findDr[0]["结果"] = newDr["结果"];
                        findDr[0]["读取时间"] = newDr["读取时间"];
                        findDr[0]["正转用量"] = newDr["正转用量"];
                        findDr[0]["反转用量"] = newDr["反转用量"];
                        findDr[0]["报警信息"] = newDr["报警信息"];
                        findDr[0]["阀门状态"] = newDr["阀门状态"];
                        findDr[0]["电压"] = newDr["电压"];
                        findDr[0]["温度"] = newDr["温度"];
                        findDr[0]["信噪比"] = newDr["信噪比"];
                        findDr[0]["信道"] = newDr["信道"];
                        findDr[0]["版本"] = newDr["版本"];
                        findDr[0]["场强↓"] = newDr["场强↓"];
                        findDr[0]["场强↑"] = newDr["场强↑"];
                    }
                    else
                    {
                        newDr["序号"] = Document.Rows.Count + 1;
                        Document.Rows.Add(newDr);
                    }
                    AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult) + " 数据类型：定时定量", null, Color.DarkBlue);
                    if (0xAA == bResult)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                    }

                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["冻结时间"].Visible = false;
                    dgvDoc.Columns["冻结方式"].Visible = false;
                    dgvDoc.Columns["冻结数量"].Visible = false;
                    dgvDoc.Columns["时间间隔"].Visible = false;
                    dgvDoc.Columns["冻结数据"].Visible = false;
                    dgvDoc.Columns["下发命令状态"].Visible = false;
                    dgvDoc.Columns["信道号"].Visible = false;
                    dgvDoc.Columns["自动信道设置"].Visible = false;
                    dgvDoc.Columns["阀控"].Visible = false;
                    dgvDoc.Columns["实时抄表"].Visible = false;
                    dgvDoc.Columns["结算价格"].Visible = false;
                    dgvDoc.Columns["保留位"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["读取时间"].Visible = true;
                    dgvDoc.Columns["正转用量"].Visible = true;
                    dgvDoc.Columns["反转用量"].Visible = true;

                    dgvDoc.Columns["报警信息"].Visible = true;
                    dgvDoc.Columns["阀门状态"].Visible = true;
                    dgvDoc.Columns["电压"].Visible = true;
                    dgvDoc.Columns["温度"].Visible = true;
                    dgvDoc.Columns["信噪比"].Visible = true;
                    dgvDoc.Columns["信道"].Visible = true;
                    dgvDoc.Columns["版本"].Visible = true;
                    dgvDoc.Columns["场强↓"].Visible = true;
                    dgvDoc.Columns["场强↑"].Visible = true;
                    strDataType = "定量数据";
                    
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读集中器中的定时定量数据出错：" + ex.Message);
            }
        }
        #endregion
        #region 单个表具冻结数据
        private void btReadFreezeData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbCurAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请先输入表具地址。", "表具地址错误");
                return;
            }
            try
            {
                byte[] dataBuf = new byte[AddrLength];
                strCurNodeAddr = tbCurAddr.Text;
                GetByteAddrFromString(strCurNodeAddr, dataBuf, 0);
                byte[] txBuf = ConstructTxBuffer(enCmd.读集中器中的冻结数据, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------读取单个表具冻结数据----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：读取单个表具冻结数据";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取单个表具冻结数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadFreezeData(ProtocolStruct rxStruct)
        {
            int resultLength = 0;
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器中的冻结数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    string strInfo = "";
                    int iStartPos = 0;
                    byte bResult = rxStruct.DataBuf[iStartPos++];
                    string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iStartPos);
                    iStartPos += AddrLength;
                    DataRow newDr = Document.NewRow();
                    newDr["表具地址"] = strAddr;
                    newDr["结果"] = GetErrorInfo(bResult);
                    if (0xAA == bResult)
                    {
                        string[] strExpInfo = ExplainMeterData(rxStruct.DataBuf, iStartPos, rxStruct.DataBuf.Length - (AddrLength + 1));
                        int index = 0;
                        resultLength = strExpInfo.Length;
                        if (resultLength == 15)
                        {
                            // 起始序号+冻结时间+冻结方式+冻结数量+时间间隔+冻结数据
                            newDr["起始序号"] = strExpInfo[index++];
                            newDr["冻结时间"] = strExpInfo[index++];
                            newDr["冻结方式"] = strExpInfo[index++];
                            newDr["冻结数量"] = strExpInfo[index++];
                            newDr["时间间隔"] = strExpInfo[index++];
                            newDr["冻结数据"] = strExpInfo[index++];
                        }
                        else
                        {
                            // 起始序号+冻结时间+冻结数据
                            newDr["起始序号"] = strExpInfo[index++];
                            newDr["冻结时间"] = strExpInfo[index++];
                            newDr["冻结数据"] = strExpInfo[index++];
                        }
                        newDr["报警信息"] = strExpInfo[index++];
                        newDr["阀门状态"] = strExpInfo[index++];
                        newDr["电压"] = strExpInfo[index++];
                        newDr["温度"] = strExpInfo[index++];
                        newDr["信噪比"] = strExpInfo[index++];
                        newDr["信道"] = strExpInfo[index++];
                        newDr["版本"] = strExpInfo[index++];
                        newDr["场强↓"] = strExpInfo[index++];
                        newDr["场强↑"] = strExpInfo[index++];

                        strInfo += "\n\t起始序号：" + newDr["起始序号"].ToString();
                        strInfo += "\n\t冻结时间：" + newDr["冻结时间"].ToString();
                        if (resultLength == 15)
                        {
                            strInfo += "\n\t冻结方式：" + newDr["冻结方式"].ToString();
                            strInfo += "\n\t冻结数量：" + newDr["冻结数量"].ToString();
                            strInfo += "\n\t时间间隔：" + newDr["时间间隔"].ToString();
                        }
                        strInfo += "\n\t冻结数据：" + newDr["冻结数据"].ToString();
                        strInfo += "\n\t报警信息：" + newDr["报警信息"].ToString();
                        strInfo += "\n\t阀门状态：" + newDr["阀门状态"].ToString();
                        strInfo += "\n\t电压：" + newDr["电压"].ToString();
                        strInfo += "\n\t温度：" + newDr["温度"].ToString();
                        strInfo += "\n\t信噪比：" + newDr["信噪比"].ToString();
                        strInfo += "\n\t信道：" + newDr["信道"].ToString();
                        strInfo += "\n\t版本：" + newDr["版本"].ToString();
                        strInfo += "\n\t场强↓：" + newDr["场强↓"].ToString();
                        strInfo += "\n\t场强↑：" + newDr["场强↑"].ToString();
                    }

                    DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                    if (findDr.Length > 0)
                    {
                        findDr[0]["结果"] = newDr["结果"];
                        findDr[0]["起始序号"] = newDr["起始序号"];
                        findDr[0]["冻结时间"] = newDr["冻结时间"];
                        findDr[0]["冻结数据"] = newDr["冻结数据"];
                        if (resultLength == 15)
                        {
                            findDr[0]["冻结方式"] = newDr["冻结方式"];
                            findDr[0]["冻结数量"] = newDr["冻结数量"];
                            findDr[0]["时间间隔"] = newDr["时间间隔"];
                        }
                        findDr[0]["报警信息"] = newDr["报警信息"];
                        findDr[0]["阀门状态"] = newDr["阀门状态"];
                        findDr[0]["电压"] = newDr["电压"];
                        findDr[0]["温度"] = newDr["温度"];
                        findDr[0]["信噪比"] = newDr["信噪比"];
                        findDr[0]["信道"] = newDr["信道"];
                        findDr[0]["版本"] = newDr["版本"];
                        findDr[0]["场强↓"] = newDr["场强↓"];
                        findDr[0]["场强↑"] = newDr["场强↑"];
                    }
                    else
                    {
                        newDr["序号"] = Document.Rows.Count + 1;
                        Document.Rows.Add(newDr);
                    }
                    AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult) + " 数据类型：冻结数据", null, Color.DarkBlue);
                    if (0xAA == bResult)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                    }
                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["读取时间"].Visible = false;
                    dgvDoc.Columns["正转用量"].Visible = false;
                    dgvDoc.Columns["反转用量"].Visible = false;

                    dgvDoc.Columns["下发命令状态"].Visible = false;
                    dgvDoc.Columns["信道号"].Visible = false;
                    dgvDoc.Columns["自动信道设置"].Visible = false;
                    dgvDoc.Columns["阀控"].Visible = false;
                    dgvDoc.Columns["实时抄表"].Visible = false;
                    dgvDoc.Columns["结算价格"].Visible = false;
                    dgvDoc.Columns["保留位"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["报警信息"].Visible = true;
                    dgvDoc.Columns["阀门状态"].Visible = true;
                    dgvDoc.Columns["电压"].Visible = true;
                    dgvDoc.Columns["温度"].Visible = true;
                    dgvDoc.Columns["信噪比"].Visible = true;
                    dgvDoc.Columns["信道"].Visible = true;
                    dgvDoc.Columns["版本"].Visible = true;
                    dgvDoc.Columns["场强↓"].Visible = true;
                    dgvDoc.Columns["场强↑"].Visible = true;


                    
                    dgvDoc.Columns["冻结时间"].Visible = true;
                    dgvDoc.Columns["冻结数据"].Visible = true;
                    if (resultLength == 12)
                    {
                        dgvDoc.Columns["冻结方式"].Visible = false;
                        dgvDoc.Columns["冻结数量"].Visible = false;
                        dgvDoc.Columns["时间间隔"].Visible = false;
                        strDataType = "冻结数据1";
                    }
                    else
                    {
                        dgvDoc.Columns["冻结方式"].Visible = true;
                        dgvDoc.Columns["冻结数量"].Visible = true;
                        dgvDoc.Columns["时间间隔"].Visible = true;
                        strDataType = "冻结数据0";
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    return;
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读集中器中的冻结数据出错：" + ex.Message);
            }
        }
        #endregion
        #region 全部表具定时定量数据
        private void ReadAllRealDataProc()
        {
            byte[] dataBuf = new byte[3];
            dataBuf[0] = (byte)ReadPos;
            dataBuf[1] = (byte)(ReadPos >> 8);
            dataBuf[2] = (byte)("无线通信" == cmbComType.Text ? 5 : 25);
            byte[] txBuf = ConstructTxBuffer(enCmd.批量读取集中器中的定时定量数据, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取全部表具定时定量数据----------------->>>", null, Color.DarkGreen);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取全部表具定时定量数据";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void btReadAllRealData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            try
            {
                ReadPos = 0;
                ReadAllRealDataProc();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具定时定量数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadAllRealData(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量读取集中器中的定时定量数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    int totalNodeCount = rxStruct.DataBuf[0] + rxStruct.DataBuf[1] * 256;
                    int ackNodeCount = rxStruct.DataBuf[2];
                    int iStartPos = 3;
                    int iLen = (rxStruct.DataBuf.Length - iStartPos) / ackNodeCount;    
                    AddStringToCommBox(false, "\n表具总数量：" + totalNodeCount.ToString("D") + " 本次读出数量：" + ackNodeCount.ToString("D"), null, Color.DarkBlue);
                    for (int iLoop = 0; iLoop < ackNodeCount; iLoop++)
                    {
                        string strInfo = "";
                        int iPos = iStartPos;
                        byte bResult = rxStruct.DataBuf[iPos++];
                        string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iPos);
                        iPos += AddrLength;
                        string strRdTime = rxStruct.DataBuf[iPos++].ToString("X2") + rxStruct.DataBuf[iPos++].ToString("X2") + "年" +
                                         rxStruct.DataBuf[iPos++].ToString("X2") + "月" + rxStruct.DataBuf[iPos++].ToString("X2") + "日" +
                                         rxStruct.DataBuf[iPos++].ToString("X2") + "时";
                        DataRow newDr = Document.NewRow();
                        newDr["表具地址"] = strAddr;
                        newDr["结果"] = GetErrorInfo(bResult);
                        if (0xAA == bResult)
                        {
                            string[] strExpInfo = ExplainMeterData(rxStruct.DataBuf, iPos, iLen - (AddrLength + 1 + 5));
                            int index = 0;
                            newDr["读取时间"] = strRdTime;
                            // 正转用量+反转用量
                            newDr["正转用量"] = strExpInfo[index++];
                            newDr["反转用量"] = strExpInfo[index++];
                            newDr["报警信息"] = strExpInfo[index++];
                            newDr["阀门状态"] = strExpInfo[index++];
                            newDr["电压"] = strExpInfo[index++];
                            newDr["温度"] = strExpInfo[index++];
                            newDr["信噪比"] = strExpInfo[index++];
                            newDr["信道"] = strExpInfo[index++];
                            newDr["版本"] = strExpInfo[index++];
                            newDr["场强↓"] = strExpInfo[index++];
                            newDr["场强↑"] = strExpInfo[index++];
                            strInfo += "\n\t读取时间：" + newDr["读取时间"].ToString();
                            strInfo += "\n\t正转用量：" + newDr["正转用量"].ToString();
                            strInfo += "\n\t反转用量：" + newDr["反转用量"].ToString();
                            strInfo += "\n\t报警信息：" + newDr["报警信息"].ToString();
                            strInfo += "\n\t阀门状态：" + newDr["阀门状态"].ToString();
                            strInfo += "\n\t电压：" + newDr["电压"].ToString();
                            strInfo += "\n\t温度：" + newDr["温度"].ToString();
                            strInfo += "\n\t信噪比：" + newDr["信噪比"].ToString();
                            strInfo += "\n\t信道：" + newDr["信道"].ToString();
                            strInfo += "\n\t版本：" + newDr["版本"].ToString();
                            strInfo += "\n\t场强↓：" + newDr["场强↓"].ToString();
                            strInfo += "\n\t场强↑：" + newDr["场强↑"].ToString();
                        }
                            
                        DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                        if (findDr.Length > 0)
                        {
                            findDr[0]["结果"] = newDr["结果"];
                            findDr[0]["读取时间"] = newDr["读取时间"];
                            findDr[0]["正转用量"] = newDr["正转用量"];
                            findDr[0]["反转用量"] = newDr["反转用量"];
                            findDr[0]["报警信息"] = newDr["报警信息"];
                            findDr[0]["阀门状态"] = newDr["阀门状态"];
                            findDr[0]["电压"] = newDr["电压"];
                            findDr[0]["温度"] = newDr["温度"];
                            findDr[0]["信噪比"] = newDr["信噪比"];
                            findDr[0]["信道"] = newDr["信道"];
                            findDr[0]["版本"] = newDr["版本"];
                            findDr[0]["场强↓"] = newDr["场强↓"];
                            findDr[0]["场强↑"] = newDr["场强↑"];
                        }
                        else
                        {
                            newDr["序号"] = Document.Rows.Count + 1;
                            Document.Rows.Add(newDr);
                        } 
                        AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult) + " 数据类型：定时定量", null, Color.DarkBlue);
                        if (0xAA == bResult)
                        {
                            AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                        }
                        iStartPos += iLen;
                    }

                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["冻结时间"].Visible = false;
                    dgvDoc.Columns["冻结方式"].Visible = false;
                    dgvDoc.Columns["冻结数量"].Visible = false;
                    dgvDoc.Columns["时间间隔"].Visible = false;
                    dgvDoc.Columns["冻结数据"].Visible = false;
                    dgvDoc.Columns["下发命令状态"].Visible = false;
                    dgvDoc.Columns["信道号"].Visible = false;
                    dgvDoc.Columns["自动信道设置"].Visible = false;
                    dgvDoc.Columns["阀控"].Visible = false;
                    dgvDoc.Columns["实时抄表"].Visible = false;
                    dgvDoc.Columns["结算价格"].Visible = false;
                    dgvDoc.Columns["保留位"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["读取时间"].Visible = true;
                    dgvDoc.Columns["正转用量"].Visible = true;
                    dgvDoc.Columns["反转用量"].Visible = true;

                    dgvDoc.Columns["报警信息"].Visible = true;
                    dgvDoc.Columns["阀门状态"].Visible = true;
                    dgvDoc.Columns["电压"].Visible = true;
                    dgvDoc.Columns["温度"].Visible = true;
                    dgvDoc.Columns["信噪比"].Visible = true;
                    dgvDoc.Columns["信道"].Visible = true;
                    dgvDoc.Columns["版本"].Visible = true;
                    dgvDoc.Columns["场强↓"].Visible = true;
                    dgvDoc.Columns["场强↑"].Visible = true;


                    strDataType = "定量数据";
                    // 继续读取
                    if (ReadPos + ackNodeCount < totalNodeCount)
                    {
                        ReadPos += ackNodeCount;
                        ReadAllRealDataProc();
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("共读出" + totalNodeCount.ToString("D") + "条定时定量数据", "读取成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具定时定量数据出错：" + ex.Message);
            }
        } 
        #endregion
        #region 全部表具冻结数据
        private void ReadAllFreezeDataProc()
        {
            byte[] dataBuf = new byte[3];
            dataBuf[0] = (byte)ReadPos;
            dataBuf[1] = (byte)(ReadPos >> 8);
            dataBuf[2] = (byte)("无线通信" == cmbComType.Text ? 1 : 6);
            byte[] txBuf = ConstructTxBuffer(enCmd.批量读取集中器中的冻结数据, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取全部表具冻结数据----------------->>>", null, Color.DarkGreen);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取全部表具冻结数据";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void btReadAllFreezeData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            try
            {
                ReadPos = 0;
                ReadAllFreezeDataProc();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具冻结数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadAllFreezeData(ProtocolStruct rxStruct)
        {
            int resultLength = 0;
            int ackNodeCount = 0;
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量读取集中器中的冻结数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    int totalNodeCount = rxStruct.DataBuf[0] + rxStruct.DataBuf[1] * 256;
                    ackNodeCount = rxStruct.DataBuf[2];
                    int iStartPos = 3;
                    int iLen = (rxStruct.DataBuf.Length - iStartPos) / ackNodeCount; 
                    AddStringToCommBox(false, "\n表具总数量：" + totalNodeCount.ToString("D") + " 本次读出数量：" + ackNodeCount.ToString("D"), null, Color.DarkBlue);
                    for (int iLoop = 0; iLoop < ackNodeCount; iLoop++)
                    {
                        string strInfo = "";
                        int iPos = iStartPos;
                        byte bResult = rxStruct.DataBuf[iPos++];
                        string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iPos);
                        iPos += AddrLength;
                        DataRow newDr = Document.NewRow();
                        newDr["表具地址"] = strAddr;
                        newDr["结果"] = GetErrorInfo(bResult);
                        if (0xAA == bResult)
                        {
                            string[] strExpInfo = ExplainMeterData(rxStruct.DataBuf, iPos, iLen - (AddrLength + 1));
                            int index = 0;
                            resultLength = strExpInfo.Length;
                            if (resultLength == 15)
                            {
                                // 起始序号+冻结时间+冻结方式+冻结数量+时间间隔+冻结数据
                                newDr["起始序号"] = strExpInfo[index++];
                                newDr["冻结时间"] = strExpInfo[index++];
                                newDr["冻结方式"] = strExpInfo[index++];
                                newDr["冻结数量"] = strExpInfo[index++];
                                newDr["时间间隔"] = strExpInfo[index++];
                                newDr["冻结数据"] = strExpInfo[index++];
                            }
                            else
                            {
                                // 起始序号+冻结时间+冻结数据
                                newDr["起始序号"] = strExpInfo[index++];
                                newDr["冻结时间"] = strExpInfo[index++];
                                newDr["冻结数据"] = strExpInfo[index++];
                            }
                            newDr["报警信息"] = strExpInfo[index++];
                            newDr["阀门状态"] = strExpInfo[index++];
                            newDr["电压"] = strExpInfo[index++];
                            newDr["温度"] = strExpInfo[index++];
                            newDr["信噪比"] = strExpInfo[index++];
                            newDr["信道"] = strExpInfo[index++];
                            newDr["版本"] = strExpInfo[index++];
                            newDr["场强↓"] = strExpInfo[index++];
                            newDr["场强↑"] = strExpInfo[index++];

                            strInfo += "\n\t起始序号：" + newDr["起始序号"].ToString();
                            strInfo += "\n\t冻结时间：" + newDr["冻结时间"].ToString();
                            if (resultLength == 15)
                            {
                                strInfo += "\n\t冻结方式：" + newDr["冻结方式"].ToString();
                                strInfo += "\n\t冻结数量：" + newDr["冻结数量"].ToString();
                                strInfo += "\n\t时间间隔：" + newDr["时间间隔"].ToString();
                            }
                            strInfo += "\n\t冻结数据：" + newDr["冻结数据"].ToString();
                            strInfo += "\n\t报警信息：" + newDr["报警信息"].ToString();
                            strInfo += "\n\t阀门状态：" + newDr["阀门状态"].ToString();
                            strInfo += "\n\t电压：" + newDr["电压"].ToString();
                            strInfo += "\n\t温度：" + newDr["温度"].ToString();
                            strInfo += "\n\t信噪比：" + newDr["信噪比"].ToString();
                            strInfo += "\n\t信道：" + newDr["信道"].ToString();
                            strInfo += "\n\t版本：" + newDr["版本"].ToString();
                            strInfo += "\n\t场强↓：" + newDr["场强↓"].ToString();
                            strInfo += "\n\t场强↑：" + newDr["场强↑"].ToString();
                        }

                        DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                        if (findDr.Length > 0)
                        {
                            findDr[0]["结果"] = newDr["结果"];
                            findDr[0]["起始序号"] = newDr["起始序号"];
                            findDr[0]["冻结时间"] = newDr["冻结时间"];
                            findDr[0]["冻结数据"] = newDr["冻结数据"];
                            if (resultLength == 15)
                            {
                                findDr[0]["冻结方式"] = newDr["冻结方式"];
                                findDr[0]["冻结数量"] = newDr["冻结数量"];
                                findDr[0]["时间间隔"] = newDr["时间间隔"];
                            }
                            findDr[0]["报警信息"] = newDr["报警信息"];
                            findDr[0]["阀门状态"] = newDr["阀门状态"];
                            findDr[0]["电压"] = newDr["电压"];
                            findDr[0]["温度"] = newDr["温度"];
                            findDr[0]["信噪比"] = newDr["信噪比"];
                            findDr[0]["信道"] = newDr["信道"];
                            findDr[0]["版本"] = newDr["版本"];
                            findDr[0]["场强↓"] = newDr["场强↓"];
                            findDr[0]["场强↑"] = newDr["场强↑"];
                        }
                        else
                        {
                            newDr["序号"] = Document.Rows.Count + 1;
                            Document.Rows.Add(newDr);
                        }
                        AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult) + " 数据类型：冻结数据", null, Color.DarkBlue);
                        if (0xAA == bResult)
                        {
                            AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                        }
                        iStartPos += iLen;
                    }
                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["读取时间"].Visible = false;
                    dgvDoc.Columns["正转用量"].Visible = false;
                    dgvDoc.Columns["反转用量"].Visible = false;
                    dgvDoc.Columns["下发命令状态"].Visible = false;
                    dgvDoc.Columns["信道号"].Visible = false;
                    dgvDoc.Columns["自动信道设置"].Visible = false;
                    dgvDoc.Columns["阀控"].Visible = false;
                    dgvDoc.Columns["实时抄表"].Visible = false;
                    dgvDoc.Columns["结算价格"].Visible = false;
                    dgvDoc.Columns["保留位"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["报警信息"].Visible = true;
                    dgvDoc.Columns["阀门状态"].Visible = true;
                    dgvDoc.Columns["电压"].Visible = true;
                    dgvDoc.Columns["温度"].Visible = true;
                    dgvDoc.Columns["信噪比"].Visible = true;
                    dgvDoc.Columns["信道"].Visible = true;
                    dgvDoc.Columns["版本"].Visible = true;
                    dgvDoc.Columns["场强↓"].Visible = true;
                    dgvDoc.Columns["场强↑"].Visible = true;


                    dgvDoc.Columns["冻结时间"].Visible = true;
                    dgvDoc.Columns["冻结数据"].Visible = true;
                    if (resultLength == 12)
                    {
                        dgvDoc.Columns["冻结方式"].Visible = false;
                        dgvDoc.Columns["冻结数量"].Visible = false;
                        dgvDoc.Columns["时间间隔"].Visible = false;
                        strDataType = "冻结数据1";
                    }
                    else
                    {
                        dgvDoc.Columns["冻结方式"].Visible = true;
                        dgvDoc.Columns["冻结数量"].Visible = true;
                        dgvDoc.Columns["时间间隔"].Visible = true;
                        strDataType = "冻结数据0";
                    }
                    // 继续读取
                    if (ReadPos + ackNodeCount < totalNodeCount)
                    {
                        ReadPos += ackNodeCount;
                        ReadAllFreezeDataProc();
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("共读出" + totalNodeCount.ToString("D") + "条冻结数据", "读取成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具冻结数据出错：" + ex.Message);
            }
        }
        #endregion
        #region 保存所有数据到Excel
        [DllImport("User32.dll", CharSet = CharSet.Auto)]
        public static extern int GetWindowThreadProcessId(IntPtr hwnd, out   int ID);     
        private void btSaveDataToXml_Click(object sender, EventArgs e)
        {
            bool bResult;

            if (false == DeviceStatus(false, true, true))
            {
                return;
            }
            if (Document.Rows.Count == 0)
            {
                MessageBox.Show("没有数据可以保存！", "错误");
                return;
            }
            string strDirectory = Common.XmlHelper.GetNodeDefValue(strConfigFile, "/Config/Global/ExportToXml", System.Windows.Forms.Application.StartupPath);
            saveFileDlg.Filter = "*.xlsx(工作簿)|*.xlsx";
            saveFileDlg.DefaultExt = "xlsx";
            saveFileDlg.FileName = tbConcAddr.Text + "_" + DateTime.Now.ToString("yyMMddHHmm") + strDataType;
            if (saveFileDlg.ShowDialog() == DialogResult.Cancel)
            {
                return;
            }
            string strFileName = saveFileDlg.FileName;
            if (strFileName.Length == 0)
            {
                return;
            }
            if (strDirectory != Path.GetDirectoryName(strFileName))
            {
                strDirectory = Path.GetDirectoryName(strFileName);
                Common.XmlHelper.SetNodeValue(strConfigFile, "/Config/Global", "ExportToXml", strDirectory);
            }

            Microsoft.Office.Interop.Excel.Application appExcel = new Microsoft.Office.Interop.Excel.Application();
            System.Reflection.Missing miss = System.Reflection.Missing.Value;
            Microsoft.Office.Interop.Excel.Workbook workbookdata = null;
            Microsoft.Office.Interop.Excel.Worksheet worksheetdata = null;
            Microsoft.Office.Interop.Excel.Range rangedata;
            try
            {
                if (appExcel == null)
                {
                    return;
                }
                //设置对象不可见
                appExcel.Visible = false;
                appExcel.DisplayAlerts = false;
                //System.Globalization.CultureInfo currentci = System.Threading.Thread.CurrentThread.CurrentCulture;
                //System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-us");
                workbookdata = appExcel.Workbooks.Add(miss);
                worksheetdata = (Microsoft.Office.Interop.Excel.Worksheet)workbookdata.Worksheets.Add(miss, miss, miss, miss);
                appExcel.ActiveWindow.DisplayGridlines = false;

                // 设置字号,文本显示
                rangedata = worksheetdata.get_Range("A1", "Z1200");
                rangedata.Font.Size = 10;
                rangedata.NumberFormatLocal = "@";
                rangedata.HorizontalAlignment = XlHAlign.xlHAlignCenter;

                // 设置列宽
                char cColumn = 'A';
                int iColumn = 1;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 5;              // 序号
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["序号"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 13.5;           // 地址
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["表具地址"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 7;              // 类型
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["类型"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 12;             // 路径0
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["路径0"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 12;             // 路径1
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["路径1"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 12;             // 结果
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["结果"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;              // 下行场强
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["场强↓"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;              // 上行场强
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["场强↑"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 12;             // 报警状态
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["报警信息"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;              // 阀状态
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["阀门状态"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 5.5;            // 电压
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["电压"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 5.5;            // 温度
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["温度"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 5.5;            // 信噪比
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["信噪比"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 12;             // 信道
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["信道"].HeaderText;
                worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;              // 版本
                worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["版本"].HeaderText;
                if ("定量数据" == strDataType)
                {
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 16;         // 抄读日期
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["读取时间"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 10;         // 正转用量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["正转用量"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 10;         // 反转用量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["反转用量"].HeaderText;
                }
                else if ("冻结数据0" == strDataType)
                {
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 起始序号
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["起始序号"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 16;         // 冻结日期
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结时间"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 冻结方式
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结方式"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 冻结数量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结数量"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 10;          // 时间间隔
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["时间间隔"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").ColumnWidth = 35;            // 冻结数据
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结数据"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").HorizontalAlignment = XlHAlign.xlHAlignLeft;
//                    worksheetdata.get_Range(cColumn.ToString() + "1").EntireColumn.AutoFit();
                    cColumn++;
                }
                else if ("冻结数据1" == strDataType)
                {
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 起始序号
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["起始序号"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 16;         // 冻结日期
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结时间"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").ColumnWidth = 35;           // 冻结数据
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结数据"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").HorizontalAlignment = XlHAlign.xlHAlignLeft;
//                    worksheetdata.get_Range(cColumn.ToString() + "1").EntireColumn.AutoFit();
                    cColumn++;
                }
                else
                {
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 16;         // 抄读日期
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["读取时间"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 10;         // 正转用量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["正转用量"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 10;         // 反转用量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["反转用量"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 起始序号
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["起始序号"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 16;         // 冻结日期
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结时间"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 冻结方式
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结方式"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 冻结数量
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结数量"].HeaderText;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").ColumnWidth = 8;          // 时间间隔
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["时间间隔"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").ColumnWidth = 35;           // 冻结数据
                    worksheetdata.Cells[1, iColumn++] = dgvDoc.Columns["冻结数据"].HeaderText;
                    worksheetdata.get_Range(cColumn.ToString() + "1").HorizontalAlignment = XlHAlign.xlHAlignLeft;
                    worksheetdata.get_Range(cColumn++.ToString() + "1").EntireColumn.AutoFit();
                }
                
                // 冻结
                cColumn--;
                appExcel.ActiveWindow.SplitRow = 1;
                appExcel.ActiveWindow.SplitColumn = 2;
                appExcel.ActiveWindow.FreezePanes = true;
                // 背景色,不能超过Z，否则按下面方法处理
                rangedata = worksheetdata.get_Range("A1", cColumn.ToString() + "1");
                rangedata.Font.Bold = true;
                rangedata.Interior.ColorIndex = 35;
                for (int i = 0; i < Document.Rows.Count; i++ )
                {
                    if (i %2 == 0)
                    {
                        continue;
                    }
                    rangedata = worksheetdata.get_Range("A" + (i + 2).ToString(), cColumn.ToString() + (i + 2).ToString());
                    rangedata.Interior.ColorIndex = 19;
                }

                // 画表格
                if (cColumn > 'Z')
                {
                    rangedata = worksheetdata.get_Range("A1", "A" + ((char)(cColumn - 'Z' + 'A' - 1)).ToString() + (Document.Rows.Count + 1).ToString());
                }
                else
                {
                    rangedata = worksheetdata.get_Range("A1", cColumn.ToString() + (Document.Rows.Count + 1).ToString());
                }
                rangedata.Borders.LineStyle = Microsoft.Office.Interop.Excel.XlLineStyle.xlContinuous;
                rangedata.Borders.get_Item(Microsoft.Office.Interop.Excel.XlBordersIndex.xlEdgeTop).Weight = XlBorderWeight.xlMedium;
                rangedata.Borders.get_Item(Microsoft.Office.Interop.Excel.XlBordersIndex.xlEdgeBottom).Weight = XlBorderWeight.xlMedium;
                rangedata.Borders.get_Item(Microsoft.Office.Interop.Excel.XlBordersIndex.xlEdgeLeft).Weight = XlBorderWeight.xlMedium;
                rangedata.Borders.get_Item(Microsoft.Office.Interop.Excel.XlBordersIndex.xlEdgeRight).Weight = XlBorderWeight.xlMedium;

                // 填参数数据
                //worksheetdata.Cells[1, 1] = "创建时间：";
                //worksheetdata.Cells[1, 3] = DateTime.Now.ToString("yyyy/MM/dd HH:mm:ss").ToString();
                //worksheetdata.Cells[2, 1] = "集中器地址：";
                //worksheetdata.Cells[2, 3] = tbConcAddr.Text;
                //worksheetdata.Cells[3, 1] = "版本信息：";
                //worksheetdata.Cells[3, 3] = "软件:" + tbSwVer.Text + " 硬件:" + tbHwVer.Text + " 协议:" + tbPtVer.Text;
                //worksheetdata.Cells[4, 1] = "工作模式：";
                //worksheetdata.Cells[4, 3] = tbWorkMode.Text;

                // 填表数据
                worksheetdata.Name = tbConcAddr.Text + "_" + strDataType;
                //int iColumn = 1;
                //for (int i = 0; i < tbDocument.Columns.Count; i++ )
                //{
                //    if (strDataType == "定量数据")
                //    {
                //        if (tbDocument.Columns[i].ColumnName == "ColStartNo" ||
                //            tbDocument.Columns[i].ColumnName == "ColFreezeTime" ||
                //            tbDocument.Columns[i].ColumnName == "ColIncrementVal")
                //        {
                //            continue;
                //        }
                //    }
                //    else if ("冻结数据" == strDataType)
                //    {
                //        if (tbDocument.Columns[i].ColumnName == "ColRdTime" ||
                //            tbDocument.Columns[i].ColumnName == "ColFwdValue" ||
                //            tbDocument.Columns[i].ColumnName == "ColRwdValue")
                //        {
                //            continue;
                //        }
                //    }
                //    worksheetdata.Cells[6, iColumn++] = dgvDoc.Columns[i].HeaderText.ToString();
                //}

                //在内存中声明一个数组
                object[,] objval = new object[Document.Rows.Count, iColumn];
                int iCol = 0;
                for (int iRow = 0; iRow < Document.Rows.Count; iRow++ )
                {
                    iCol = 0;
                    DataRow dr = Document.Rows[iRow];
                    objval[iRow, iCol++] = iRow.ToString("D4");
                    objval[iRow, iCol++] = dr["表具地址"].ToString();
                    objval[iRow, iCol++] = dr["类型"].ToString();    
                    objval[iRow, iCol++] = dr["路径0"].ToString();
                    objval[iRow, iCol++] = dr["路径1"].ToString();
                    objval[iRow, iCol++] = dr["结果"].ToString();
                    objval[iRow, iCol++] = dr["场强↓"].ToString();
                    objval[iRow, iCol++] = dr["场强↑"].ToString();
                    objval[iRow, iCol++] = dr["报警信息"].ToString();
                    objval[iRow, iCol++] = dr["阀门状态"].ToString();
                    objval[iRow, iCol++] = dr["电压"].ToString();
                    objval[iRow, iCol++] = dr["温度"].ToString();
                    objval[iRow, iCol++] = dr["信噪比"].ToString();
                    objval[iRow, iCol++] = dr["信道"].ToString();
                    objval[iRow, iCol++] = dr["版本"].ToString();
                    if ("定量数据" == strDataType)
                    {
                        objval[iRow, iCol++] = dr["读取时间"].ToString();
                        objval[iRow, iCol++] = dr["正转用量"].ToString();
                        objval[iRow, iCol++] = dr["反转用量"].ToString();
                    }
                    else if ("冻结数据0" == strDataType)
                    {
                        objval[iRow, iCol++] = dr["起始序号"].ToString();
                        objval[iRow, iCol++] = dr["冻结时间"].ToString();
                        objval[iRow, iCol++] = dr["冻结方式"].ToString();
                        objval[iRow, iCol++] = dr["冻结数量"].ToString();
                        objval[iRow, iCol++] = dr["时间间隔"].ToString();
                        objval[iRow, iCol++] = dr["冻结数据"].ToString();
                    }
                    else if ("冻结数据1" == strDataType)
                    {
                        objval[iRow, iCol++] = dr["起始序号"].ToString();
                        objval[iRow, iCol++] = dr["冻结时间"].ToString();
                        objval[iRow, iCol++] = dr["冻结数据"].ToString();
                    }
                    else
                    {
                        objval[iRow, iCol++] = dr["读取时间"].ToString();
                        objval[iRow, iCol++] = dr["正转用量"].ToString();
                        objval[iRow, iCol++] = dr["反转用量"].ToString();
                        objval[iRow, iCol++] = dr["起始序号"].ToString();
                        objval[iRow, iCol++] = dr["冻结时间"].ToString();
                        objval[iRow, iCol++] = dr["冻结方式"].ToString();
                        objval[iRow, iCol++] = dr["冻结数量"].ToString();
                        objval[iRow, iCol++] = dr["时间间隔"].ToString();
                        objval[iRow, iCol++] = dr["冻结数据"].ToString();
                    }
                    
                    System.Windows.Forms.Application.DoEvents();
                }
                if ('A' + iCol - 1 > 'Z')
                {
                    rangedata = worksheetdata.get_Range("A2", "A" + ((char)('A' + iCol - 1 - 'Z' + 'A' - 1)).ToString() + (dgvDoc.Rows.Count + 1).ToString());
                }
                else
                {
                    rangedata = worksheetdata.get_Range("A2", ((char)('A' + iCol - 1)).ToString() + (Document.Rows.Count + 1).ToString());
                }
                rangedata.Value2 = objval;

                //保存工作表
                System.Runtime.InteropServices.Marshal.ReleaseComObject(rangedata);
                rangedata = null;

                //调用方法关闭excel进程
                //appExcel.Visible = true;
                //appExcel.ActiveWorkbook.SaveAs(strFileName);
                appExcel.ActiveWorkbook.SaveAs(strFileName, Microsoft.Office.Interop.Excel.XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Microsoft.Office.Interop.Excel.XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);
                //xlbook.saveas(filepath, microsoft.office.interop.excel .xlfileformat.xlexcel 8,        type.missing, type.missing, type.missing, type.missing, excel .xlsaveasaccessmode.xlnochange, type.missing, type.missing, type.missing, type.missing, type.missing);
                //xlbook.saveas(filepath, microsoft.office.interop.excel .xlfileformat.xlworkbooknormal, type.missing, type.missing, type.missing, type.missing, excel .xlsaveasaccessmode.xlnochange, type.missing, type.missing, type.missing, type.missing, type.missing);
                //appExcel.SaveWorkspace();
                bResult = true;
            }
            catch (Exception ex)
            {
                MessageBox.Show("导出数据到文件发生错误，" + ex.Message, "出错了", MessageBoxButtons.OK, MessageBoxIcon.Information);
                bResult = false;
            }
            finally
            {
                workbookdata.Close(false, miss, miss);
                appExcel.Workbooks.Close();
                appExcel.Quit();
                IntPtr t = new IntPtr(appExcel.Hwnd);          //杀死进程的好方法，很有效
                int k = 0;
                GetWindowThreadProcessId(t, out k);
                System.Diagnostics.Process p = System.Diagnostics.Process.GetProcessById(k);
                p.Kill();
            }
            if (false == bResult)
            {
                return;
            }
            if (DialogResult.Yes == MessageBox.Show("导出文件成功！是否打开这个文件？", "导出成功", MessageBoxButtons.YesNo, MessageBoxIcon.Question))
            {
                Process.Start(strFileName);
            }
        }
        #endregion

        #region 单个表具下发命令数据
        private void btReadCmdData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (tbCurCmdAddr.Text.Length != AddrLength * 2)
            {
                MessageBox.Show("请先输入表具地址。", "表具地址错误");
                return;
            }
            try
            {
                byte[] dataBuf = new byte[AddrLength];
                strCurNodeAddr = tbCurCmdAddr.Text;
                GetByteAddrFromString(strCurNodeAddr, dataBuf, 0);
                byte[] txBuf = ConstructTxBuffer(enCmd.读集中器中的下发命令数据, NeedAck, null, dataBuf);
                DataTransmit(txBuf);
                AddStringToCommBox(false, "\n<<<-----------------读取单个表具下发命令数据----------------->>>", null, Color.DarkGreen, true);
                AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                lbCurState.Text = "设备状态：读取单个表具下发命令数据";
                ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取单个表具下发命令数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadCmdData(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器中的下发命令数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    string strInfo = "";
                    int iStartPos = 0;
                    byte bResult = rxStruct.DataBuf[iStartPos++];
                    string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iStartPos);
                    iStartPos += AddrLength;
                    DataRow newDr = Document.NewRow();
                    newDr["表具地址"] = strAddr;
                    newDr["结果"] = GetErrorInfo(bResult);
                    if (0xAA == bResult)
                    {
                        byte CmdStatus = rxStruct.DataBuf[iStartPos++];
                        newDr["下发命令状态"] = "";
                        if ( 0x1 == (CmdStatus & 0x1))
                        {
                            newDr["下发命令状态"] = "设置信道 + ";
                        }
                        if (0x2 == (CmdStatus & 0x2))
                        {
                            newDr["下发命令状态"] += "开关自动信道 + ";
                        }
                        if (0x4 == (CmdStatus & 0x4))
                        {
                            newDr["下发命令状态"] += "阀控 + ";
                        }
                        if (0x8 == (CmdStatus & 0x8))
                        {
                            newDr["下发命令状态"] += "开关实时抄表 + ";
                        }
                        if (0x10 == (CmdStatus & 0x10))
                        {
                            newDr["下发命令状态"] += "结算价格";
                        }
                        byte CmdSetChannel = rxStruct.DataBuf[iStartPos++];
                        newDr["信道号"] = CmdSetChannel&0xF;
                        if (0x0 == (CmdStatus & 0x1))
                        {
                            newDr["信道号"] = "";
                        }

                        byte OpenAutoChannel = rxStruct.DataBuf[iStartPos++];
                        if ((0x2 == (CmdStatus & 0x2)) && (0x80 == OpenAutoChannel))
                        {
                            newDr["自动信道设置"] = "开自动信道";
                        }
                        else if ((0x2 == (CmdStatus & 0x2)) && (0x0 == OpenAutoChannel))
                        {
                            newDr["自动信道设置"] = "关自动信道";
                        }
                        else
                        {
                            newDr["自动信道设置"] = "";
                        }

                        byte CmdValveCtrl = rxStruct.DataBuf[iStartPos++];
                        if ( (0x4 == (CmdStatus & 0x4)) &&  (0x1 == CmdValveCtrl))
                        {
                            newDr["阀控"] = "开阀";
                        }
                        else if ((0x4 == (CmdStatus & 0x4)) && (0x2 == CmdValveCtrl))
                        {
                            newDr["阀控"] = "关阀";
                        }
                        else
                        {
                            newDr["阀控"] = "";
                        }

                        byte RealTimeMeterRead = rxStruct.DataBuf[iStartPos++];
                        if ((0x8 == (CmdStatus & 0x8)) && (0x80 == RealTimeMeterRead))
                        {
                            newDr["实时抄表"] = "开实时抄表";
                        }
                        else if ((0x8 == (CmdStatus & 0x8)) && (0x0 == RealTimeMeterRead))
                        {
                            newDr["实时抄表"] = "关实时抄表";
                        }
                        else
                        {
                            newDr["实时抄表"] = "";
                        }

                        byte cbSettlementPrice0 = rxStruct.DataBuf[iStartPos++];
                        byte cbSettlementPrice1 = rxStruct.DataBuf[iStartPos++];
                        if (0x10 == (CmdStatus & 0x10))
                        {
                            newDr["结算价格"] = Bcd2Hex(cbSettlementPrice1).ToString() + '.' + Bcd2Hex(cbSettlementPrice0).ToString() + "元";
                        }
                        else
                        {
                            newDr["结算价格"] = "";
                        }

                        iStartPos += 2;
                        byte autoChannelSwitch = rxStruct.DataBuf[iStartPos++];
                        if (autoChannelSwitch == 0x1)
                        {
                            newDr["表具状态"] = "自动信道已打开";
                        }
                        else
                        {
                            newDr["表具状态"] = "自动信道关闭中";
                        }
                        strInfo += "\n\t下发命令状态：" + newDr["下发命令状态"].ToString();
                        strInfo += "\n\t信道号：" + newDr["信道号"].ToString();
                        strInfo += "\n\t自动信道设置：" + newDr["自动信道设置"].ToString();
                        strInfo += "\n\t阀控：" + newDr["阀控"].ToString();
                        strInfo += "\n\t实时抄表：" + newDr["实时抄表"].ToString();
                        strInfo += "\n\t结算价格：" + newDr["结算价格"].ToString();
                        strInfo += "\n\t表具状态：" + newDr["表具状态"].ToString();
                    }
                    DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                    if (findDr.Length > 0)
                    {
                        findDr[0]["结果"] = newDr["结果"];
                        findDr[0]["下发命令状态"] = newDr["下发命令状态"];
                        findDr[0]["信道号"] = newDr["信道号"];
                        findDr[0]["自动信道设置"] = newDr["自动信道设置"];
                        findDr[0]["阀控"] = newDr["阀控"];
                        findDr[0]["实时抄表"] = newDr["实时抄表"];
                        findDr[0]["结算价格"] = newDr["结算价格"];
                        findDr[0]["表具状态"] = newDr["表具状态"];
                    }
                    else
                    {
                        newDr["序号"] = Document.Rows.Count + 1;
                        Document.Rows.Add(newDr);
                    }
                    AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult), null, Color.DarkBlue);
                    if (0xAA == bResult)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                    }
                    dgvDoc.Columns["路径0"].Visible = false;
                    dgvDoc.Columns["路径1"].Visible = false;
                    dgvDoc.Columns["场强↓"].Visible = false;
                    dgvDoc.Columns["场强↑"].Visible = false;
                    dgvDoc.Columns["报警信息"].Visible = false;
                    dgvDoc.Columns["阀门状态"].Visible = false;
                    dgvDoc.Columns["电压"].Visible = false;
                    dgvDoc.Columns["温度"].Visible = false;
                    dgvDoc.Columns["信噪比"].Visible = false;
                    dgvDoc.Columns["信道"].Visible = false;
                    dgvDoc.Columns["版本"].Visible = false;
                    dgvDoc.Columns["读取时间"].Visible = false;
                    dgvDoc.Columns["正转用量"].Visible = false;
                    dgvDoc.Columns["反转用量"].Visible = false;
                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["冻结时间"].Visible = false;
                    dgvDoc.Columns["冻结方式"].Visible = false;
                    dgvDoc.Columns["冻结数量"].Visible = false;
                    dgvDoc.Columns["时间间隔"].Visible = false;
                    dgvDoc.Columns["冻结数据"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["结果"].Visible = true;
                    dgvDoc.Columns["下发命令状态"].Visible = true;
                    dgvDoc.Columns["信道号"].Visible = true;
                    dgvDoc.Columns["自动信道设置"].Visible = true;
                    dgvDoc.Columns["阀控"].Visible = true;
                    dgvDoc.Columns["实时抄表"].Visible = true;
                    dgvDoc.Columns["结算价格"].Visible = true;
                    dgvDoc.Columns["表具状态"].Visible = true;
                    dgvDoc.Columns["保留位"].Visible = false;

                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读集中器中的下发命令数据出错：" + ex.Message);
            }
        }
        #endregion
        #region 全部表具下发命令数据
        private void ReadAllCmdDataProc()
        {
            byte[] dataBuf = new byte[3];
            dataBuf[0] = (byte)ReadPos;
            dataBuf[1] = (byte)(ReadPos >> 8);
            dataBuf[2] = (byte)("无线通信" == cmbComType.Text ? 5 : 25);
            byte[] txBuf = ConstructTxBuffer(enCmd.批量读取集中器中的下发命令数据, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读取全部表具下发命令数据----------------->>>", null, Color.DarkGreen);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读取全部表具下发命令数据";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void btReadAllCmdData_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            try
            {
                ReadPos = 0;
                ReadAllCmdDataProc();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具下发命令数据时出错：" + ex.Message, "错误");
            }
        }
        private void ExplainReadAllCmdData(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量读取集中器中的下发命令数据)
                {
                    if (rxStruct.DataBuf.Length == 1)
                    {
                        AddStringToCommBox(false, "\n执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), null, Color.DarkBlue);
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("执行结果：" + GetErrorInfo(rxStruct.DataBuf[0]), "发生错误");
                        return;
                    }
                    int totalNodeCount = rxStruct.DataBuf[0] + rxStruct.DataBuf[1] * 256;
                    int ackNodeCount = rxStruct.DataBuf[2];
                    int iStartPos = 3;
                    int iLen = (rxStruct.DataBuf.Length - iStartPos) / ackNodeCount;
                    AddStringToCommBox(false, "\n表具总数量：" + totalNodeCount.ToString("D") + " 本次读出数量：" + ackNodeCount.ToString("D"), null, Color.DarkBlue);
                    for (int iLoop = 0; iLoop < ackNodeCount; iLoop++)
                    {
                        string strInfo = "";
                        int iPos = iStartPos;
                        byte bResult = rxStruct.DataBuf[iPos++];
                        string strAddr = GetStringAddrFromByte(rxStruct.DataBuf, iPos);
                        iPos += AddrLength;
                        DataRow newDr = Document.NewRow();
                        newDr["表具地址"] = strAddr;
                        newDr["结果"] = GetErrorInfo(bResult);
                        if (0xAA == bResult)
                        {

                            byte CmdStatus = rxStruct.DataBuf[iPos++];
                            newDr["下发命令状态"] = "";
                            if (0x1 == (CmdStatus & 0x1))
                            {
                                newDr["下发命令状态"] = "设置信道 + ";
                            }
                            if (0x2 == (CmdStatus & 0x2))
                            {
                                newDr["下发命令状态"] += "开关自动信道 + ";
                            }
                            if (0x4 == (CmdStatus & 0x4))
                            {
                                newDr["下发命令状态"] += "阀控 + ";
                            }
                            if (0x8 == (CmdStatus & 0x8))
                            {
                                newDr["下发命令状态"] += "开关实时抄表 + ";
                            }
                            if (0x10 == (CmdStatus & 0x10))
                            {
                                newDr["下发命令状态"] += "结算价格";
                            }

                            byte CmdSetChannel = rxStruct.DataBuf[iPos++];
                            newDr["信道号"] = CmdSetChannel & 0xF;
                            if (0x0 == (CmdStatus & 0x1))
                            {
                                newDr["信道号"] = "";
                            }
                            byte OpenAutoChannel = rxStruct.DataBuf[iPos++];
                            if ((0x2 == (CmdStatus & 0x2)) && (0x80 == (OpenAutoChannel & 0x80)))
                            {
                                newDr["自动信道设置"] = "开自动信道";
                            }
                            else if ((0x2 == (CmdStatus & 0x2)) && (0x0 == (OpenAutoChannel & 0x80)))
                            {
                                newDr["自动信道设置"] = "关自动信道";
                            }
                            else
                            {
                                newDr["自动信道设置"] = "";
                            }

                            byte CmdValveCtrl = rxStruct.DataBuf[iPos++];
                            if ((0x4 == (CmdStatus & 0x4)) && (0x1 == CmdValveCtrl))
                            {
                                newDr["阀控"] = "开阀";
                            }
                            else if ((0x4 == (CmdStatus & 0x4)) && (0x2 == CmdValveCtrl))
                            {
                                newDr["阀控"] = "关阀";
                            }
                            else
                            {
                                newDr["阀控"] = "";
                            }

                            byte RealTimeMeterRead = rxStruct.DataBuf[iPos++];
                            if ((0x8 == (CmdStatus & 0x8)) && (0x80 == RealTimeMeterRead))
                            {
                                newDr["实时抄表"] = "开实时抄表";
                            }
                            else if ((0x8 == (CmdStatus & 0x8)) && (0x0 == RealTimeMeterRead))
                            {
                                newDr["实时抄表"] = "关实时抄表";
                            }
                            else
                            {
                                newDr["实时抄表"] = "";
                            }

                            byte cbSettlementPrice0 = rxStruct.DataBuf[iPos++];
                            byte cbSettlementPrice1 = rxStruct.DataBuf[iPos++];
                            if (0x10 == (CmdStatus & 0x10))
                            {
                                newDr["结算价格"] = Bcd2Hex(cbSettlementPrice1).ToString() + '.' + Bcd2Hex(cbSettlementPrice0).ToString() + "元";
                            }
                            else
                            {
                                newDr["结算价格"] = "";
                            }

                            iPos += 2; // 保留位
                            byte autoChannelSwitch = rxStruct.DataBuf[iPos++];
                            if (autoChannelSwitch == 0x1)
                            {
                                newDr["表具状态"] = "自动信道已打开";
                            }
                            else
                            {
                                newDr["表具状态"] = "自动信道关闭中";
                            }
                            strInfo += "\n\t下发命令状态：" + newDr["下发命令状态"].ToString();
                            strInfo += "\n\t信道号：" + newDr["信道号"].ToString();
                            strInfo += "\n\t自动信道设置：" + newDr["自动信道设置"].ToString();
                            strInfo += "\n\t阀控：" + newDr["阀控"].ToString();
                            strInfo += "\n\t实时抄表：" + newDr["实时抄表"].ToString();
                            strInfo += "\n\t结算价格：" + newDr["结算价格"].ToString();
                            strInfo += "\n\t表具状态：" + newDr["表具状态"].ToString();
                        }

                        DataRow[] findDr = Document.Select("表具地址='" + strAddr + "'");
                        if (findDr.Length > 0)
                        {
                            findDr[0]["结果"] = newDr["结果"];
                            findDr[0]["下发命令状态"] = newDr["下发命令状态"];
                            findDr[0]["信道号"] = newDr["信道号"];
                            findDr[0]["自动信道设置"] = newDr["自动信道设置"];
                            findDr[0]["阀控"] = newDr["阀控"];
                            findDr[0]["实时抄表"] = newDr["实时抄表"];
                            findDr[0]["结算价格"] = newDr["结算价格"];
                            findDr[0]["表具状态"] = newDr["表具状态"];
                        }
                        else
                        {
                            newDr["序号"] = Document.Rows.Count + 1;
                            Document.Rows.Add(newDr);
                        }
                        AddStringToCommBox(false, "\n 表具地址：" + strAddr + " 执行结果：" + GetErrorInfo(bResult) + " 数据类型：下发命令", null, Color.DarkBlue);
                        if (0xAA == bResult)
                        {
                            AddStringToCommBox(false, strInfo, null, Color.DarkGreen);
                        }
                        iStartPos += iLen;
                    }

                    dgvDoc.Columns["路径0"].Visible = false;
                    dgvDoc.Columns["路径1"].Visible = false;
                    dgvDoc.Columns["场强↓"].Visible = false;
                    dgvDoc.Columns["场强↑"].Visible = false;
                    dgvDoc.Columns["报警信息"].Visible = false;
                    dgvDoc.Columns["阀门状态"].Visible = false;
                    dgvDoc.Columns["电压"].Visible = false;
                    dgvDoc.Columns["温度"].Visible = false;
                    dgvDoc.Columns["信噪比"].Visible = false;
                    dgvDoc.Columns["信道"].Visible = false;
                    dgvDoc.Columns["版本"].Visible = false;
                    dgvDoc.Columns["读取时间"].Visible = false;
                    dgvDoc.Columns["正转用量"].Visible = false;
                    dgvDoc.Columns["反转用量"].Visible = false;
                    dgvDoc.Columns["起始序号"].Visible = false;
                    dgvDoc.Columns["冻结时间"].Visible = false;
                    dgvDoc.Columns["冻结方式"].Visible = false;
                    dgvDoc.Columns["冻结数量"].Visible = false;
                    dgvDoc.Columns["时间间隔"].Visible = false;
                    dgvDoc.Columns["冻结数据"].Visible = false;

                    dgvDoc.Columns["序号"].Visible = true;
                    dgvDoc.Columns["结果"].Visible = true;
                    dgvDoc.Columns["下发命令状态"].Visible = true;
                    dgvDoc.Columns["信道号"].Visible = true;
                    dgvDoc.Columns["自动信道设置"].Visible = true;
                    dgvDoc.Columns["阀控"].Visible = true;
                    dgvDoc.Columns["实时抄表"].Visible = true;
                    dgvDoc.Columns["结算价格"].Visible = true;
                    dgvDoc.Columns["表具状态"].Visible = true;
                    dgvDoc.Columns["保留位"].Visible = false;

                    // 继续读取
                    if (ReadPos + ackNodeCount < totalNodeCount)
                    {
                        ReadPos += ackNodeCount;
                        ReadAllCmdDataProc();
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                        MessageBox.Show("共读出" + totalNodeCount.ToString("D") + "条下发命令数据", "读取成功");
                    }
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读取全部表具下发命令数据出错：" + ex.Message);
            }
        }
        #endregion
        #region 下发命令到集中器
        private void RunLssuedCmdProc()
        {
            if (strDataFwdNodeAddrList != null && strDataFwdNodeAddrList.Length > 0)
            {
                strCurDataFwdAddr = strDataFwdNodeAddrList[0];
                if (strCurDataFwdAddr.Length != AddrLength * 2)
                {
                    MessageBox.Show("当前表具地址中的数据不合法！", "地址错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    btBatchWriteMeterCmd.Text = "下发命令";
                    cbRunCmdNodeAddr.Enabled = true;
                    strDataFwdNodeAddrList = null;
                    strCurDataFwdAddr = "";
                    return;
                }
                //btBatchWriteMeterCmd.Text = "停止命令";
                //cbRunCmdNodeAddr.Enabled = false;
                try
                {
                    byte[] dataBuf = new byte[1200];
                    int iLen = 0;
                    int iNum = "无线通信" == cmbComType.Text ? 150 : 700;
                    byte chioce = 0;
                    
                    for (int iLoop = 0; iLoop < strDataFwdNodeAddrList.Length; iLoop++)
                    {
                        strCurDataFwdAddr = strDataFwdNodeAddrList[iLoop];
                        iLen += GetByteAddrFromString(strCurDataFwdAddr, dataBuf, iLen);

                        if ( rdbSetChannel1.Checked == true) // 设置信道选择
                        {
                            chioce |= 0x1;
                            byte rdbChannel = (byte)cbChannelNum.Value;
                            if (rdbAutoSetChannel.Checked == true)
                            {
                                // 自动设置信道，信道号为集中器地址的最后一位
                                cbChannelNum.Value = Convert.ToByte(tbConcAddr.Text.Substring(tbConcAddr.Text.Length - 1)) & 0xF;
                            }
                        }
                        if (rdbOpenAutoChannel2.Checked != true) // 开启自动信道设置选择
                        {
                            chioce |= 0x2;
                        }
                        if (rdbValveCtrl2.Checked != true) // 阀控开关选择
                        {
                            chioce |= 0x4;
                        }
                        if (rdbRealTimeMeterRead2.Checked != true) // 开关实时抄表
                        {
                            chioce |= 0x8;
                        }
                        if (rdbSettlementPrice2.Checked != true) // 结算价格
                        {
                            chioce |= 0x10;
                        }
                        dataBuf[iLen] = chioce; // 几种选择的标志位
                        iLen += 1;
                        if (rdbSetChannel1.Checked == true)
                        {
                            dataBuf[iLen] = (byte)cbChannelNum.Value;
                            dataBuf[iLen] |= 0x80;
                        }
                        iLen += 1;
                        if (rdbOpenAutoChannel1.Checked == true) // 开启自动信道设置选择
                        {
                            dataBuf[iLen] = 0x80;
                        }
                        iLen += 1;
                        if (rdbValveCtrl1.Checked == true) // 阀控开关选择
                        {
                            dataBuf[iLen] = 0x1;
                        }
                        else if (rdbValveCtrl0.Checked == true)//关阀
                        {
                            dataBuf[iLen] = 0x2;
                        }
                        iLen += 1;
                        if (rdbRealTimeMeterRead1.Checked == true) // 开关实时抄表
                        {
                            dataBuf[iLen] = 0x80;
                        }
                        iLen += 1;
                        if (rdbSettlementPrice1.Checked == true) // 结算价格
                        {                        
                            float dPrice = float.Parse(tbSettlementPrice.Text); // 强制转换成double
                            int intPrice = (int)dPrice; // 获取整数部分
                            if (intPrice > 99)
                            {
                                AddStringToCommBox(false, "\n结算价格不能超过99.99元", null, Color.DarkRed, true);
                                return;
                            }
                            dataBuf[iLen + 1] = Hex2Bcd((byte)intPrice);

                            int xPrice =(int)((dPrice - intPrice + 0.001) * 100);// 获取小数部分
                            dataBuf[iLen] = Hex2Bcd((byte)xPrice);
                        }
                        iLen += 2; // 两个字节的结算价格
                        iLen += 2; // 2个保留位


                        if (iLen > iNum)
                        {
                            break;
                        }
                    }
                    if (iLen > 0)
                    {
                        Array.Resize(ref dataBuf, iLen);
                        byte[] txBuf = ConstructTxBuffer(enCmd.批量写集中器中的下发命令数据, NeedAck, null, dataBuf);
                        DataTransmit(txBuf);
                        AddStringToCommBox(false, "\n<<<-----------------批量写集中器中的下发命令数据----------------->>>", null, Color.DarkGreen, true);
                        AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                        lbCurState.Text = "设备状态：批量写集中器中的下发命令数据";
                        ProgressBarCtrl(0, 0, (CommDelayTime + 2000) / timer.Interval);
                    }
                    else
                    {
                        Command = enCmd.指令空闲;
                        lbCurState.Text = "设备状态：空闲";
                        ProgressBarCtrl(0, 0, 1000);
                       // MessageBox.Show("共写入" + iLoop.ToString("D") + "个下发命令", "写入成功");
                    }
                                       
                }
                catch (Exception ex)
                {
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    btBatchWriteMeterCmd.Text = "下发命令";
                    gpbFunList.Enabled = true;
                    cmbOperateCmd.Enabled = true;
                    cbRunNodeAddr.Enabled = true;
                    strDataFwdNodeAddrList = null;
                    strCurDataFwdAddr = "";
                    MessageBox.Show("批量写集中器中的下发命令数据出错：" + ex.Message, "错误");
                }
            }
            else
            {
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btBatchWriteMeterCmd.Text = "下发命令";
                gpbFunList.Enabled = true;
                cmbOperateCmd.Enabled = true;
                cbRunNodeAddr.Enabled = true;
                btClearNodeAddr_Click(null, null);
            }
        }
        private void btBatchWriteMeterCmd_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            if (strDataFwdNodeAddrList == null || strDataFwdNodeAddrList.Length == 0)
            {
                MessageBox.Show("任务队列为空，请输入或选择要执行任务的表具地址", "任务列表空");
                return;
            }
            if (DialogResult.Cancel == MessageBox.Show("是否将全部下发命令下发到集中器中？", "警告", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning))
            {
                return;
            }
            try
            {
                RunLssuedCmdProc();
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btBatchWriteMeterCmd.Text = "下发命令";
                cbRunCmdNodeAddr.Text = "";
                cbRunNodeAddr.Text = "";
                lbNodeAddrList.Text = "表具地址列表[0]";
                lbCmdNodeAddrList.Text = "表具地址列表[0]";
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
            }
        }
        private void ExplainBatchWriteMeterCmd(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.批量写集中器中的下发命令数据)
                {
                    int setNodeCount = rxStruct.DataBuf.Length / 7;
                    int iLen = 0;
                    string strInfo = "";
                    for (int iLoop = 0; iLoop < setNodeCount; iLoop++)
                    {
                        string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                        iLen += AddrLength;
                        string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                        DataRow[] dr = Document.Select("表具地址='" + strNodeAddr + "'");
                        if (dr.Length > 0)
                        {
                            dr[0]["结果"] = "写集中器中的下发命令：" + strResult;
                        }
                        strInfo += "\n " + strNodeAddr + "-" + strResult;
                    }
                    AddStringToCommBox(false, "\n执行结果：成功 ", null, Color.DarkBlue);
                    if (strInfo.Length > 0)
                    {
                        AddStringToCommBox(false, strInfo, null, Color.DarkBlue);
                    }

                    for (int iLoop = 0; iLoop < setNodeCount; iLoop++)
                    {
                        if (strDataFwdNodeAddrList != null && strDataFwdNodeAddrList.Length > 0 && Command != enCmd.指令空闲)
                        {
                            for (int addrLoop = 0; addrLoop < strDataFwdNodeAddrList.Length - 1; addrLoop++)
                            {
                                strDataFwdNodeAddrList[addrLoop] = strDataFwdNodeAddrList[addrLoop + 1];
                            }
                            if (strDataFwdNodeAddrList.Length > 1 ||
                                (strDataFwdNodeAddrList.Length == 1 && strDataFwdNodeAddrList[0] == strCurDataFwdAddr))
                            {
                                Array.Resize(ref strDataFwdNodeAddrList, strDataFwdNodeAddrList.Length - 1);
                            }
                        }
                    }
                    RunLssuedCmdProc();
                }
            }
            catch (Exception ex)
            {
                
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btBatchWriteMeterCmd.Text = "下发命令";
                cmbOperateCmd.Enabled = true;
                cbRunCmdNodeAddr.Enabled = true;
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
                MessageBox.Show("集中器数据转发出错：" + ex.Message);
            }
            return;
        }
        #endregion
        #region 集中器数据转发功能
        private byte[] TxByte = new byte[200];
        private byte GetCmdFromString(string strCommand)
        {
            switch (strCommand)
            {
                case "读取表具计量信息": return 0x01;
                case "表具阀门控制": return 0x03;
                case "设置表端功能使能状态": return 0x08;
                case "读取表端功能使能状态": return 0x0B;
                default: return 0xFF;
            }
        }
        public static string GetStringFromCmd(byte bCmd)
        {
            switch (bCmd)
            {
                case 0x01: return "读取表具计量信息";
                case 0x03: return "开关阀指令";
                case 0x08: return "设置表端功能使能状态";
                case 0x0B: return "读取表端功能使能状态";
                default: return "未知指令";
            }
        }
        private void cbRunNodeAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            ComboBox tbAddress = (ComboBox)sender;

            if ("0123456789\b\r\x03\x16\x18".IndexOf(e.KeyChar) < 0)
            {
                e.Handled = true;
                return;
            }
            if (e.KeyChar == '\r')
            {
                tbAddress.Text = tbAddress.Text.PadLeft(AddrLength * 2, '0');
                int iLoop = 0;
                for (; strDataFwdNodeAddrList != null && iLoop < strDataFwdNodeAddrList.Length; iLoop++)
                {
                    if (tbAddress.Text == strDataFwdNodeAddrList[iLoop])
                    {
                        break;
                    }
                }
                if (strDataFwdNodeAddrList == null)
                {
                    strDataFwdNodeAddrList = new string[1];
                    strDataFwdNodeAddrList[0] = cbRunNodeAddr.Text;
                }
                if (iLoop >= strDataFwdNodeAddrList.Length)
                { 
                    Array.Resize(ref strDataFwdNodeAddrList, strDataFwdNodeAddrList.Length + 1);
                    strDataFwdNodeAddrList[strDataFwdNodeAddrList.Length - 1] = tbAddress.Text;
                }
                e.Handled = true;
            }
            if (tbAddress.Text.Length >= AddrLength * 2 && e.KeyChar != '\b')
            {
                if (tbAddress.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        private void cbRunCmdNodeAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            ComboBox tbAddress = (ComboBox)sender;

            if ("0123456789\b\r\x03\x16\x18".IndexOf(e.KeyChar) < 0)
            {
                e.Handled = true;
                return;
            }
            if (e.KeyChar == '\r')
            {
                tbAddress.Text = tbAddress.Text.PadLeft(AddrLength * 2, '0');
                int iLoop = 0;
                for (; strDataFwdNodeAddrList != null && iLoop < strDataFwdNodeAddrList.Length; iLoop++)
                {
                    if (tbAddress.Text == strDataFwdNodeAddrList[iLoop])
                    {
                        break;
                    }
                }
                if (strDataFwdNodeAddrList == null)
                {
                    strDataFwdNodeAddrList = new string[1];
                    strDataFwdNodeAddrList[0] = cbRunCmdNodeAddr.Text;
                }
                if (iLoop >= strDataFwdNodeAddrList.Length)
                {
                    Array.Resize(ref strDataFwdNodeAddrList, strDataFwdNodeAddrList.Length + 1);
                    strDataFwdNodeAddrList[strDataFwdNodeAddrList.Length - 1] = tbAddress.Text;
                }
                e.Handled = true;
            }
            if (tbAddress.Text.Length >= AddrLength * 2 && e.KeyChar != '\b')
            {
                if (tbAddress.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
        }
        private void btClearNodeAddr_Click(object sender, EventArgs e)
        {
            strDataFwdNodeAddrList = null;
            cbRunNodeAddr.Text = "";
            cbRunCmdNodeAddr.Text = "";
            iLastDataFwdNodeCount = 0;
            ReadRealData.TimeSlot.TableNumber = 1;
            ReadRealData.TimeSlot.TotalTableNumber = 1;
        }
        private void btClearCmdNodeAddr_Click(object sender, EventArgs e)
        {
            strDataFwdNodeAddrList = null;
            cbRunNodeAddr.Text = "";
            cbRunCmdNodeAddr.Text = "";
            iLastDataFwdNodeCount = 0;
        }
        private void cbRunNodeAddr_Leave(object sender, EventArgs e)
        {
            //if (cbRunNodeAddr.Text != "")
            //{
            //    cbRunNodeAddr.Text = cbRunNodeAddr.Text.PadLeft(AddrLength * 2, '0');
            //    int iLoop = 0;
            //    for (; strDataFwdNodeAddrList != null && iLoop < strDataFwdNodeAddrList.Length; iLoop++)
            //    {
            //        if (cbRunNodeAddr.Text == strDataFwdNodeAddrList[iLoop])
            //        {
            //            break;
            //        }
            //    }
            //    if (strDataFwdNodeAddrList == null)
            //    {
            //        strDataFwdNodeAddrList = new string[1];
            //        strDataFwdNodeAddrList[0] = cbRunNodeAddr.Text;
            //    }
            //    if (iLoop >= strDataFwdNodeAddrList.Length)
            //    {
            //        Array.Resize(ref strDataFwdNodeAddrList, strDataFwdNodeAddrList.Length + 1);
            //        strDataFwdNodeAddrList[strDataFwdNodeAddrList.Length - 1] = cbRunNodeAddr.Text;
            //    }
            //}
        }

        private void cmbOperateCmd_SelectedIndexChanged(object sender, EventArgs e)
        {
            gpbFunList.Controls.Clear();
            tbDataFwdResult.Controls.Clear();
            if (cmbOperateCmd.Text == "选择要执行的命令")
            {
                tbDataFwdResult.Parent = null;
            }
            else
            {
                tbDataFwdResult.Parent = tbPageDocument;
                tbPageDocument.SelectedTab = tbDataFwdResult; 
            }
            if (cmbOperateCmd.Text == "读取表具计量信息")
            {
                ReadRealData paras = new ReadRealData();
                gpbFunList.Controls.Add(paras);
                paras.Dock = DockStyle.Fill;
                ReadRealDataResult result = new ReadRealDataResult();
                tbDataFwdResult.Controls.Add(result);
                result.Dock = DockStyle.Fill;
            }
            else if (cmbOperateCmd.Text == "表具阀门控制")
            {
                ValveControl paras = new ValveControl();
                gpbFunList.Controls.Add(paras);
                paras.Dock = DockStyle.Fill;
                ValveControlResult result = new ValveControlResult();
                tbDataFwdResult.Controls.Add(result);
                result.Dock = DockStyle.Fill;
            }
            else if (cmbOperateCmd.Text == "设置表端功能使能状态")
            {
                SetFunOption paras = new SetFunOption();
                gpbFunList.Controls.Add(paras);
                paras.Dock = DockStyle.Fill;
                SetFunOptionResult result = new SetFunOptionResult();
                tbDataFwdResult.Controls.Add(result);
                result.Dock = DockStyle.Fill;
            }
            else if (cmbOperateCmd.Text == "读取表端功能使能状态")
            {
                GetFunOption paras = new GetFunOption();
                gpbFunList.Controls.Add(paras);
                paras.Dock = DockStyle.Fill;
                GetFunOptionResult result = new GetFunOptionResult();
                tbDataFwdResult.Controls.Add(result);
                result.Dock = DockStyle.Fill;
            }
        }
        private void RunCmdProc()
        {
            if (strDataFwdNodeAddrList != null && strDataFwdNodeAddrList.Length > 0)
            {
                strCurDataFwdAddr = strDataFwdNodeAddrList[0];
                if (strCurDataFwdAddr.Length != AddrLength * 2)
                {
                    MessageBox.Show("当前表具地址中的数据不合法！", "地址错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    btRunCmd.Text = "执行命令";
                    gpbFunList.Enabled = true;
                    cmbOperateCmd.Enabled = true;
                    cbRunNodeAddr.Enabled = true;
                    strDataFwdNodeAddrList = null;
                    strCurDataFwdAddr = "";
                    return;
                }
                btRunCmd.Text = "停止命令";
                gpbFunList.Enabled = false;
                cmbOperateCmd.Enabled = false;
                cbRunNodeAddr.Enabled = false;
                try
                {
                    byte[] dataBuf = new byte[200];
                    int iLen = 0;

                    dataBuf[iLen++] = GetCmdFromString(cmbOperateCmd.Text);
                    iLen += GetByteAddrFromString(strCurDataFwdAddr, dataBuf, iLen);
                    for (int iRow = 0; iRow < dgvDoc.Rows.Count; iRow++)
                    {
                        if (strCurDataFwdAddr == dgvDoc.Rows[iRow].Cells["表具地址"].Value.ToString())
                        {
                            ReadRealData.TimeSlot.TableNumber = iRow + 1;
                            ReadRealData.TimeSlot.TotalTableNumber = dgvDoc.Rows.Count;
                            break;
                        }
                    }

                    foreach (Control ctr in gpbFunList.Controls)
                    {
                        if (ctr is IGetParas) iLen += (ctr as IGetParas).GetDataBuf(dataBuf, iLen, ReadRealData.TimeSlot.TableNumber, ReadRealData.TimeSlot.TotalTableNumber);
                    }
                    Array.Resize(ref dataBuf, iLen+2);
                    Array.Resize(ref TxByte, iLen+2);
                    Array.Copy(dataBuf, TxByte, iLen+2);
                    if (cbDefChannel.Checked == false)
                    {
                        dataBuf[iLen] = 0x0;
                        dataBuf[iLen+1] = 0x0;
                    }
                    else
                    {
                        dataBuf[iLen] = 0x44;
                        dataBuf[iLen+1] = 0x44;
                    }
                    byte[] txBuf = ConstructTxBuffer(enCmd.集中器数据转发, NeedAck, null, dataBuf);
                    DataTransmit(txBuf);
                    AddStringToCommBox(false, "\n<<<-----------------集中器数据转发：" + cmbOperateCmd.Text + "----------------->>>", null, Color.DarkGreen, true);
                    AddStringToCommBox(true, "发送：", txBuf, Color.Black);
                    lbCurState.Text = "设备状态：集中器数据转发-" + cmbOperateCmd.Text;
                    ProgressBarCtrl(0, 0, 50 * 1000 / timer.Interval);
                }
                catch (Exception ex)
                {
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                    btRunCmd.Text = "执行命令";
                    gpbFunList.Enabled = true;
                    cmbOperateCmd.Enabled = true;
                    cbRunNodeAddr.Enabled = true;
                    strDataFwdNodeAddrList = null;
                    strCurDataFwdAddr = "";
                    MessageBox.Show("集中器数据转发出错：" + ex.Message, "错误");
                }
            }
            else
            {
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btRunCmd.Text = "执行命令";
                gpbFunList.Enabled = true;
                cmbOperateCmd.Enabled = true;
                cbRunNodeAddr.Enabled = true;
                btClearNodeAddr_Click(null, null);
            }
        }
        #region 集中器数据转发命令
        private void btRunCmd_Click(object sender, EventArgs e)
        {
            if (btRunCmd.Text == "执行命令")
            {
                if (false == DeviceStatus(true, true, true))
                {
                    return;
                }
                if (cmbOperateCmd.SelectedIndex == 0)
                {
                    MessageBox.Show("请选择要执行的命令！", "命令选择", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                if (strDataFwdNodeAddrList == null || strDataFwdNodeAddrList.Length == 0)
                {
                    MessageBox.Show ("任务队列为空，请输入或选择要执行任务的表具地址", "任务列表空");
                    return;
                }
                RunCmdProc(); 
            }
            else
            {
                if (DialogResult.Cancel == MessageBox.Show("集中器需要最长为3分钟才能停止进程，请确认是否停止指令！", "停止确认", MessageBoxButtons.OKCancel, MessageBoxIcon.Question))
                {
                    return;
                }
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btRunCmd.Text = "执行命令";
                gpbFunList.Enabled = true;
                cmbOperateCmd.Enabled = true;
                cbRunNodeAddr.Enabled = true;
                cbRunCmdNodeAddr.Text = "";
                cbRunNodeAddr.Text = "";
                lbNodeAddrList.Text = "表具地址列表[0]";
                lbCmdNodeAddrList.Text = "表具地址列表[0]";
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
            }
        }
        private void ExplainRunCmd(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.集中器数据转发)
                {
                    int iLen = 0;
                    int iDataLen = rxStruct.DataBuf.Length;
                    string strCmd = GetStringFromCmd(rxStruct.DataBuf[iLen++]);
                    string strNodeAddr = GetStringAddrFromByte(rxStruct.DataBuf, iLen);
                    iLen += AddrLength;
                    string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                    AddStringToCommBox(false, "\n执行结果：" + strResult, null, Color.DarkBlue);
                    AddStringToCommBox(false, "  表具地址：" + strNodeAddr + " 命令字：" + strCmd, null, Color.DarkBlue);
                    if (0xAA == rxStruct.DataBuf[1 + AddrLength])
                    {
                        AddStringToCommBox(false, " 场强：↓-" + rxStruct.DataBuf[iDataLen - 2].ToString("D") + "dBm  ↑-" + rxStruct.DataBuf[iDataLen - 1].ToString("D") + "dBm", null, Color.DarkBlue);
                    }
                    string strResultInfo = "";
                    foreach (Control ctr in tbDataFwdResult.Controls)
                    {
                        if (ctr is IGetResult) strResultInfo = (ctr as IGetResult).GetResultString(rxStruct.DataBuf, TxByte);
                    }
                    if (strResultInfo != null)
                    {
                        AddStringToCommBox(false, "\n 表端应答：" + strResultInfo, null, Color.DarkViolet);
                    }

                    if (strDataFwdNodeAddrList != null && strDataFwdNodeAddrList.Length > 0 && Command != enCmd.指令空闲)
                    {
                        for (int iLoop = 0; iLoop < strDataFwdNodeAddrList.Length - 1; iLoop++)
                        {
                            strDataFwdNodeAddrList[iLoop] = strDataFwdNodeAddrList[iLoop + 1];
                        }
                        if (strDataFwdNodeAddrList.Length > 1 ||
                            (strDataFwdNodeAddrList.Length == 1 && strDataFwdNodeAddrList[0] == strCurDataFwdAddr))
                        {
                            Array.Resize(ref strDataFwdNodeAddrList, strDataFwdNodeAddrList.Length - 1);
                        }
                    }
                    RunCmdProc();
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                btRunCmd.Text = "执行命令";
                gpbFunList.Enabled = true;
                cmbOperateCmd.Enabled = true;
                cbRunNodeAddr.Enabled = true;
                strDataFwdNodeAddrList = null;
                strCurDataFwdAddr = "";
                MessageBox.Show("集中器数据转发出错：" + ex.Message);
            }
            return;
        }
        #endregion


        #endregion
        #region 其他公共功能函数
        public static string[] ExplainMeterData(byte[] DataBuf, int iStart, int iLength)
        {
            int index;
            string[] strResult = new string[20];

            try
            {
                for (index = 0; index < strResult.Length; index++)
                {
                    strResult[index] = "";
                }

                index = 0;
                // 实时数据
                if (iLength == 22)
                {
                    // 正转用量+反转用量
                    strResult[index++] = Byte6ToUint64(DataBuf, iStart) + "m³";
                    iStart += 6;
                    strResult[index++] = Byte6ToUint64(DataBuf, iStart) + "m³";
                    iStart += 6;
                }
                // 冻结数据(旧格式)
                else if (iLength == 89)
                {
                    // 起始序号+冻结时间+冻结方式+冻结数量+时间间隔+冻结数据
                    strResult[index++] = DataBuf[iStart++].ToString("D");
                    strResult[index++] = DataBuf[iStart++].ToString("X2") + DataBuf[iStart++].ToString("X2") + "年" +
                                         DataBuf[iStart++].ToString("X2") + "月" + DataBuf[iStart++].ToString("X2") + "日" + DataBuf[iStart++].ToString("X2") + "时";
                    strResult[index++] = (DataBuf[iStart++] & 0x80) == 0x80 ? "按月冻结" : "按日冻结";
                    strResult[index++] = DataBuf[iStart++].ToString("D");
                    if (DataBuf[iStart] == 0)
                    {
                        strResult[index] = "1条/日(月)";
                    }
                    else
                    {
                        strResult[index] = DataBuf[iStart].ToString("D") +"时/日(月)";
                    }
                    index++;
                    iStart++;
                    strResult[index] = "";
                    for (int iLoop = 0; iLoop < 10; iLoop++)
                    {
                        strResult[index] += Byte6ToUint64(DataBuf, iStart) + "m³";
                        iStart += 6;
                        strResult[index] += "/" + DataBuf[iStart++].ToString("D") + "日→";
                    }
                    strResult[index] = strResult[index].TrimEnd('→');
                    index++;
                }
                // 冻结数据(新格式)
                else if (iLength == 115)
                {
                    // 起始序号+冻结时间+冻结数据
                    strResult[index++] = DataBuf[iStart++].ToString("D");
                    strResult[index++] = DataBuf[iStart++].ToString("X2") + "月" + DataBuf[iStart++].ToString("X2") + "日" +
                                         DataBuf[iStart++].ToString("X2") + "时" + DataBuf[iStart++].ToString("X2") + "分";

                    int iBase = 0;
                    for (int iLoop = 0; iLoop < 4; iLoop++)
                    {
                        iBase <<= 8;
                        iBase += DataBuf[iStart + 4 - iLoop - 1];
                    }
                    iBase *= 1000;
                    iBase += DataBuf[iStart + 4] + DataBuf[iStart + 5] * 256;
                    iStart += 6;
                    strResult[index] = (iBase / 1000).ToString("D") + "." + (iBase % 1000).ToString("D3") + "m³→";
                    for (int iLoop = 0; iLoop < 47; iLoop++)
                    {
                        iBase += DataBuf[iStart++] + DataBuf[iStart++] * 256;
                        strResult[index] += (iBase / 1000).ToString("D") + "." + (iBase % 1000).ToString("D3") + "m³→";
                    }
                    strResult[index] = strResult[index].TrimEnd('→');
                    index++;
                }
                // 报警
                strResult[index] = "";
                if ((DataBuf[iStart] & 0x01) == 0x01)
                {
                    strResult[index] += "干簧管故障、";
                }
                if ((DataBuf[iStart] & 0x02) == 0x02)
                {
                    strResult[index] += "阀门故障、";
                }
                if ((DataBuf[iStart] & 0x04) == 0x04)
                {
                    strResult[index] += "传感器线断开、";
                }
                if ((DataBuf[iStart] & 0x08) == 0x08)
                {
                    strResult[index] += "电池欠压、";
                }
                if ((DataBuf[iStart] & 0x20) == 0x20)
                {
                    strResult[index] += "磁干扰、";
                }
                if ((DataBuf[iStart] & 0x40) == 0x40)
                {
                    strResult[index] += "光电直读表坏、";
                }
                if ((DataBuf[iStart] & 0x80) == 0x80)
                {
                    strResult[index] += "光电直读表被强光干扰、";
                }
                iStart += 1;
                if ((DataBuf[iStart] & 0x01) == 0x01)
                {
                    strResult[index] += "水表反转、";
                }
                if ((DataBuf[iStart] & 0x02) == 0x02)
                {
                    strResult[index] += "水表被拆卸、";
                }
                if ((DataBuf[iStart] & 0x04) == 0x04)
                {
                    strResult[index] += "水表被垂直安装、";
                }
                if ((DataBuf[iStart] & 0x08) == 0x08)
                {
                    strResult[index] += "Eeprom异常、";
                }
                if ((DataBuf[iStart] & 0x10) == 0x10)
                {
                    strResult[index] += "煤气泄漏、";
                }
                if ((DataBuf[iStart] & 0x20) == 0x20)
                {
                    strResult[index] += "欠费标志、";
                }
                if (strResult[index].ToString() == "")
                {
                    strResult[index] = "无报警内容";
                }
                else
                {
                    strResult[index] = strResult[index].ToString().TrimEnd('、');
                }
                iStart += 1;
                index += 1;
                // 阀状态
                if ((DataBuf[iStart] & 0x03) == 0)
                {
                    strResult[index] = "阀门故障";
                }
                else if ((DataBuf[iStart] & 0x03) == 1)
                {
                    strResult[index] = "开阀";
                }
                else if ((DataBuf[iStart] & 0x03) == 2)
                {
                    strResult[index] = "关阀";
                }
                else
                {
                    strResult[index] = "阀门未知";
                }
                iStart += 1;
                index += 1;
                // 电池电压
                if (DataBuf[iStart] >= 0xF0)
                {
                    strResult[index] = "备用电池";
                }
                else
                {
                    strResult[index] = (DataBuf[iStart] / 10).ToString("D") + "." + (DataBuf[iStart] % 10).ToString("D") + "V";
                }
                iStart += 1;
                index += 1;
                // 环境温度
                strResult[index++] = DataBuf[iStart++].ToString("D") + "℃";
                // 信噪比
                if ((DataBuf[iStart] & 0x80) == 0x80)
                {
                    strResult[index] = "-" + (DataBuf[iStart] & 0x7F).ToString("D") + "dB";
                }
                else
                {
                    strResult[index] = "+" + DataBuf[iStart].ToString("D") + "dB";
                }
                // 信道
                iStart += 1;
                index += 1;
                strResult[index++] = "Rx:" + (DataBuf[iStart] >> 4 & 0x0F).ToString("D") + "/Tx:" + (DataBuf[iStart] & 0x0F).ToString("D");
                iStart += 1;
                // 版本
                strResult[index++] = "Ver" + DataBuf[iStart++].ToString("D");
                // 下行信号强度
                strResult[index++] = "-" + DataBuf[iStart++].ToString("D") + "dBm";
                // 上行信号强度
                strResult[index++] = "-" + DataBuf[iStart++].ToString("D") + "dBm";
                Array.Resize(ref strResult, index);
                return strResult;
            }
            catch (Exception ex)
            {
                MessageBox.Show("数据解析出错，" + ex.Message);
                return null;
            }
        }
        private void tsmiClearDocument_Click(object sender, EventArgs e)
        {
            if (Document.Rows.Count > 0)
            {
                if (DialogResult.Yes == MessageBox.Show("是否要清空所有表具档案吗？", "清空确认", MessageBoxButtons.YesNo, MessageBoxIcon.Question))
                {
                    Document.Clear();
                }
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
            int item = dgvDoc.SelectedRows.Count - 1 - iLoop;
            if (null == FrmMain.strDataFwdNodeAddrList)
            {
                FrmMain.strDataFwdNodeAddrList = new string[1];
                FrmMain.strDataFwdNodeAddrList[0] = dgvDoc.SelectedRows[item].Cells[1].Value.ToString();
                iLoop = 1;
            }
            for (; iLoop < dgvDoc.SelectedRows.Count; iLoop++)
            {
                item = dgvDoc.SelectedRows.Count - 1 - iLoop;
                for (index = 0; index < FrmMain.strDataFwdNodeAddrList.Length; index++)
                {
                    if (dgvDoc.SelectedRows[item].Cells[1].Value.ToString() == FrmMain.strDataFwdNodeAddrList[index].ToString())
                    {
                        break;
                    }
                }
                if (index >= FrmMain.strDataFwdNodeAddrList.Length)
                {
                    Array.Resize(ref FrmMain.strDataFwdNodeAddrList, FrmMain.strDataFwdNodeAddrList.Length + 1);
                    FrmMain.strDataFwdNodeAddrList[FrmMain.strDataFwdNodeAddrList.Length - 1] = dgvDoc.SelectedRows[item].Cells[1].Value.ToString();
                }
            }
        }
        private void tsmiAddToCmdList_Click(object sender, EventArgs e)
        {
            int index, iLoop;
            if (dgvDoc.SelectedRows.Count <= 0)
            {
                return;
            }
            iLoop = 0;
            int item = dgvDoc.SelectedRows.Count - 1 - iLoop;
            if (null == FrmMain.strDataFwdNodeAddrList)
            {
                FrmMain.strDataFwdNodeAddrList = new string[1];
                FrmMain.strDataFwdNodeAddrList[0] = dgvDoc.SelectedRows[item].Cells[1].Value.ToString();
                iLoop = 1;
            }
            for (; iLoop < dgvDoc.SelectedRows.Count; iLoop++)
            {
                item = dgvDoc.SelectedRows.Count - 1 - iLoop;
                for (index = 0; index < FrmMain.strDataFwdNodeAddrList.Length; index++)
                {
                    if (dgvDoc.SelectedRows[item].Cells[1].Value.ToString() == FrmMain.strDataFwdNodeAddrList[index].ToString())
                    {
                        break;
                    }
                }
                if (index >= FrmMain.strDataFwdNodeAddrList.Length)
                {
                    Array.Resize(ref FrmMain.strDataFwdNodeAddrList, FrmMain.strDataFwdNodeAddrList.Length + 1);
                    FrmMain.strDataFwdNodeAddrList[FrmMain.strDataFwdNodeAddrList.Length - 1] = dgvDoc.SelectedRows[item].Cells[1].Value.ToString();
                }
            }
        }
        private void cmsMenu_Opening(object sender, CancelEventArgs e)
        {
            if (Document.Rows.Count > 0)
            {
                tsmiClearDocument.Enabled = true;
                tsmiAddToForwardList.Enabled = true;
                tsmiAddToCmdList.Enabled = true;
            }
            else
            {
                tsmiClearDocument.Enabled = false;
                tsmiAddToForwardList.Enabled = false;
                tsmiAddToCmdList.Enabled = false;
            }
        }
        private void tbCurAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
        }
        private void tbCurAddr_Leave(object sender, EventArgs e)
        {
            if (tbCurAddr.Text != "")
            {
                tbCurAddr.Text = tbCurAddr.Text.PadLeft(AddrLength * 2, '0');
            }
            dgvDoc.ClearSelection();
            foreach (DataGridViewRow row in dgvDoc.Rows)
            {
                if (row.Cells["表具地址"].Value.ToString() == tbCurAddr.Text)
                {
                    row.Selected = true;
                    break;
                }
            }
        }
        private void tbCurCmdAddr_KeyPress(object sender, KeyPressEventArgs e)
        {
            tbAddress_KeyPress(sender, e, AddrLength * 2, "0123456789\b\r\x03\x16\x18");
        }
        private void tbCurCmdAddr_Leave(object sender, EventArgs e)
        {
            if (tbCurCmdAddr.Text != "")
            {
                tbCurCmdAddr.Text = tbCurCmdAddr.Text.PadLeft(AddrLength * 2, '0');
            }
            dgvDoc.ClearSelection();
            foreach (DataGridViewRow row in dgvDoc.Rows)
            {
                if (row.Cells["表具地址"].Value.ToString() == tbCurCmdAddr.Text)
                {
                    row.Selected = true;
                    break;
                }
            }
        }
        private void tbSettlementPrice_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (tbSettlementPrice.Text.Length >= 5 && e.KeyChar != '\b')
            {
                if (tbSettlementPrice.SelectionLength == 0)
                {
                    e.Handled = true;
                }
            }
            //验证正整数,小数   
            if (e.KeyChar != 8 && e.KeyChar != 13 && e.KeyChar != 46 && !char.IsNumber(e.KeyChar))
            {
                e.Handled = true;
            }

            if (e.KeyChar == 46 && this.tbSettlementPrice.Text.Length == 0)
            {
                e.Handled = true;
                return;
            }
            int a = 0;
            try
            {
                a = this.tbSettlementPrice.Text.ToString().Trim().Split('.').Length;
            }
            catch
            {

            }
            if (e.KeyChar == 46 && a > 1)
                e.Handled = true;
            
        }



        #endregion

        #endregion
        #region 写集中器第二扫描信道
        private void btWSecondChannel_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            byte[] dataBuf = new byte[1];
            dataBuf[0] = (byte)((byte)secondChannelNum.Value&0xF);
            byte[] txBuf = ConstructTxBuffer(enCmd.写集中器第二扫描信道, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------写集中器第二扫描信道----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：写集中器第二扫描信道";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainWSecondChannel(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.写集中器第二扫描信道)
                {
                    if ( rxStruct.DataBuf[0] == 0xAA )
                    {
                        AddStringToCommBox(false, "\n执行结果：成功", null, Color.DarkBlue);
                    }
                    else
                    {
                        AddStringToCommBox(false, "集中器第二扫描信道设置失败，请重新设置！", null, Color.Blue);
                    }
                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("写集中器第二扫描信道出错：" + ex.Message);
            }
            return;
        }

        #endregion
        #region 读集中器第二扫描信道
        private void btRSecondChannel_Click(object sender, EventArgs e)
        {
            if (false == DeviceStatus(true, true, true))
            {
                return;
            }
            int iLen = 0;
            byte[] dataBuf = new byte[1+6+9];
            dataBuf[iLen++ ] = 0x82;
            for (int iLoop = 0; iLoop < AddrLength; iLoop++)
            {
                dataBuf[iLen++] = 0xD5;
            }
            dataBuf[iLen++] = 0x02;
            dataBuf[iLen++] = 0x5A;
            dataBuf[iLen++] = 0x5B;
            dataBuf[iLen++] = 0x5C;
            dataBuf[iLen++] = 0x5D;
            dataBuf[iLen++] = 0x5E;
            dataBuf[iLen++] = 0x5F;
            dataBuf[iLen++] = 0x60;
            dataBuf[iLen++] = 0x30;

            byte[] txBuf = ConstructTxBuffer(enCmd.读集中器第二扫描信道, NeedAck, null, dataBuf);
            DataTransmit(txBuf);
            AddStringToCommBox(false, "\n<<<-----------------读集中器第二扫描信道----------------->>>", null, Color.DarkGreen, true);
            AddStringToCommBox(true, "发送：", txBuf, Color.Black);
            lbCurState.Text = "设备状态：读集中器第二扫描信道";
            ProgressBarCtrl(0, 0, CommDelayTime / timer.Interval);
        }
        private void ExplainRSecondChannel(ProtocolStruct rxStruct)
        {
            try
            {
                if (rxStruct.CommandIDByte == (byte)enCmd.读集中器第二扫描信道)
                {
                    int iLen = 0;
                    int iDataLen = rxStruct.DataBuf.Length;
                    string strCmd = GetStringFromCmd(rxStruct.DataBuf[iLen++]);
                    iLen += AddrLength;
                    string strResult = GetErrorInfo(rxStruct.DataBuf[iLen++]);
                    //AddStringToCommBox(false, "\n执行结果：" + strResult, null, Color.DarkBlue);
                    if (0xAA == rxStruct.DataBuf[1 + AddrLength])
                    {
                        if ((0x02 == rxStruct.DataBuf[iLen]) && (0xAA == rxStruct.DataBuf[iLen + 1]))
                        {
                            if ((rxStruct.DataBuf[iLen + 2] & 0xF) >= 0x0 && (rxStruct.DataBuf[iLen + 2] & 0xF) <= 0xF )
                            {
                                secondChannelNum.Value = (rxStruct.DataBuf[iLen + 2] & 0xF);
                                AddStringToCommBox(false, "\n执行结果：成功  集中器第二扫描信道为：" + secondChannelNum.Text, null, Color.DarkBlue);
                            }
                            else
                            {
                                AddStringToCommBox(false, " 集中器第二扫描信道必须在 0 - 9 范围内，请重新设置！", null, Color.Blue);
                            }
                        }
                        else
                        {
                            AddStringToCommBox(false, "\n执行结果：读取集中器第二扫描信道失败", null, Color.Red);
                        }
                    }



                    Command = enCmd.指令空闲;
                    lbCurState.Text = "设备状态：空闲";
                    ProgressBarCtrl(0, 0, 1000);
                }
            }
            catch (Exception ex)
            {
                Command = enCmd.指令空闲;
                lbCurState.Text = "设备状态：空闲";
                ProgressBarCtrl(0, 0, 1000);
                MessageBox.Show("读集中器第二扫描信道出错：" + ex.Message);
            }
            return;
        }
        #endregion
    }

    interface IGetParas
    {
        int GetDataBuf(byte[] dataBuf, int iLen, int TableNumber, int TotalTableNumber);
        string GetResultString(byte[] DataBuf);
    }
    interface IGetResult
    {
        string GetResultString(byte[] DataBuf, byte[] TxBuf);
    }
}
