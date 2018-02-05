using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Windows.Forms;

namespace SR6009_Concentrator_Tools
{
    static class Program
    {
        /// <summary>
        /// 应用程序的主入口点。
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            try
            {
                //WindowHandler.GetSysConfig();
                //Util.ConcAddr = WindowHandler.GetLangValue("UNKNOW");
                Application.Run(new FrmMain());
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.StackTrace);
                Application.Exit();
            }
        }
    }
}
