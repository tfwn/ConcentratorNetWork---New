set addr=%~dp0
"C:\Program Files (x86)\Microsoft\ILMerge"\ilmerge.exe /ndebug /target:winexe /out:%addr%\Concentrator_Tools.exe %addr%\SR6009_Concentrator_Tools.exe %addr%\UsbLibrary.dll %addr%\Microsoft.Office.Interop.Excel.dll %addr%\Microsoft.Vbe.Interop.dll %addr%\Office.dll
pause 