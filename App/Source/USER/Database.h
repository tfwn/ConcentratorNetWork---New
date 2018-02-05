/************************************************************************************************
*                                   SRWF-6009
*    (c) Copyright 2015, Software Department, Sunray Technology Co.Ltd
*                               All Rights Reserved
*
* FileName     : Database.h
* Description  :
* Version      :
* Function List:
*------------------------------Revision History--------------------------------------------------
* No.   Version     Date            Revised By      Item            Description
* 1     V1.1        08/11/2015      Zhangxp         SRWF-6009       Original Version
************************************************************************************************/

#ifndef  DATABASE_H
#define  DATABASE_H

#ifdef   DATABASE_GLOBALS
#define  DATABASE_EXT
#else
#define  DATABASE_EXT  extern
#endif

/************************************************************************************************
*                               Pubilc Macro Define Section
************************************************************************************************/
#define NULL_U32_ID                             0xFFFFFFFF  // 32位无效数据
#define NULL_U16_ID                             0xFFFF      // 16位无效数据
#define NULL_U12_ID                             0xFFF       // 12位无效数据
#define NULL_U10_ID                             0x3FF       // 10位无效数据
#define NULL_U8_ID                              0xFF        // 8位无效数据
#define NULL_U4_ID                              0xF         // 4位无效数据

#define MAX_NODE_NUM                            1024        // 节点档案总数
#define MAX_NEIGHBOUR_NUM                       3           // 最大邻居数
#define MAX_CUSTOM_ROUTE_LEVEL                  5           // 自定义路径中最大的级数
#define MAX_CUSTOM_ROUTES                       2           // 每个节点最多可定义的路径数

#define DATA_CENTER_ID                          2048        // 中心节点ID编号
#define DATA_SAVE_DELAY_TIMER                   3           // 数据保存延时时间(秒)

#define UPDOWN_RSSI_SIZE                        2           // 上下行场强域大小
#define REALTIME_DATA_AREA_SIZE                 27          // 定时定量数据总数(含5个字节的时间和上下行场强)
#define FREEZE_DATA_AREA_SIZE                   115         // 冻结数据总数(含上下行场强)
#define NODE_INFO_SIZE                          150		//128         // 每个节点信息占据的存储区大小

/************************************************************************************************
*                                   Enum Define Section
************************************************************************************************/
// 集中器工作类型定义
typedef enum {
    RealTimeDataMode = 0,                                   // 定时定量工作模式
    FreezeDataMode,                                         // 冻结数据工作模式
} WORK_TYPE;

// 设备类型
typedef enum {
    Dev_AllMeter = 0x00,                                    // 全部表类型
    Dev_WaterMeter = 0x10,                                  // 水表
    Dev_GprsWaterMeter = 0x11,                              // Gprs水表
    Dev_HotWaterMeter = 0x20,                               // 热水表
    Dev_GasMeter = 0x30,                                    // 气表
    Dev_GprsGasMeter = 0x31,                                // Gprs气表
    Dev_AmMeter = 0x40,                                     // 电表

    Dev_USB = 0xF9,                                         // USB端口
    Dev_Server = 0xFA,                                      // 服务器
    Dev_SerialPort = 0xFB,                                  // PC机串口
    Dev_Concentrator = 0xFC,                                // 集中器
    Dev_CRouter = 0xFD,                                     // 采集器或中继器
    Dev_Handset = 0xFE,                                     // 手持机

    Dev_Empty = 0xFF                                        // 空类型
} DEVICE_TYPE;

/************************************************************************************************
*                                   Union Define Section
************************************************************************************************/

/************************************************************************************************
*                                  Struct Define Section
************************************************************************************************/
// GPRS模块参数
typedef struct {
    uint8 PriorDscIp[4];                                    // 首选服务器的IP地址
    uint16 PriorDscPort;                                    // 首选服务器的端口号
    uint8 BackupDscIp[4];                                   // 备用服务器的IP地址
    uint16 BackupDscPort;                                   // 备用服务器的端口号
    char Apn[12 + 1];                                       // 连接的APN,最后一个为0
    char Username[12 + 1];                                  // 连接APN的用户名,最后一个为0
    char Password[12 + 1];                                  // 连接APN的密码,最后一个为0
    uint8 HeatBeatPeriod;                                   // 心跳包间隔,单位为10秒
} GPRS_PARAMETER;

// 集中器工作参数
typedef struct {
    WORK_TYPE WorkType;                                     // 集中器工作类型
    uint8 DataReplenishCtrl: 1;                             // 数据补抄控制位:1为打开,0为关闭
    uint8 DataUploadCtrl: 1;                                // 数据上传控制位:1为打开,0为关闭
    uint8 DataUploadMode: 1;                                // 数据上传类型控制位:1为冻结数据,0为定时定量数据
    uint8 DataEncryptCtrl: 1;                               // 数据加密控制:1为加密,0为不加密
    uint8 DataUploadTime;                                   // 数据上传时间点:BCD格式
    uint8 DataReplenishDay[4];                              // 数据补抄的日期
    uint8 DataReplenishHour;                                // 数据补抄的时间:BCD格式
    uint8 DataReplenishCount;                               // 数据补抄失败时重复补抄的次数
} WORK_PARAM_STRUCT;

// 自定义路径信息(大小必须为偶数)
typedef struct {
    uint16 AddrCrc16;                                       // 节点长地址的CRC值
    uint16 RouteNode[MAX_CUSTOM_ROUTES][MAX_CUSTOM_ROUTE_LEVEL];    // 中继节点
} CUSTOM_ROUTE_INFO;

// 节点属性定义
typedef struct {
    uint8 LastResult: 2;                                    // 最后一次抄表结果:0-失败,1-成功,其他-未知
    uint8 CurRouteNo: 2;                                    // 当前路径号,当此值大于CUST_ROUTES_PART的时候使用的是自定义路径
    uint8 UploadData: 1;                                    // 节点上传了数据
    uint8 UploadPrio: 1;                                    // 此节点的上传优先级高
} SUBNODE_PROPERETY;


typedef union{
	uint8 Content;
	struct{
		uint8 SetChannel:1;				// bit0 设置信道
		uint8 AutoChannel:1;			// bit1 开关自动信道设置
		uint8 ValveCtrl:1;				// bit3 阀控
		uint8 RealTimeMeterReading:1;   // bit4 开关实时抄表功能
		uint8 SettlementPrice:1; 		// bit5 结算价格
		uint8 Reserve:3;				// 保留
	};
}CMD_PROPERTY;

typedef struct{
	uint8 SetChannel; 				// 设置信道
	uint8 AutoChannel;				// 开启自动信道设置
	uint8 ValveCtrl;				// 阀控
	uint8 RealTimeMeterReading;		// 开关实时抄表功能
	uint8 SettlementPriceLow;			// 结算价格(低位)
	uint8 SettlementPriceHigh;			// 结算价格(高位)
	uint8 Reserve[2];				// 保留
}CMD_DATA;


// 节点信息定义(大小必须为偶数)
typedef struct {
    uint8 LongAddr[LONG_ADDR_SIZE];                         // 节点长地址(保存在Flash中)
    DEVICE_TYPE DevType;                                    // 设备类型(保存在Flash中)
    SUBNODE_PROPERETY Property;                             // 设备属性(保存在Eeprom中)
    uint8 RxMeterDataDay;                                   // 接收到表具数据的日期
	uint8 RxMeterDataTemp[4];								// 接收到表具数据的具体时间信息
	CMD_PROPERTY CmdProp;									// 集中器响应ACK时的任务标志
	CMD_DATA CmdData;										// 集中器响应ACK时的任务内容
    uint8 RxChannel;                                        // 表端接收使用的信道
    uint8 TxChannel;										// 表端发射使用的信道
    uint8 RxMeterVersion;									// 表端版本号
    uint8 AutoChannelSwitch;								// 表端自动信道开关
    //uint8 xxxx_MustEven;									// 保留位，凑偶数
} SUBNODE_INFO;

// 集中器基本信息定义(大小必须为偶数)
typedef struct {
    uint16 Fcs;                                             // Fcs为集中器地址的校验值,用于验证集中器信息是否正确
    uint8 LongAddr[LONG_ADDR_SIZE];                         // 集中器的长地址,Bcd码
    uint16 MaxNodeId;                                       // 最大节点的数量,即是保存节点的存储的最大位置
    uint8 CustomRouteSaveRegion: 1;                         // 自定义路径保存的区域
    WORK_PARAM_STRUCT Param;                                // 工作的参数
    GPRS_PARAMETER GprsParam;                               // GPRS模块参数
    uint8 SecondChannel;									// 第二扫描信道
    uint8 SaveSecondChannel;								// 保存第二扫描信道号
} CONCENTRATOR_INFO;

// 表数据保存格式定义
typedef struct {
    uint8 Address[LONG_ADDR_SIZE];                          // 节点长地址
    SUBNODE_PROPERETY Property;                             // 节点属性
    uint8 RxMeterDataDay;                                   // 接收到表具数据的日
	uint8 RxMeterDataTemp[4]; 								// 接收到表具数据的具体时间信息
    uint8 RxMeterVersion;									// 表端版本号
	CMD_PROPERTY CmdProp;									// 集中器响应ACK时的任务标志
	CMD_DATA CmdData;										// 集中器响应ACK时的任务内容
    uint8 RxChannel;                                        // 表端接收信道
    uint8 TxChannel;                                        // 表端发射信道
    uint8 AutoChannelSwitch;								// 表端自动信道开关
    uint8 Crc8MeterData;                                    // 表数据的校验值
    uint8 MeterData[1];                                     // 表数据区(该区最后两个字节为下行和上行信号强度)
} METER_DATA_SAVE_FORMAT;

/************************************************************************************************
*                        Global Variable Declare Section
************************************************************************************************/
DATABASE_EXT CONCENTRATOR_INFO Concentrator;
DATABASE_EXT SUBNODE_INFO SubNodes[MAX_NODE_NUM];

/************************************************************************************************
*                            Function Declare Section
************************************************************************************************/
DATABASE_EXT void Data_Init(void);
DATABASE_EXT void Data_RefreshNodeStatus(void);
DATABASE_EXT void Data_RdWrConcentratorParam(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_MeterDataInit(METER_DATA_SAVE_FORMAT *MeterBufPtr, uint16 NodeId, uint8 MeterDataLen);
DATABASE_EXT void Data_SetConcentratorAddr(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT uint16 Data_FindNodeId(uint16 StartId, uint8 *BufPtr);
DATABASE_EXT bool Data_ClearMeterData(void);
DATABASE_EXT void Data_ClearDatabase(void);
DATABASE_EXT void Data_ReadNodesCount(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_ReadNodes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_WriteNodes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_DeleteNodes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_ModifyNodes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_GetTimeSlot(uint8 *NodeIdPtr, uint8 *TimeSlotPtr);
DATABASE_EXT uint8 Data_GetRoute(uint16 NodeId, uint8 *BufPtr);
DATABASE_EXT void Data_ReadCustomRoute(DATA_FRAME_STRUCT *DataBufPtr);
DATABASE_EXT void Data_WriteCustomRoute(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_BatchReadCustomRoutes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_BatchWriteCustomRoutes(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_GprsParameter(DATA_FRAME_STRUCT *DataFramePtr);
DATABASE_EXT void Data_SwUpdate(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_EepromCheckProc(DATA_FRAME_STRUCT *DataFrmPtr);
DATABASE_EXT void Data_BatchWriteMeterCmdLoadProc(DATA_FRAME_STRUCT *DataFrmPtr);

#endif
/***************************************End of file*********************************************/


