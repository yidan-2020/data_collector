#ifndef _IRSDK_H
#define _IRSDK_H

#include <stdio.h>


//#ifndef _WIN32
//#define _WIN32
//#endif

#ifdef _WIN32
#define IR_SDK_API  extern "C" __declspec(dllexport)
#else
#define IR_SDK_API
#endif
#define DEVICE_COUNT			(32)							//最多支持例化32个设备的实例句柄
#define OBJ_MAX					(32)							//最多支持例化32个对象，上层只能少于32，不能超过

typedef int (*CBF_IR)(void * lData, void * lParam);				//SDK中回调函数的格式声明

//温度转换
#define CALTEMP(x,y)		((x-10000)/(float)y)				//Y16数据与温度数据的转换关系

//文件操作
#define OPEN_FILE				(1)
#define CLOSE_FILE				(2)
#define WR_FRAME 				(3)

//聚焦参数
#define  STOPFOCUS				(0)
#define  FARFOCUS				(1)
#define  NEARFOCUS				(2)
#define  AUTOFOCUS				(3)
#define  FOCUSSTATUS			(4)

//帧格式,数据前面32Byte帧头
typedef struct tagFrame
{
    unsigned short width;				//图像宽度
    unsigned short height;				//图像高度
    unsigned short u16FpaTemp;			//焦面温度
    unsigned short u16EnvTemp;			//环境温度
    unsigned char  u8TempDiv;			//数据转换成温度时需要除以该数据，文档具体说明
    unsigned char  u8DeviceType;		//unused
    unsigned char  u8SensorType;		//unused
    unsigned char  u8MeasureSel;		//unused
    unsigned char  u8Lens;				//unused
    unsigned char  u8Fps;				//unused
    unsigned char  u8TriggerFrame;		//unused
    unsigned char  Reversed[17];		//unused
    unsigned short buffer[327680];		//图像数据，按行存储，最大支持 640x512，每个像素是一个 ushort 类型
} Frame;


//点， （x,y)为坐标
typedef struct t_point
{
    unsigned short x;
    unsigned short y;
}T_POINT;

//线，P1,P2为两端点
typedef struct t_line
{
    T_POINT P1;
    T_POINT P2;
}T_LINE;

//圆（支持椭圆），Pc表示圆心坐标，a表示横半轴，b表示竖半轴，对于圆形a=b，表示半径
typedef struct t_circle
{
    T_POINT Pc;
    unsigned short a;
    unsigned short b;
}T_CIRCLE;

//矩形， P1为矩形左上顶点坐标，P2为右下顶点坐标
typedef struct t_rect
{
    T_POINT P1;
    T_POINT P2;
}T_RECT;

//任意多边形，最多支持16个顶点，Pt_num为顶点个数，Pt为每个点的坐标
typedef struct t_polygon
{
    unsigned int Pt_num;
    T_POINT Pt[16];
}T_POLYGON;

//雷达图，最多支持64个顶点，Pt_num为交点个数，Pt为每个点的坐标
typedef struct t_radar
{
    unsigned int Pt_num;
    T_POINT Pt[64];
}T_RADAR;

//最大最小平均温度及最值对应坐标
typedef struct stat_temper
{
    float maxTemper;
    float minTemper;
    float avgTemper;

    T_POINT maxTemperPT;
    T_POINT minTemperPT;
}STAT_TEMPER;

//颜色
typedef struct t_color
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
}T_COLOR;

//参数配置类型
enum T_ALARMTYPE
{
    OverHigh	= 0,			//高于高门限
    UnderLow	= 1,			//低于低门限
    BetweenHL	= 2,			//区间
    DeBetweenHL = 3,			//反选区间
};

//告警
typedef struct t_alarm
{
    unsigned char alarmType;	//告警类型
    unsigned char isDraw;		//是否屏显
    unsigned char isVioce;		//是否声音告警
    unsigned char isVideo;		//是否录像
    float		  HighThresh;	//高门限
    float		  LowThresh;	//低门限
    T_COLOR		  colorAlarm; 	//告警颜色
}T_ALARM;

//点以及点温度
typedef struct stat_point
{
    stat_point()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_POINT sPoint;
    STAT_TEMPER sTemp;
    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR	 color;						//对象颜色
    T_ALARM  sAlarm;
}STAT_POINT;

//线以及线上温度统计
typedef struct stat_line
{
    stat_line()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_LINE sLine;
    STAT_TEMPER sTemp;
    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR	 color;					//对象颜色
    T_ALARM  sAlarm;
}STAT_LINE;

//圆（椭圆)以及圆内温度统计
typedef struct stat_circle
{
    stat_circle()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_CIRCLE sCircle;
    STAT_TEMPER sTemp;
    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR		 color;					//对象颜色
    T_ALARM  sAlarm;
}STAT_CIRCLE;

//矩形以及矩形内温度统计
typedef struct stat_rect
{
    stat_rect()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_RECT sRect;
    STAT_TEMPER sTemp;
    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR	 color;					//对象颜色
    T_ALARM  sAlarm;
}STAT_RECT;

//任意多边形以及多边形内温度统计
typedef struct stat_polygon
{
    stat_polygon()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_POLYGON sPolygon;
    STAT_TEMPER sTemp;

    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR	 color;
    T_ALARM  sAlarm;
}STAT_POLYGON;

//雷达图温度统计
typedef struct stat_radar
{
    stat_radar()
    {
        inputEmiss = 1.00;
        inputReflect = 20.0;
        inputDis = 2.0;
    }
    T_RADAR sRadar;
    STAT_TEMPER sTemp[64];

    unsigned int  LableEx[32];		//对象名
    unsigned char Lable[32];		//对象名
    float		  inputEmiss;		//输入辐射率
    float		  inputReflect;		//输入反射温度
    float		  inputDis;			//输入距离
    float		  Area;				//计算面积
    unsigned char reserved1;
    unsigned char reserved2;
    unsigned char reserved3;
    unsigned char reserved4;
    T_COLOR	 color;
    T_ALARM  sAlarm;
}STAT_RADAR;

//所有对象，该结构体用来存放所有测温对象，num表示该类型对象的个数，对应的指针，需要指向测温对象，该空间需要在使用前开辟
typedef struct stat_obj
{
    unsigned char numPt;
    unsigned char numLine;
    unsigned char numCircle;
    unsigned char numRect;
    unsigned char numPolygon;
    unsigned char Reserved1;
    unsigned char Reserved2;
    unsigned char Reserved3;

    T_ALARM sGlobalAlarm;		//全局告警信息

    STAT_POINT	sPt[OBJ_MAX];
    STAT_LINE	sLine[OBJ_MAX];
    STAT_CIRCLE sCircle[OBJ_MAX];
    STAT_RECT	sRect[OBJ_MAX];
    STAT_POLYGON sPolygon[OBJ_MAX];
    STAT_RADAR Reserved[1];
}STAT_OBJ;


//128byte，存储原始视频时，该头信息会保存到视频文件中，读取时，会读出该头信息以供使用
typedef struct tagSAVEHead
{
    unsigned char  Head[32];
    unsigned short width;
    unsigned short height;
    unsigned int   totalFrames;
    unsigned short Freq;
    unsigned char  Reserved0;
    unsigned char  version;
    unsigned int   timelen;
    unsigned char  timestamp[20];
    unsigned char  Reserved[60];
} T_SAVE_HEAD;

typedef struct tagTIME {
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short hour;
    unsigned short minute;
    unsigned short sencond;
    unsigned short millisecond;
} T_TIME;


//512byte，设备信息，均以字符串形式存储，该空间已经开辟，不需要再申请空间
typedef struct tagDeviceID
{
    unsigned char  Name[32];			//设备名称
    unsigned char  Model[32];		//设备型号
    unsigned char  SerialNum[32];	//设备序列号
    unsigned char  Lens[32];			//镜头规格
    unsigned char  FactoryTime[32];	//出厂时间
    unsigned char  WorkTime[32];		//工作时间
    unsigned char  Mac[32];			//MAC地址
    unsigned char  IP[32];			//IP地址
    unsigned char  Reserved1[32];
    unsigned char  Reserved2[32];
    unsigned char  Reserved3[32];
    unsigned char  Reserved4[32];
    unsigned char  Reserved5[32];
    unsigned char  Reserved6[32];
    unsigned char  Reserved7[32];
    unsigned char  Reserved8[32];
}T_DEVICE_INFO;

typedef struct tagIPADDR
{
    char IPAddr[32];			//IP
    unsigned char Reserved[32]; //保留
    unsigned int DataPort;		//Port
    unsigned char isValid;		//是否在线
    unsigned char totalOnline;  //在线个数
    unsigned char Index;        //在列表中的索引
}T_IPADDR;

//参数配置类型
enum T_PARAMTYPE
{
    paramDevice			= 0,			//设备类型
    paramDownSample		= 1,			//降采样
    paramDecCoef		= 2,			//校正系数
    paramReserved1	    = 3,			//保留
    paramReserved2		= 4,			//保留
    paramSpaceFilter	= 5,			//空域滤波
    paramReserved4 		= 6,			//保留
    paramTempSegSel		= 7,			//温度段选择
};


#ifndef _T_CTRLPROTOCOL			//防止重复定义
#define _T_CTRLPROTOCOL
//运动控制类型
enum T_PARAMCTRL
{
    //协议选择
    paramPelcod 		= 0,		   //pelco-d
    paramUserDef1		= 1,		   //自定义协议1（升降杆）
    paramUserDef2		= 2,		   //自定义协议2（舵机）
    paramUserDef3		= 3,		   //自定义协议3（转台）

    //控制
    paramCtrlUp			= 4,			//上
    paramCtrlDown		= 5,			//下
    paramCtrlLeft		= 6,			//左
    paramCtrlRight		= 7,			//右
    paramCtrlStop		= 8,			//停止
    paramCtrlBaudRate	= 9,			//波特率
};
#endif

IR_SDK_API int IRSDK_Init(void);
IR_SDK_API int IRSDK_Quit(void);
IR_SDK_API int IRSDK_Create(int handle, T_IPADDR sIPAddr, CBF_IR cbf_stm, CBF_IR cbf_cmd, CBF_IR cbf_comm, void * param = 0);
IR_SDK_API int IRSDK_Destroy(int handle);
IR_SDK_API int IRSDK_Connect(int handle);
IR_SDK_API int IRSDK_Play(int handle);
IR_SDK_API int IRSDK_Stop(int handle);
IR_SDK_API int IRSDK_SetIP(int handle, char * pIp);
IR_SDK_API int IRSDK_Command(int handle, int command, int param);
IR_SDK_API int IRSDK_Calibration(int handle);
IR_SDK_API int IRSDK_CommSend(int handle, char *pBuf, int len);
IR_SDK_API int IRSDK_IsConnected(int handle);
IR_SDK_API int IRSDK_NearFarFocus(int handle, unsigned int param);
IR_SDK_API int IRSDK_InqureIP(void * pIpInfo, unsigned int TimerInterval);
IR_SDK_API int IRSDK_InqureDeviceInfo(int handle, T_DEVICE_INFO* pDevInfo);

IR_SDK_API int IRSDK_ParamCfg(int handle, T_PARAMTYPE mParamType, float f32Param);

IR_SDK_API int IRSDK_FrameConvert(Frame *pFrame, unsigned short *pGray, float f32Constrast, float f32Bright, unsigned int *pGethist, STAT_TEMPER *pFull_temper, unsigned short u16TFilterCoef);

IR_SDK_API int IRSDK_Gray2Rgb(unsigned short* pGray, unsigned char* pRgb, unsigned short Width, unsigned short Height, int PalType, int Pal);

IR_SDK_API int IRSDK_GetPaletteJpeg(unsigned char* pPaletteJpeg, unsigned int *pJpegLen, unsigned char Method, int PalType, int Pal);
IR_SDK_API int IRSDK_GetPaletteBmp(unsigned char* pPaletteBmp, unsigned int *pBmpLen, unsigned char Method, int PalType, int Pal);

IR_SDK_API int IRSDK_GetPointTemp(Frame *pFrame, STAT_POINT *pPointStat,unsigned char index);
IR_SDK_API int IRSDK_GetLineTemp(Frame *pFrame, STAT_LINE *pLineStat, unsigned char index);
IR_SDK_API int IRSDK_GetCircleTemp(Frame *pFrame, STAT_CIRCLE *pCircleStat, unsigned char index);
IR_SDK_API int IRSDK_GetRectTemp(Frame *pFrame, STAT_RECT *pRectStat, unsigned char index);
IR_SDK_API int IRSDK_GetPolygonTemp(Frame *pFrame, STAT_POLYGON *pPolygonStat, unsigned char index);
IR_SDK_API int IRSDK_GetObjTemp(Frame *pFrame, STAT_OBJ *pObjStat);
IR_SDK_API int IRSDK_DrawObj(Frame *pFrame, unsigned char *pRgb, unsigned short width, unsigned short height, STAT_OBJ *pObjStat);

IR_SDK_API int IRSDK_Rgb2Bmp(unsigned char * pBmpData, unsigned int *pLen, unsigned char* pRgb, unsigned short Width, unsigned short Height);
IR_SDK_API int IRSDK_Rgb2Jpeg(unsigned char * pJpegout, unsigned int *pLen, int quality, unsigned char * pRgb, unsigned short Width, unsigned short Height);

IR_SDK_API int IRSDK_SaveFrame2Jpeg(char* pFile, Frame *pFrame, unsigned char* pRgb, STAT_OBJ *pObj);
IR_SDK_API int IRSDK_ReadJpeg2Frame(char *pFile, Frame *pFrame, unsigned char isLoadObj, STAT_OBJ *pObj);

IR_SDK_API int IRSDK_SaveFrame2Video(char *pFile, Frame *pFrame, unsigned char Op, STAT_OBJ *pObj, unsigned char * pThreadBuf);
IR_SDK_API int IRSDK_ReadVideo2Frame(char *pFile, Frame *pFrame, unsigned int Index, unsigned char Op, T_SAVE_HEAD *pVideoHead, STAT_OBJ *pObj, unsigned char * pThreadBuf);

IR_SDK_API int IRSDK_SaveRgb2AVI(char* pFile, unsigned char *pRgb, unsigned short Width, unsigned short Height, unsigned char Op, int quality, unsigned char * pThreadBuf);

IR_SDK_API int IRSDK_SaveObj2CSV(char *pFile, unsigned char Op, STAT_OBJ *pObj, STAT_TEMPER *pGlobalTemper, unsigned char * pThreadBuf);
IR_SDK_API int IRSDK_SaveFrame2CSV(char* pFile, Frame *pFrame);
IR_SDK_API int IRSDK_SaveLine2CSV(char* pFile, Frame *pFrame, T_LINE sLine, unsigned char u8Format);
IR_SDK_API int IRSDK_SaveRect2CSV(char* pFile, Frame *pFrame, T_RECT sRect);
IR_SDK_API int IRSDK_SaveCircle2CSV(char* pFile, Frame *pFrame, T_CIRCLE sCircle);
IR_SDK_API int IRSDK_SavePolygon2CSV(char* pFile, Frame *pFrame, T_POLYGON sPolygon);

IR_SDK_API int IRSDK_MoveCtrl(int handle, T_PARAMCTRL mProtocol, T_PARAMCTRL mType, unsigned int u32Param);

#endif

