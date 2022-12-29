#ifndef SERIAL_H
#define  SERIAL_H



//定义数组长度
#define GPS_Buffer_Length 80
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 
#define Speed_Length 7 
#define Heading_Length 6 
#define Mag_Length 7

typedef struct SaveData 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//GPS信息是否有用
	char  earthSpeed[Speed_Length];//地表速度
	char  earthHeading[Heading_Length];//航向角
	char UTCDate[UTCTime_Length];		//UTC日期
	char MagneticDeclination[Mag_Length];//地磁角
} _SaveData;
extern _SaveData Save_Data;
#endif 
