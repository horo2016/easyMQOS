#ifndef SERIAL_H
#define  SERIAL_H



//�������鳤��
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
	char isGetData;		//�Ƿ��ȡ��GPS����
	char isParseData;	//�Ƿ�������
	char UTCTime[UTCTime_Length];		//UTCʱ��
	char latitude[latitude_Length];		//γ��
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//����
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//GPS��Ϣ�Ƿ�����
	char  earthSpeed[Speed_Length];//�ر��ٶ�
	char  earthHeading[Heading_Length];//�����
	char UTCDate[UTCTime_Length];		//UTC����
	char MagneticDeclination[Mag_Length];//�شŽ�
} _SaveData;
extern _SaveData Save_Data;
#endif 
