#ifndef _OPTOFORCECAN_H
#define _OPTOFORCECAN_H

#include "stdio.h"
#include <cstdint>
#include <canlib.h>

#define MAX_CHANNELS 63
#define FORCE_DIV	10.0
#define TORQUE_DIV	1000.0
#define canIOCTL_GET_EVENTHANDLE 14
#define ALARM_INTERVAL_IN_S     (1)
#define PRINTF_ERR(x)     printf x


typedef unsigned long DWORD;
typedef void* HANDLE;

typedef struct {
int        channel;
char       name[100];
DWORD      hwType;
canHandle  hnd;
int        hwIndex;
int        hwChannel;
int        isOnBus;
int        driverMode;
int        txAck;
} ChannelDataStruct;

typedef struct {
unsigned int       channelCount;
ChannelDataStruct  channel[MAX_CHANNELS];
} driverData;

driverData     m_channelData;
driverData    *m_DriverConfig = &m_channelData;


class optoforcecan
{
public:
	optoforcecan(){}
	virtual ~optoforcecan(){}

	///CAN Variable
	int   m_usedBaudRate = 1000000;
	canHandle      m_usedChannel = 0;
	unsigned int   m_usedId = 0x101;
	unsigned int   m_usedFlags = 0;
	unsigned int   m_Verbose = 0;

	HANDLE        th[MAX_CHANNELS + 1];
	DWORD         active_handle;
	char          c;
	canStatus     stat;
	bool canCheck;

	double Fx[3], Fy[3], Fz[3], Mx[3], My[3], Mz[3];
	unsigned int samplecounter[3];
	unsigned char change_hz=10;
	bool offset = true;
	int sensorid;
	int16_t Fdata[3][8];

	int framecounter[3];
	int cntforcount[3];
	int cali_count;

	double SAMPLERATE;
	double dCalibrationTime;
	double _calibMaxIndex;
	double leftArmBias[6];
	double rightArmBias[6];
	double _calibRATData[6];
	double _calibLATData[6];
	double rightArmAxisData[6];
	double leftArmAxisData[6];
	double leftArmAxisData_prev[6];
	double rightArmAxisData_prev[6];

    double lowPassFilter(double input, double prev, double ts, double tau)
    {
        return (tau*prev + ts*input)/(tau+ts);
    }

	////// Set Bus parameters/////
	void InitDriver(void)
	{
		int  i;
		canStatus  stat;

		// Initialize ChannelData.
		memset(m_channelData.channel, 0, sizeof(m_channelData.channel));
		for (i = 0; i < MAX_CHANNELS; i++) 
		{
			m_channelData.channel[i].isOnBus      = 0;
			m_channelData.channel[i].driverMode   = canDRIVER_NORMAL;
			m_channelData.channel[i].channel      = -1;
			m_channelData.channel[i].hnd          = canINVALID_HANDLE;
			m_channelData.channel[i].txAck        = 0; // Default is TxAck off
		}
		m_channelData.channelCount = 0;

		//
		// Enumerate all installed channels in the system and obtain their names
		// and hardware types.
		//

		//initialize CANlib
		canInitializeLibrary();
		SAMPLERATE = 500;
		cali_count = 0.0;
		dCalibrationTime = 4.0;
		_calibMaxIndex = dCalibrationTime * SAMPLERATE;

		for(int i = 0; i < 6; i++)
		{
			leftArmBias[i] = 0.0;
			rightArmBias[i] = 0.0;
			_calibRATData[i] = 0.0;
			_calibLATData[i] = 0.0;
			leftArmAxisData_prev[i] = 0.0;
			rightArmAxisData_prev[i] = 0.0;
		} 
		
		//get number of present channels
		
		stat = canGetNumberOfChannels((int*)&m_channelData.channelCount);
			printf("ssss2");
		for (i = 0; (unsigned int)i < m_channelData.channelCount; i++) 
		{
			canHandle  hnd;
			//obtain some hardware info from CANlib
			m_channelData.channel[i].channel = i;
			canGetChannelData(i, canCHANNELDATA_CHANNEL_NAME,
							m_channelData.channel[i].name,
							sizeof(m_channelData.channel[i].name));
			canGetChannelData(i, canCHANNELDATA_CARD_TYPE,
							&m_channelData.channel[i].hwType,
							sizeof(DWORD));

		printf("34443");
			//open CAN channel
			hnd = canOpenChannel(i, canOPEN_ACCEPT_VIRTUAL);
			if (hnd < 0) 
			{
		printf("3333");
				// error
				PRINTF_ERR(("ERROR canOpenChannel() in initDriver() FAILED Err= %d. <line: %d>\n",
							hnd, __LINE__));
			}
			else 
			{
		printf("4444");
				m_channelData.channel[i].hnd = hnd;
				if ((stat = canIoCtl(hnd, canIOCTL_FLUSH_TX_BUFFER, NULL, (unsigned int)NULL)) != canOK)
					PRINTF_ERR(("ERROR canIoCtl(canIOCTL_FLUSH_TX_BUFFER) FAILED, Err= %d <line: %d>\n",
								stat, __LINE__));
			}

			//set up the bus
			if (i == 0) {
			switch(m_usedBaudRate) 
			{
				case 1000000:
				m_usedBaudRate = canBITRATE_1M;
				break;
				case 500000:
				m_usedBaudRate = canBITRATE_500K;
				break;
				case 250000:
				m_usedBaudRate = canBITRATE_250K;
				break;
				case 125000:
				m_usedBaudRate = canBITRATE_125K;
				break;
				case 100000:
				m_usedBaudRate = canBITRATE_100K;
				break;
				case 62500:
				m_usedBaudRate = canBITRATE_62K;
				break;
				case 50000:
				m_usedBaudRate = canBITRATE_50K;
				break;
				default:
				printf("Baudrate set to 125 kbit/s. \n");
				m_usedBaudRate = canBITRATE_125K;
				break;
			}
			}
			
			//set the channels busparameters
			stat = canSetBusParams(hnd, m_usedBaudRate, 0, 0, 0, 0, 0);
			if (stat < 0) 
			{
				PRINTF_ERR(("ERROR canSetBusParams() in InitDriver(). Err = %d <line: %d>\n",
							stat, __LINE__));
			}

			for (i = 1; i < (m_channelData.channelCount + 1); i++) 
			{
				HANDLE tmp;
				//go on bus (every channel)
				stat = canBusOn(m_channelData.channel[i-1].hnd);
				if (stat < 0) 
				{
					PRINTF_ERR(("ERROR canBusOn(). Err = %d <line: %d>\n", stat, __LINE__));
				}
				else 
				{
					m_DriverConfig->channel[i-1].isOnBus = 1;
				}
			}
		}
	}

	void DAQSensorData()
	{
		alarm(ALARM_INTERVAL_IN_S);
  
		unsigned int    i;
		unsigned int    j;
		long            id;
		unsigned char   data[8];
		unsigned int    dlc;
		unsigned int    flags;
		DWORD           time;
		int             moreDataExist;
		
		sensorid = 0;
		
		do {
			moreDataExist = 0;
			for (i = 0; i < m_channelData.channelCount; i++) 
			{
				stat = canRead(m_channelData.channel[i].hnd, &id, &data[0], &dlc, &flags, &time);
				if(canCheck == false)
				{
					if(stat == canOK)
					{
						//printf("channel %i OK,  %i\n", m_channelData.channel[i].hnd, stat);
					}
					else
					{
						//printf("channel %i Err", m_channelData.channel[i].hnd, stat);  
					}   
				}
			}

			canCheck = true;	

			switch (stat) 
			{
				case canOK:

				if ((flags & canMSG_RTR) == 0) 
				{ 
					if (id==0x111) 
					{
						sensorid = 1;
					}
					else if (id == 0x100) 
					{
						sensorid = 0;
					}
				else 
				{
					sensorid = 2;
				}

				if (data[0] == 170 && data[1] == 7) 
				{
					samplecounter[sensorid] = (((unsigned int)data[4]) << 8) + ((unsigned int)data[5]);
					framecounter[sensorid] = 1;
				}
				else if (framecounter[sensorid] == 1) 
				{
					Fdata[sensorid][0] = (((unsigned int)data[0])<<8)+ ((unsigned int)data[1]);
					Fdata[sensorid][1] = (((unsigned int)data[2]) << 8) + ((unsigned int)data[3]);
					Fdata[sensorid][2] = (((unsigned int)data[4]) << 8) + ((unsigned int)data[5]);
					Fdata[sensorid][3] = (((unsigned int)data[6]) << 8) + ((unsigned int)data[7]);
					Fdata[sensorid][4] = (((unsigned int)data[0]) << 8) + ((unsigned int)data[1]);
					Fdata[sensorid][5] = (((unsigned int)data[2]) << 8) + ((unsigned int)data[3]);
					
					Fdata[sensorid][0] = Fdata[sensorid][0]/FORCE_DIV;
					Fdata[sensorid][1] = Fdata[sensorid][1]/FORCE_DIV;
					Fdata[sensorid][2] = Fdata[sensorid][2]/FORCE_DIV;
					Fdata[sensorid][3] = Fdata[sensorid][3]/TORQUE_DIV;
					Fdata[sensorid][4] = Fdata[sensorid][4]/TORQUE_DIV;
					Fdata[sensorid][5] = Fdata[sensorid][5]/TORQUE_DIV;

					framecounter[sensorid] = 0;
				}
			}
				moreDataExist = 1;
				break;

				case canERR_NOMSG:
				// No more data on this handle
				break;

				default:
				PRINTF_ERR(("ERROR canRead() FAILED, Err= %d <line: %d>\n", stat, __LINE__));
				break;
			}
		} while (moreDataExist);
	}

	void calibrationFTData(bool ft_calib_finish)
	{	
		if(ft_calib_finish == false)
        {
			if(cali_count >= 500)
			{
				double _lf, _rf;
				for(int i=0; i<6; i++)
				{
					_lf = 0.0;
					_rf = 0.0;
					_lf = (float)Fdata[0][i];
					_rf = (float)Fdata[1][i];			
					_calibLATData[i] += _lf / _calibMaxIndex;
					_calibRATData[i] += _rf / _calibMaxIndex;
				}
			}
		}
        else
        {
            for(int i=0; i<6; i++)
            {
                leftArmBias[i] = _calibLATData[i];
                rightArmBias[i] = _calibRATData[i];
            }
        }
		cali_count++;
	}

	void computeFTData(bool ft_calib_finish)
	{
		if(ft_calib_finish == false)
		{
			for(int i=0; i<6; i++)
			{
				double _lf = 0.0;
				double _rf = 0.0;
				_lf = (float)Fdata[0][i];
				_rf = (float)Fdata[1][i];
    			leftArmBias[i] = _lf;
				rightArmBias[i] = _rf;
			}
		}
		else
		{
			for(int i=0; i<6; i++)
			{
				double _lf = 0.0;
				double _rf = 0.0;
				_lf = (float)Fdata[0][i];
				_rf = (float)Fdata[1][i];
				_lf -= leftArmBias[i];
				_rf -= rightArmBias[i];

				leftArmAxisData[i] = lowPassFilter(_lf, leftArmAxisData_prev[i], 1.0 / SAMPLERATE, 0.05);
				rightArmAxisData[i] = lowPassFilter(_rf, rightArmAxisData_prev[i], 1.0/ SAMPLERATE,0.05);
			
				leftArmAxisData_prev[i] = leftArmAxisData[i];
				rightArmAxisData_prev[i] = rightArmAxisData[i];
			}
		}
	}
};

#endif
