#include "826api.h"
#include <ros/ros.h>
#include <thread>

#define PRINT_ERR(FUNC)  /* if ((errcode = FUNC) != S826_ERR_OK) { ROS_INFO("\nERROR: %d\n", errcode); }*/

const double SAMPLE_RATE = 1000; // Hz

enum SLOT_TIME {NONE = 0, DEFAULT = 50};

struct SLOTATTR
{
    uint chan;      // analog input channel
    uint tsettle;   // settling time in microseconds
};

const SLOTATTR slotAttrs[16] = {
    {0, DEFAULT}, {1, DEFAULT}, {2, DEFAULT}, {3, DEFAULT},
    {4, DEFAULT}, {5, DEFAULT}, {6, DEFAULT}, {7, NONE},
    {8, DEFAULT}, {9, DEFAULT}, {10, DEFAULT}, {11, DEFAULT},
    {12, DEFAULT}, {13, DEFAULT}, {14, DEFAULT}, {15, NONE}
};

class sensoray826_dev
{

    static const int ADC_MAX_SLOT = 16;

    uint board;// change this if you want to use other than board number 0
    int errcode;
    int boardflags;        // open 826 driver and find all 826 boards
    bool isOpen;

    bool isADCThreadOn;
   // boost::thread adcThread;


    uint _timeStamp[ADC_MAX_SLOT];
    int _adBuf[ADC_MAX_SLOT];

    enum AD_INDEX {LEFT_FOOT = 0, RIGHT_FOOT = 8};


public:
    // Analog Datas
    int adcDatas[ADC_MAX_SLOT];
    double adcVoltages[ADC_MAX_SLOT];
    int burstNum[ADC_MAX_SLOT];

    const double calibrationMatrixLFoot[6][6] = 
    {
    {-7.31079,   0.57154,   3.96660, -193.51164, -10.48427,  191.62950},
    {-4.75952,  219.70659,   0.37703, -110.68639,  11.48251, -110.46257},
    {235.50673,  15.29352,  235.02213,   6.13158,  241.08552,  12.23876},
    {-0.11263,   3.04924,  -7.88257,  -1.87193,   7.90115,  -1.05469},
    {9.07350,   0.69075,  -4.45980,   2.47521,  -4.43928,  -2.91897},
    {0.06164,  -4.92947,   0.06684,  -5.01570,   0.62875,  -4.91040}
    };
    const double calibrationMatrixRFoot[6][6] = 
    {
    {-4.14276,   0.47169,  18.14996, -192.32406, -12.17854,  188.45251},
    {-15.82426,  215.16146,   5.86168, -110.41180,  10.19489, -109.28200},
    {234.13572,  12.22003,  236.48927,   9.62643,  237.56447,   6.46699},
    {-0.31526,   2.96688,  -7.74985,  -1.95993,   7.82324,  -1.21225},
    {9.00393,   0.58525,  -4.66110,   2.40109,  -4.41446,  -2.77714},
    {0.45306,  -4.90875,   0.37646,  -5.12388,   0.37545,  -4.83415}
    };

    double leftFootAxisData[6];
    double rightFootAxisData[6];
    double leftFootAxisData_prev[6];
    double rightFootAxisData_prev[6];

    double leftFootBias[6];
    double rightFootBias[6];

    bool isCalibration;
    double dCalibrationTime;

    double _calibLFTData[6];
    double _calibRFTData[6];
    int _calibTimeIndex;
    int _calibMaxIndex;

public:
    sensoray826_dev(int boardno) : board(boardno), errcode(S826_ERR_OK), isOpen(false), isADCThreadOn(false), isCalibration(false), dCalibrationTime(5.0) {}
    virtual ~sensoray826_dev() { analogSampleStop(); S826_SystemClose(); }

    int open()
    {
        boardflags = S826_SystemOpen();
        if (boardflags < 0)
            {
                ROS_ERROR("BOARD ERROR"); 
                errcode = boardflags; // problem during open
                return errcode;
            }
        else if ((boardflags & (1 << board)) == 0) {
            int i;
            ROS_ERROR("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
            for (i = 0; i < 8; i++) {
                if (boardflags & (1 << i)) {
                    ROS_WARN("board %d detected. try [ %d ] board \n", i, i);
                }
            }
            return 0;
        }
        else
        {
            for (int i = 0; i < 8; i++) {
                if (boardflags & (1 << i)) {
                    ROS_INFO("board %d detected." , i);
                }
            }
            isOpen = true;
            return 1;
        }

        switch (errcode)
        {
        case S826_ERR_OK:           break;
        case S826_ERR_BOARD:        ROS_ERROR("Illegal board number"); break;
        case S826_ERR_VALUE:        ROS_ERROR("Illegal argument"); break;
        case S826_ERR_NOTREADY:     ROS_ERROR("Device not ready or timeout"); break;
        case S826_ERR_CANCELLED:    ROS_ERROR("Wait cancelled"); break;
        case S826_ERR_DRIVER:       ROS_ERROR("Driver call failed"); break;
        case S826_ERR_MISSEDTRIG:   ROS_ERROR("Missed adc trigger"); break;
        case S826_ERR_DUPADDR:      ROS_ERROR("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
        case S826_ERR_BOARDCLOSED:  ROS_ERROR("Board not open"); break;
        case S826_ERR_CREATEMUTEX:  ROS_ERROR("Can't create mutex"); break;
        case S826_ERR_MEMORYMAP:    ROS_ERROR("Can't map board"); break;
        default:                    ROS_ERROR("Unknown error"); break;
        }
    }

    /**
     * @brief analogSingleSamplePrepare, Preparing ADC but not turning on the AD thread
     * @param slotattrs ADC channel slots
     * @param count number of adc channel
     */
    void analogSingleSamplePrepare(const SLOTATTR *slotattrs , int count)
    {
        for(int i=0; i<count; i++)
        {
            PRINT_ERR(S826_AdcSlotConfigWrite(board, i, slotattrs[i].chan, slotattrs[i].tsettle, S826_ADC_GAIN_1) );
        }
        PRINT_ERR( S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)   );  // enable all timeslots
        PRINT_ERR( S826_AdcTrigModeWrite(board, 0)                       );  // select continuous (untriggered) mode
        PRINT_ERR( S826_AdcEnableWrite(board, 1)                         );  // enable conversions  
    }

    /**
     * @brief analogSampleStop Stop current AD thread and ADC
     */
    void analogSampleStop()
    {
        if(isADCThreadOn)
        {
            isADCThreadOn = false;
         //   adcThread.join();
        }
        PRINT_ERR( S826_AdcEnableWrite(board, 0)                         );  // halt adc conversions
    }
    /**
     * @brief analogOversample Do a single sample
     * @warning before calling this method, you should check whether ADC is prepared.
     */
    void analogOversample()
    {
        uint slotList = 0xFFFF;
        PRINT_ERR ( S826_AdcRead(board, _adBuf, _timeStamp, &slotList, 0));

        for(int i=0; i<ADC_MAX_SLOT; i++)
        {
            if ((((slotList >> (int)i) & 1) != 0)) {
                // extract adcdata, burstnum, and bufoverflowflag from buf
                adcDatas[i] = (int16_t)((_adBuf[i] & 0xFFFF));
                burstNum[i] = ((uint32_t)_adBuf[i] >> 24);
                adcVoltages[i] = adcDatas[i] * 10.0 / 32768;
            }
        }
        // ROS_INFO("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", adcVoltages[0], adcVoltages[1], adcVoltages[2], adcVoltages[3], adcVoltages[4], adcVoltages[5]);
    }

    double lowPassFilter(double input, double prev, double ts, double tau)
    {
        return (tau*prev + ts*input)/(tau+ts);
    }

    void initCalibration()
    {
        for(int i=0; i<6; i++)
        {
            _calibLFTData[i] = 0.0;
            _calibRFTData[i] = 0.0;
        }
        for(int i=0; i<6; i++)
        {
            leftFootBias[i] = _calibLFTData[i];
            rightFootBias[i] = _calibRFTData[i];
        }
        _calibMaxIndex = dCalibrationTime * SAMPLE_RATE;
        ROS_INFO("FT sensor calibration start... time = %.1lf sec, total %d samples ", dCalibrationTime, _calibMaxIndex);
    }

    void calibrationFTData(bool ft_calib_finish)
    {
        if(ft_calib_finish == false)
        {
            for(int i=0; i<6; i++)
            {
                double _lf = 0.0;
                double _rf = 0.0;
                for(int j=0; j<6; j++)
                {
                    _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                    _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                }
                _calibLFTData[i] += _lf / _calibMaxIndex;
                _calibRFTData[i] += _rf / _calibMaxIndex;
            }
        }
        else
        {
            for(int i=0; i<6; i++)
            {
                leftFootBias[i] = _calibLFTData[i];
                rightFootBias[i] = _calibRFTData[i];
            }
            leftFootBias[2] = leftFootBias[2]+22.81806;
            rightFootBias[2] = rightFootBias[2]+22.81806; 
        }
    }

    void computeFTData()
    {
        for(int i=0; i<6; i++)
        {
            double _lf = 0.0;
            double _rf = 0.0;
            for(int j=0; j<6; j++)
            {
                _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
            }

            _lf -= leftFootBias[i];
            _rf -= rightFootBias[i];

            leftFootAxisData[i] = lowPassFilter(_lf, leftFootAxisData_prev[i], 1.0 / SAMPLE_RATE, 0.05);
            rightFootAxisData[i] = lowPassFilter(_rf, rightFootAxisData_prev[i], 1.0/ SAMPLE_RATE,0.05);
            leftFootAxisData_prev[i] = leftFootAxisData[i];
            rightFootAxisData_prev[i] = rightFootAxisData[i];
        }
    }
};