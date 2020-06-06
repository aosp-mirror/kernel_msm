/*
 *
 * Xun.Ouyang add for oppo project
 *
 *
 */
#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

enum{
    HW_VERSION__UNKNOWN           = 0,

    /*For common project definition*/
    HW_VERSION__10,
    HW_VERSION__11,
    HW_VERSION__12,
    HW_VERSION__13,
    HW_VERSION__14,
    HW_VERSION__15,
    HW_VERSION__16,

};


enum{
    RF_VERSION__UNKNOWN           = 0,

    /*For common project definition*/
    RF_VERSION__11,
    RF_VERSION__12,
    RF_VERSION__13,
    RF_VERSION__21,
    RF_VERSION__22,
    RF_VERSION__23,
    RF_VERSION__31,
    RF_VERSION__32,
    RF_VERSION__33,

};


#define GET_PCB_VERSION() (get_PCB_Version())
#define GET_PCB_VERSION_STRING() (get_PCB_Version_String())

#define GET_MODEM_VERSION() (get_Modem_Version())
#define GET_OPERATOR_VERSION() (get_Operator_Version())



enum OPPO_PROJECT {
    OPPO_UNKOWN = 0,
    OPPO_19001 = 19001,
    OPPO_19901 = 19901,
    OPPO_19903 = 19903,
    OPPO_19905 = 19905,
    OPPO_19907 = 19907,
};

enum OPPO_OPERATOR {
    OPERATOR_UNKOWN                               = 0,

    /*For common project definition*/
    OPERATOR_OPEN_MARKET                           = 1,
    OPERATOR_CHINA_MOBILE                           = 2,
    OPERATOR_CHINA_UNICOM                           = 3,
    OPERATOR_CHINA_TELECOM                           = 4,
    OPERATOR_FOREIGN                               = 5,
    OPERATOR_FOREIGN_WCDMA                           = 6,
    OPERATOR_FOREIGN_RESERVED                     = 7,
    OPERATOR_ALL_CHINA_CARRIER                      = 8,    //instead of TELECOM CARRIER because of history Tong.han@Bsp.Group.Tp add for all china carrier phone, 2015/03/23
    OPERATOR_ALL_CHINA_CARRIER_MOBILE              = 9,    //rendong.shi@Bsp.Group.Tp add for all china carrier MOBILE phone, 2016/01/07
    OPERATOR_ALL_CHINA_CARRIER_UNICOM             = 10,   //rendong.shi@Bsp.Group.Tp add for all china carrier UNICOM  phone, 2016/01/07
    OPERATOR_FOREIGN_TAIWAN                          = 102,
    OPERATOR_FOREIGN_TAIWAN_RFMD                  = 103,  //lile@Prd6.BasicDrv.CDT, 2016/10/12, add for match new RFMD TAIWAN mainborad
    OPERATOR_FOREIGN_INDIA                          = 104,
    OPERATOR_FOREIGN_ALLNET                          = 105,
    OPERATOR_FOREIGN_ASIA                          = 106,
    OPERATOR_FOREIGN_ASIA_RFMD                      = 107,  //lile@Prd6.BasicDrv.CDT, 2016/10/12, add for match new RFMD ASIA mainborad
    OPERATOR_FOREIGN_VODAFONE_SKY                  = 108,  //libin@Prd6.BasicDrv.CDT, 2016/11/18, add for match new VODAFONE SKY mainborad
    OPERATOR_FOREIGN_VODAFONE_RFMD                  = 109,  //libin@Prd6.BasicDrv.CDT, 2016/11/18, add for match new VODAFONE RFMD mainborad
    OPERATOR_FOREIGN_ASIA_INDIA                      = 110,  //Mofei@Prd6.BasicDrv.CDT, 2016/12/30, add for match new ASIA and India mainborad
    OPERATOR_FOREIGN_ASIA_WIFI                      = 111,  //Mofei@Prd6.BasicDrv.CDT, 2016/12/30, add for match new ASIA with deffrent wifi mainborad
};


typedef enum OPPO_PROJECT OPPO_PROJECT;

#define OCPCOUNTMAX 4
typedef struct
{
  unsigned int                  nProject;
  unsigned char                 nModem;
  unsigned char                 nOperator;
  unsigned char                 nPCBVersion;
  unsigned char                 nBootMode;
  unsigned char                 nPmicOcp_LDO[OCPCOUNTMAX];
  unsigned char                 nPmicOcp_SMPS[OCPCOUNTMAX];
} ProjectInfoCDTType;

#ifdef CONFIG_OPPO_COMMON_SOFT
unsigned int init_project_version(void);
unsigned int get_project(void);
unsigned int is_project(OPPO_PROJECT project );
unsigned char get_PCB_Version(void);
unsigned char get_Modem_Version(void);
unsigned char get_Operator_Version(void);
#else
unsigned int init_project_version(void) { return 0;}
unsigned int get_project(void) { return 0;}
unsigned int is_project(OPPO_PROJECT project ) { return 0;}
unsigned char get_PCB_Version(void) { return 0;}
unsigned char get_Modem_Version(void) { return 0;}
unsigned char get_Operator_Version(void) { return 0;}
#endif
#endif
