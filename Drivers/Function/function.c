#include "function.h"

#define NEX_TIMEOUTMON 500

char IOmap[500];


int nex_SetPDO_Mapping(uint16 slave)
{
	int retval;
	uint8 u8val;
	uint16 u16val;
	uint32 u32val;
	uint16 SM2_PDO_OUT = 0x1600;
	uint16 SM3_PDO_INPUT = 0x1A00;
	retval = 0;

	u8val = 0;
	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);
	u32val = 0x60400010;//keywords
	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x01, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x607A0020;
	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x02, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60B80010;
	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x03, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
//	u32val = 0x60710010;//target Torque
//	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x04, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u8val = 3;//
	retval += nex_SDOwrite(slave, SM2_PDO_OUT, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);

	u8val = 0;
	retval += nex_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);
	u16val = SM2_PDO_OUT;//
	retval += nex_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, NEX_TIMEOUTRXM);
	u8val = 1;
	retval += nex_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);

	u8val = 0;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);
	u32val = 0x603F0010;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x01, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60410010;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x02, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60610008;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x03, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60640020;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x04, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60B90010;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x05, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60BA0020;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x06, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u32val = 0x60FD0020;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x07, FALSE, sizeof(u32val), &u32val, NEX_TIMEOUTRXM);
	u8val = 7;
	retval += nex_SDOwrite(slave, SM3_PDO_INPUT, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);

	u8val = 0;
	retval += nex_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);
	u16val = SM3_PDO_INPUT;//
	retval += nex_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, NEX_TIMEOUTRXM);
	u8val = 1;
	retval += nex_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);

	u8val = 8;//operation model
	retval += nex_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, NEX_TIMEOUTRXM);

    while(EcatError) printf("%s", nex_elist2string());

    printf("AEP slave %d set, retval = %d\n", slave, retval);
    return 1;
}


int nex_mastaer_init(void)
{
	if(nex_init())
	{
		debug_printf("nex_init()function ok\n");
		
		if(nex_config_init()>0)
		{
			debug_printf("nex_config_init()function ok\n");
		}
		else
		{
			debug_printf("nex_config_init()function execution failed\n");
		}
	}
		
	return 1;
}




