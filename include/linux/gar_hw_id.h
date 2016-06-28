#ifndef __GAR_HW_ID_H
#define __GAR_HW_ID_H
/* Implement GAR HW ID/PROJECT ID */

#define GAR_HWID_TYPE_STRLEN	20

enum GAR_PROJ_ID_TYPE {
	PFW1		= 0,
	PFW2		= 1,
	PFW3		= 2,
	PFW4		= 3,
	PFW5		= 4,
	PFW6		= 5,
	PROJ_ID_INVALID	= 6,
	PROJ_ID_SIZE	= 7,
};

enum GAR_HW_ID_TYPE {
	HVT3		= 0,
	EVT		= 1,
	DVT		= 2,
	PVT		= 3,
	SA		= 4,
	DVT1_2		= 5,
	HW_ID_INVALID	= 6,
	HW_ID_SIZE	= 7,
};

typedef struct {
	long gar_proj_id;
	long gar_hw_id;
	long gar_qfuse;
} gar_hwid_data_type;

extern long get_gar_proj_id(void);
extern long get_gar_hw_id(void);
extern void gar_hwid_info_read(void);

#endif /* __GAR_HW_ID_H */
