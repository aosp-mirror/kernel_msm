/*
 * External charger driver for Fan5451X
 * 3.2A Dual Input, Switch Mode Charger
 *
 * Copyright (C) 2016 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_STAT0		0x00
#define REG_STAT1		0x01
#define REG_STAT2		0x02
#define REG_INT0		0x04
#define REG_INT1		0x05
#define REG_INT2		0x06
#define REG_MINT0		0x08
#define REG_MINT1		0x09
#define REG_MINT2		0x0A
#define REG_CON0		0x0C
#define REG_CON1		0x0D
#define REG_CON2		0x0E
#define REG_CON3		0x0F
#define REG_VFLOAT		0x11
#define REG_IOCHRG		0x12
#define REG_IBAT		0x13
#define REG_IBUS		0x14
#define REG_VBUS		0x15
#define REG_IIN			0x16
#define REG_VIN			0x17
#define REG_NTC			0x18
#define REG_TIMER		0x19
#define REG_SAFETY		0x1A
#define REG_TOPOFF		0x1B
#define REG_BOOST		0x1C
#define REG_DPLUS		0x1F
#define REG_MNT0		0x20
#define REG_MNT1		0x21
#define REG_IC_INFO		0x2D
#define REG_FEAT_CON	0x30

/******************************************************************************
* Register bits
******************************************************************************/
/* REG_STAT0 (0x00) */
#define STAT0_NOBAT				0x01
#define STAT0_SLP_SHIFT			1
#define STAT0_SLP				(0x01 << STAT0_SLP_SHIFT)
#define STAT0_PRE_SHIFT			2
#define STAT0_PRE				(0x01 << STAT0_PRE_SHIFT)
#define STAT0_STAT_SHIFT		3
#define STAT0_STAT				(0x01 << STAT0_STAT_SHIFT)
#define STAT0_PWROK_SHIFT		4
#define STAT0_PWROK				(0x01 << STAT0_PWROK_SHIFT)
#define STAT0_VBUSPWR_SHIFT		5
#define STAT0_VBUSPWR			(0x01 << STAT0_VBUSPWR_SHIFT)
#define STAT0_VINPWR_SHIFT		6
#define STAT0_VINPWR			(0x01 << STAT0_VINPWR_SHIFT)
#define STAT0_BATID_SHIFT		7
#define STAT0_BATID				(0x01 << STAT0_BATID_SHIFT)

/* REG_STAT1 (0x01) */
#define STAT1_BATLO				0x01
#define STAT1_BOOST_SHIFT		1
#define STAT1_BOOST				(0x01 << STAT1_BOOST_SHIFT)
#define STAT1_DIVC_SHIFT		2
#define STAT1_DIVC				(0x01 << STAT1_DIVC_SHIFT)
#define STAT1_TOCHG_SHIFT		3
#define STAT1_TOCHG				(0x01 << STAT1_TOCHG_SHIFT)
#define STAT1_CHGCMP_SHIFT		4
#define STAT1_CHGCMP			(0x01 << STAT1_CHGCMP_SHIFT)
#define STAT1_CHGDET_SHIFT		5
#define STAT1_CHGDET			(0x03 << STAT1_CHGDET_SHIFT)
#define STAT1_LOIBAT_SHIFT		7
#define STAT1_LOIBAT			(0x01 << STAT1_LOIBAT_SHIFT)

/* REG_STAT2 (0x02) */
#define STAT2_TMRTO				0x01
#define STAT2_WDTTO_SHIFT		1
#define STAT2_WDTTO				(0x01 << STAT2_WDTTO_SHIFT)
#define STAT2_TBAT_SHIFT		2
#define STAT2_TBAT				(0x01 << STAT2_TBAT_SHIFT)
#define STAT2_JEITA_SHIFT		3
#define STAT2_JEITA				(0x01 << STAT2_JEITA_SHIFT)
#define STAT2_TEMPFB_SHIFT		4
#define STAT2_TEMPFB			(0X01 << STAT2_TEMPFB_SHIFT)
#define STAT2_TEMPSD_SHIFT		5
#define STAT2_TEMPSD			(0x01 << STAT2_TEMPSD_SHIFT)
#define STAT2_INPUTOVP_SHIFT	6
#define STAT2_INPUTOVP			(0x01 << STAT2_INPUTOVP_SHIFT)
#define STAT2_INPUTSEL_SHIFT	7
#define STAT2_INPUTSEL			(0x01 << STAT2_INPUTSEL_SHIFT)

/* REG_INT0 (0x04) */
#define INT0_BATINT				0x01
#define INT0_WKBAT				0x02
#define INT0_CHGMOD				0x04
#define INT0_CHGEND				0x08
#define INT0_VLOWTH				0x10
#define INT0_VBUSINT			0x20
#define INT0_VININT				0x40
#define INT0_VALFAIL			0x80

/* REG_INT1 (0x05) */
#define INT1_VBATLV				0x01
#define INT1_BSTWDTTO			0x02
#define INT1_BATUVL				0x04
#define INT1_BSTFAIL			0x08
#define INT1_BSTTSD				0x10
#define INT1_BSTOVP				0x20
#define INT1_RCHGN				0x40
#define INT1_IBATLO				0x80

/* REG_INT2 (0x06) */
#define INT2_TIMER				0x01
#define INT2_BATOCP				0x02
#define INT2_OTGOCP				0x04
#define INT2_BATTEMP			0x08
#define INT2_ICTEMP				0x10
#define INT2_SHORTBAT			0x20
#define INT2_OVPINPUT			0x40
#define INT2_TOCMP				0x80

/* REG_MINT0 (0x08) */
#define MINT0_BATINT			0x01
#define MINT0_WKBAT				0x02
#define MINT0_CHGMOD			0x04
#define MINT0_CHGEND			0x08
#define MINT0_VLOWTH			0x10
#define MINT0_VBUSINT			0x20
#define MINT0_VININT			0x40
#define MINT0_VALFAIL			0x80

/* REG_MINT1 (0x09) */
#define MINT1_VBATLV			0x01
#define MINT1_BSTWDTTO			0x02
#define MINT1_BATUVL			0x04
#define MINT1_BSTFAIL			0x08
#define MINT1_BSTTSD			0x10
#define MINT1_BSTOVP			0x20
#define MINT1_RCHGN				0x40
#define MINT1_IBATLO			0x80

/* REG_MINT2 (0x0A) */
#define MINT2_TIMER				0x01
#define MINT2_BATOCP			0x02
#define MINT2_OTGOCP			0x04
#define MINT2_BATTEMP			0x08
#define MINT2_ICTEMP			0x10
#define MINT2_SHORTBAT			0x20
#define MINT2_OVPINPUT			0x40
#define MINT2_TOCMP				0x80

/* REG_CON0 (0x0C) */
#define CON0_VBATMIN			0x07
#define CON0_VLOWV_SHIFT		3
#define CON0_VLOWV				(0x07 << CON0_VLOWV_SHIFT)

/* REG_CON1 (0x0D) */
#define CON1_VSYS				0x07
#define CON1_VLDO_SHIFT			3
#define CON1_VLDO				(0x03 << CON1_VLDO_SHIFT)
#define CON1_LDO_OFF_SHIFT		5
#define CON1_LDO				(0x01 << CON1_LDO_OFF_SHIFT)
#define CON1_GPO1_SHIFT			6
#define CON1_GPO1				(0x01 << CON1_GPO1_SHIFT)
#define CON1_GPO2_SHIFT			7
#define CON1_GPO2				(0x01 << CON1_GPO2_SHIFT)

/* REG_CON2 (0x0E) */
#define CON2_HZMOD_SHIFT		1
#define CON2_HZMOD				(0x01 << CON2_HZMOD_SHIFT)
#define CON2_TOEN_SHIFT			2
#define CON2_TOEN				(0x01 << CON2_TOEN_SHIFT)
#define CON2_TE_SHIFT			3
#define CON2_TE					(0x01 << CON2_TE_SHIFT)
#define CON2_NOBATOP_SHIFT		4
#define CON2_NOBATOP			(0x01 << CON2_NOBATOP_SHIFT)
#define CON2_RCHGDIS_SHIFT		5
#define CON2_RCHGDIS			(0x01 << CON2_RCHGDIS_SHIFT)
#define CON2_CONT_SHIFT			7
#define CON2_CONT				(0x01 << CON2_CONT_SHIFT)

/* REG_CON3 (0x0F) */
#define CON3_CE					0x01
#define CON3_PPOFF_SHIFT		1
#define CON3_PPOFF				(0x01 << CON3_PPOFF_SHIFT)
#define CON3_PPOFFSLP_SHIFT		2
#define CON3_PPOFFSLP			(0x01 << CON3_PPOFFSLP_SHIFT)
#define CON3_TREGTH_SHIFT		5
#define CON3_TREGTH				(0x03 << CON3_TREGTH_SHIFT)
#define CON3_RST_SHIFT			7
#define CON3_RST				(0x01 << CON3_RST_SHIFT)

/* REG_VFLOAT (0x11) */
#define FLOAT_VAL				0xFF

/* REG_IOCHRG (0x12) */
#define IOCHRG_VAL				0x3F

/* REG_IBAT (0x13) */
#define IBAT_PRECHG				0x0F
#define IBAT_ITERM_SHIFT		4
#define IBAT_ITERM				(0x0F << IBAT_ITERM_SHIFT)

/* REG_IBUS (0x14) */
#define IBUS_LIM				0x7F

/* REG_VBUS (0x15) */
#define VBUS_LIM				0x0F
#define VBUS_OVP_SHIFT			4
#define VBUS_OVP				(0x03 << VBUS_OVP_SHIFT)

/* REG_IIN (0x16) */
#define IIN_VAL					0x7F

/* REG_VIN (0x17) */
#define VIN_LIM					0x0F
#define VIN_OVP_SHIFT			4
#define VIN_OVP					(0x03 << VIN_OVP_SHIFT)

/* REG_NTC (0x18) */
#define NTC_1					0x01
#define NTC_2_SHIFT				1
#define NTC_2					(0x01 << NTC_2_SHIFT)
#define NTC_3_SHIFT				2
#define NTC_3					(0x01 << NTC_3_SHIFT)
#define NTC_4_SHIFT				3
#define NTC_4					(0x01 << NTC_4_SHIFT)
#define NTC_OK_SHIFT			4
#define NTC_OK					(0x01 << NTC_OK_SHIFT)
#define NTC_TEMPDIS_SHIFT		5
#define NTC_TEMPDIS				(0x01 << NTC_TEMPDIS_SHIFT)

/* REG_TIMER (0x19) */
#define TIMER_FCTMR				0x07
#define TIMER_PRETMR_SHIFT		3
#define TIMER_PRETMR			(0x03 << TIMER_PRETMR_SHIFT)
#define TIMER_WDEN_SHIFT		6
#define TIMER_WDEN				(0x01 << TIMER_WDEN_SHIFT)
#define TIMER_TMRRST_SHIFT		7
#define TIMER_TMRRST			(0x01 << TIMER_TMRRST_SHIFT)

/* REG_SAFETY (0x1A) */
#define SAFETY_I				0x0F
#define SAFETY_V_SHIFT			4
#define SAFETY_V				(0x0F << 4)

/* REG_TOPOFF (0x1B) */
#define TO_BDETDIS				0x08
#define TOPOFF_TMR				0x07

/* REG_BOOST (0x1C) */
#define BOOST_V					0x1F
#define BOOST_EN_SHIFT			5
#define BOOST_EN				(0x01 << BOOST_EN_SHIFT)
#define BOOST_OTG_SHIFT			6
#define BOOST_OTG				(0x01 << BOOST_OTG_SHIFT)

/* REG_DPLUS (0x1F) */
#define DPLUS_SETTMR0			0x01
#define DPLUS_BC12DET_SHIFT		7
#define DPLUS_BC12DET			(0x01 << DPLUS_BC12DET_SHIFT)

/* REG_MNT0 (0x20) */
#define MNT0_CV					0x01
#define MNT0_ICHG				0x02
#define MNT0_IBUS				0x04
#define MNT0_HIVBAT				0x08
#define MNT0_BATSHORT			0x10
#define MNT0_VLOWVCMP			0x20
#define MNT0_VBATCMP			0x40
#define MNT0_ITERMCMP			0x80

/* REG_MNT1 (0x21) */
#define MNT1_ILINPIN			0x01
#define MNT1_DISPIN				0x02
#define MNT1_NTCGND				0x04
#define MNT1_VBUSCMP			0x08
#define MNT1_BUCKON				0x10
#define MNT1_PPON				0x20
#define MNT1_PMIDVBAT			0x40

/* REG_IC_INFO (0x2D) */
#define IC_INFO_REV				0x07
#define IC_INFO_PN_SHIFT		3
#define IC_INFO_PN				(0x07 << IC_INFO_PN_SHIFT)
#define IC_INFO_VC_SHIFT		6
#define IC_INFO_VC				(0x03 << IC_INFO_VC_SHIFT)

/* REG_FEAT_CON (0x30) */
#define FEAT_CON_ENREF_SHIFT	4
#define FEAT_CON_ENREF			(0x01 << FEAT_CON_ENREF_SHIFT)
#define FEAT_CON_DIVCON_SHIFT	5
#define FEAT_CON_DIVCON			(0x01 << FEAT_CON_DIVCON_SHIFT)
