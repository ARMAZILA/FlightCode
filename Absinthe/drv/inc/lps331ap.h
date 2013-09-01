/*
 * lps331ap.h
 *
 *  Created on: 10.03.2013
 *      Author: avgorbi
 */

#ifndef LPS331AP_H_
#define LPS331AP_H_

#define LPS331AP_ADDRESS         	0x5C	/*  LPS331AP, SA0 = 0 		*/
#define LPS331AP_ID              	0xBB	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define	LPS331AP_REF_P_XL			0x08	/*	pressure reference		*/
#define	LPS331AP_REF_P_L			0x09	/*	pressure reference		*/
#define	LPS331AP_REF_P_H			0x0A	/*	pressure reference		*/
#define	LPS331AP_REF_T_L			0x0B	/*	temperature reference	*/
#define	LPS331AP_REF_T_H			0x0C	/*	temperature reference	*/
#define	LPS331AP_WHO_AM_I			0x0F	/*	WhoAmI register			*/
#define	LPS331AP_RES				0x10	/*	Pres Temp resolution set*/
#define	LPS331AP_DGAIN_L			0x18	/*	Dig Gain (3 regs)		*/
#define	LPS331AP_CTRL_REG1			0x20	/*	power / ODR control reg	*/
#define	LPS331AP_CTRL_REG2			0x21	/*	boot reg				*/
#define	LPS331AP_CTRL_REG3			0x22	/*	interrupt control reg	*/
#define	LPS331AP_INT_CFG_REG		0x23	/*	interrupt config reg	*/
#define	LPS331AP_INT_SRC_REG		0x24	/*	interrupt source reg	*/
#define	LPS331AP_THS_P_L			0x25	/*	pressure threshold		*/
#define	LPS331AP_THS_P_H			0x26	/*	pressure threshold		*/
#define	LPS331AP_STATUS_REG			0X27	/*	status reg				*/
#define	LPS331AP_PRESS_OUT			0xA8	/*	press output (3 regs)	*/
#define	LPS331AP_TEMP_OUT			0xAB	/*	temper output (2 regs)	*/
#define	LPS331AP_COMPENS_L			0x30	/*	compensation reg (9 regs) */


/* REGISTERS MASKS */
#define	LPS331AP_PRS_ENABLE_MASK	0x80	/*  ctrl_reg1 */
#define	LPS331AP_PRS_ODR_MASK		0x70	/*  ctrl_reg1 */
#define	LPS331AP_PRS_DIFF_MASK		0x08	/*  ctrl_reg1 */
#define	LPS331AP_PRS_BDU_MASK		0x04	/*  ctrl_reg1 */
#define	LPS331AP_PRS_AUTOZ_MASK		0x02	/*  ctrl_reg2 */

#define	LPS331AP_PRS_PM_NORMAL		0x80	/* Power Normal Mode*/
#define	LPS331AP_PRS_PM_OFF			0x00	/* Power Down */

#define	LPS331AP_PRS_DIFF_ON		0x08	/* En Difference circuitry */
#define	LPS331AP_PRS_DIFF_OFF		0x00	/* Dis Difference circuitry */

#define	LPS331AP_PRS_AUTOZ_ON		0x02	/* En AutoZero Function */
#define	LPS331AP_PRS_AUTOZ_OFF		0x00	/* Dis Difference Function */

#define	LPS331AP_PRS_BDU_ON			0x04	/* En BDU Block Data Upd */
#define	LPS331AP_PRS_RES_AVGTEMP_064	0x60
#define	LPS331AP_PRS_RES_AVGTEMP_128	0x70
#define	LPS331AP_PRS_RES_AVGPRES_512	0x0A

#define	LPS331AP_PRS_RES_MAX		(LPS331AP_PRS_RES_AVGTEMP_128  | \
						LPS331AP_PRS_RES_AVGPRES_512)
						/* Max Resol. for 1/7/12,5Hz */

#define	LPS331AP_PRS_RES_25HZ		(LPS331AP_PRS_RES_AVGTEMP_064  | \
						LPS331AP_PRS_RES_AVGPRES_512)
						/* Max Resol. for 25Hz */

bool lps331apDetect(void);
void lps331apRead(float* Pressure_mb, float* Temperature_DegC);
void lps331ReadPressure(int16_t * Pressure_mb);
void lps331ReadTemp(float* Temperature_DegC);
void From_Pressure_mb_To_Altitude_cm(float* Pressure_mb, float* Altitude_cm);
void From_ft_To_m(double* ft, double* m);

#endif /* LPS331AP_H_ */
