/* 
 * File:   vrac.h
 * Author: rbeal
 *
 * Created on May 5, 2017, 11:51 AM
 */

#ifndef VRAC_H
#define	VRAC_H

short crc16(unsigned char *Adresse_tab, unsigned char Taille_max);
double lp_filter2(double signal, double *u, double *y);
short odometer(int hall, int hallold);

#endif	/* VRAC_H */

