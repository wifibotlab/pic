
//~~~~~~~~~~~~~~~~~~~~~~~init lp_filter~~~~~~~~~~~~~~~~~~~~~~~~~~~
double lp_filter2(double signal, double *u, double *y)
{
         const int m=1;  //m = order of denominator of low pass filter

         u[m] = signal;

         y[m] = 0.9608*y[m-1] + 0.0392*u[m-1];   // these coefficients come from a discretized low pass filter with a pole at 2 rad/sec

         u[m-1] = u[m];          // initialize past values for next frame
         y[m-1] = y[m];

         return y[m];
}

short crc16(unsigned char *Adresse_tab, unsigned char Taille_max) {

    unsigned int Crc = 0xFFFF;

    unsigned int Polynome = 0xA001;

    unsigned int CptOctet = 0;

    unsigned int CptBit = 0;

    unsigned int Parity = 0;

    Crc = 0xFFFF;

    Polynome = 0xA001; // Polyn\F4me = 2^15 + 2^13 + 2^0 = 0xA001.

    for (CptOctet = 0; CptOctet < Taille_max; CptOctet++) {

        Crc ^= *(Adresse_tab + CptOctet); //Ou exculsif entre octet message et CRC

        for (CptBit = 0; CptBit <= 7; CptBit++) /* Mise a 0 du compteur nombre de bits */ {

            Parity = Crc;

            Crc >>= 1; // D\E9calage a droite du crc

            if (Parity % 2 == 1) Crc ^= Polynome; // Test si nombre impair -> Apres decalage \E0 droite il y aura une retenue
        } // "ou exclusif" entre le CRC et le polynome generateur.
    }
    return (Crc);
}

short odometer(int hall, int hallold) {

    switch (hall) {
        case 5:
            if (hallold == 4) return 0;
            else if (hallold == 1) return 1;
            break;
        case 1:
            if (hallold == 5) return 0;
            else if (hallold == 3) return 1;
            break;
        case 3:
            if (hallold == 1) return 0;
            else if (hallold == 2) return 1;
            break;
        case 2:
            if (hallold == 3) return 0;
            else if (hallold == 6) return 1;
            break;
        case 6:
            if (hallold == 2) return 0;
            else if (hallold == 4) return 1;
            break;
        case 4:
            if (hallold == 6) return 0;
            else if (hallold == 5) return 1;
            break;
        default:
            return 2;
            break;
    }
    return 2;
}