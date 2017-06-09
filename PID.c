#include <xc.h>
#include <dsp.h>

#include "global.h"
#include "PID.h"

// PID coeff
extern unsigned char pp, ii, dd;

//Declare a PID Data Structure named, fooPID
extern tPID fooPID1;
fractional abcCoefficient1[3] __attribute__((space(xmemory)));
fractional controlHistory1[3] __attribute__((space(ymemory), eds));
fractional kCoeffs1[] = {0, 0, 0};

extern tPID fooPID2;
fractional abcCoefficient2[3] __attribute__((space(xmemory)));
fractional controlHistory2[3] __attribute__((space(ymemory), eds));
fractional kCoeffs2[] = {0, 0, 0};

extern tPID fooPID3;
fractional abcCoefficient3[3] __attribute__((space(xmemory)));
fractional controlHistory3[3] __attribute__((space(ymemory), eds));
fractional kCoeffs3[] = {0, 0, 0};

extern tPID fooPID4;
fractional abcCoefficient4[3] __attribute__((space(xmemory)));
fractional controlHistory4[3] __attribute__((space(ymemory), eds));
fractional kCoeffs4[] = {0, 0, 0};

void initPID() {

    fooPID1.abcCoefficients = &abcCoefficient1[0]; //Set up pointer to derived coefficients

    fooPID1.controlHistory = &controlHistory1[0]; //Set up pointer to controller history samples

    PIDInit(&fooPID1); //Clear the controler history and the controller output

    kCoeffs1[0] = Q15((((float) pp) / 100.)); //P 0.55

    kCoeffs1[1] = Q15((((float) ii) / 100.)); //I 0.1

    kCoeffs1[2] = Q15((((float) dd) / 100.)); //D 0.28

    PIDCoeffCalc(&kCoeffs1[0], &fooPID1); //Derive the a,b, & c coefficients from the Kp, Ki & Kd

    fooPID2.abcCoefficients = &abcCoefficient2[0]; //Set up pointer to derived coefficients

    fooPID2.controlHistory = &controlHistory2[0]; //Set up pointer to controller history samples

    PIDInit(&fooPID2); //Clear the controler history and the controller output

    kCoeffs2[0] = Q15((((float) pp) / 100.)); //P 0.55

    kCoeffs2[1] = Q15((((float) ii) / 100.)); //I 0.1

    kCoeffs2[2] = Q15((((float) dd) / 100.)); //D 0.28

    PIDCoeffCalc(&kCoeffs2[0], &fooPID2); //Derive the a,b, & c coefficients from the Kp, Ki & Kd

    fooPID3.abcCoefficients = &abcCoefficient3[0]; //Set up pointer to derived coefficients

    fooPID3.controlHistory = &controlHistory3[0]; //Set up pointer to controller history samples

    PIDInit(&fooPID3); //Clear the controler history and the controller output

    kCoeffs3[0] = Q15((((float) pp) / 100.)); //P 0.55

    kCoeffs3[1] = Q15((((float) ii) / 100.)); //I 0.1

    kCoeffs3[2] = Q15((((float) dd) / 100.)); //D 0.28

    PIDCoeffCalc(&kCoeffs3[0], &fooPID3); //Derive the a,b, & c coefficients from the Kp, Ki & Kd

    fooPID4.abcCoefficients = &abcCoefficient4[0]; //Set up pointer to derived coefficients

    fooPID4.controlHistory = &controlHistory4[0]; //Set up pointer to controller history samples

    PIDInit(&fooPID4); //Clear the controler history and the controller output

    kCoeffs4[0] = Q15((((float) pp) / 100.)); //P 0.55

    kCoeffs4[1] = Q15((((float) ii) / 100.)); //I 0.1

    kCoeffs4[2] = Q15((((float) dd) / 100.)); //D 0.28

    PIDCoeffCalc(&kCoeffs4[0], &fooPID4); //Derive the a,b, & c coefficients from the Kp, Ki & Kd
}
