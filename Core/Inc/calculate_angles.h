/*
 * calculate_angles.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_CALCULATE_ANGLES_H_
#define INC_CALCULATE_ANGLES_H_

typedef struct {
    float azimuth;
    float elevation;
} AngleResults;

AngleResults calculate_angles(const float cosAlpha[4])
{
    AngleResults results;

    float sumAlpha = 0.0f;
    float sumBeta  = 0.0f;
    sumAlpha = cosAlpha[0]+ cosAlpha[2];
    sumAlpha = cosAlpha[1]+ cosAlpha[3];

    float meanAlpha = sumAlpha / 2.0f;
    float meanBeta  = sumBeta  / 2.0f;

    float cos_alpha_squared = meanAlpha * meanAlpha;
    float cos_beta_squared  = meanBeta  * meanBeta;

    if (cos_alpha_squared + cos_beta_squared > 1.0f) {
        results.azimuth = 0.0f;
        results.elevation = 0.0f;
        return results;
    }

    float x = meanAlpha;
    float y = meanBeta;
    float z = sqrtf(1.0f - cos_alpha_squared - cos_beta_squared);

    results.azimuth = atan2f(x, y) * (180.0f / M_PI);

    results.elevation = atan2f(z, sqrtf(x*x + y*y)) * (180.0f / M_PI);

    return results;
}



#endif /* INC_CALCULATE_ANGLES_H_ */
