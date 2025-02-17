/*
 * calculate_angles.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_CALCULATE_ANGLES_H_
#define INC_CALCULATE_ANGLES_H_
#include <math.h>
typedef struct {
    float azimuth;
    float elevation;
} AngleResults;

AngleResults calculate_angles(const float cosAlpha[4], const float cosBeta[4])
{
    AngleResults results;


    float sumAlpha = 0.0f;
    float sumBeta  = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        sumAlpha += cosAlpha[i];
        sumBeta  += cosBeta[i];
    }
    float meanAlpha = sumAlpha / 4.0f;
    float meanBeta  = sumBeta  / 4.0f;

    // Od tego miejsca - identyczna logika, jak w starej calculate_angles(...)
    float cos_alpha_squared = meanAlpha * meanAlpha;
    float cos_beta_squared  = meanBeta  * meanBeta;

    // Warunek sprawdzający, czy nie przekraczamy 1 (czy istnieje rozwiązanie):
    if (cos_alpha_squared + cos_beta_squared > 1.0f) {
        // Brak geometrycznego przecięcia stożków => błąd pomiaru
        results.azimuth = 0.0f;
        results.elevation = 0.0f;
        return results;
    }

    // Wyliczenie "x, y, z" wg Twoich wzorów
    float x = meanAlpha;
    float y = meanBeta;
    float z = sqrtf(1.0f - cos_alpha_squared - cos_beta_squared);

    // azimuth = atan2(x, y) => jak w Twoim kodzie
    results.azimuth = atan2f(x, y) * (180.0f / M_PI);

    // elevation = atan2(z, sqrt(x^2 + y^2)) lub arcsin(z) => jak w Twoim kodzie
    results.elevation = atan2f(z, sqrtf(x*x + y*y)) * (180.0f / M_PI);

    return results;
}



#endif /* INC_CALCULATE_ANGLES_H_ */
