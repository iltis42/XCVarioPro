
#include "Rotate.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"

Point Point::rotate(const Point& center, float radians) {

    // Verschieben des Punktes zum Koordinatenursprung
    float translatedX = x - center.x;
    float translatedY = y - center.y;

    // Berechnung der rotierten Koordinaten
    float newX = translatedX * fast_cos_rad(radians) - translatedY * fast_sin_rad(radians);
    float newY = translatedX * fast_sin_rad(radians) + translatedY * fast_cos_rad(radians);

    // Verschieben des rotierten Punktes zurück zum ursprünglichen Koordinatensystem
    newX += center.x;
    newY += center.y;

    // Erstellen und Zurückgeben des rotierten Punktes
    Point rotatedPoint( fast_iroundf(newX), fast_iroundf(newY) );
    return rotatedPoint;
}

void Point::moveVertical(int16_t pixel){
    y += pixel;
}
