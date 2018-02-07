// Own header
#include "vector2.h"

#include <math.h>


Vector2 Vector2::operator + (Vector2 const &b) const
{
    return Vector2(x + b.x, y + b.y);
}


Vector2 Vector2::operator - (Vector2 const &b) const
{
    return Vector2(x - b.x, y - b.y);
}


Vector2 Vector2::operator * (double const b) const
{
    return Vector2(x * b, y * b);
}


Vector2 const &Vector2::Normalize()
{
    double lenSqrd = x*x + y*y;
    if (lenSqrd > 0.0f)
    {
        double invLen = 1.0f / sqrtf(lenSqrd);
        x *= invLen;
        y *= invLen;
    }
    else
    {
        x = 0.0f;
        y = 1.0f;
    }

    return *this;
}


void Vector2::SetLen(double len)
{
    double scaler = len / Len();
    x *= scaler;
    y *= scaler;
}


void Vector2::Rotate(double radians)
{
    double tmp = x;
    double cs = cos(radians);
    double sn = sin(radians);
    x = x * cs - y * sn;
    y = tmp * sn + y * cs;
}


double Vector2::AngleBetween(Vector2 const &other)
{
    Vector2 thisNormalized = *this;
    thisNormalized.Normalize();
    Vector2 otherNormalized = other;
    otherNormalized.Normalize();
    return thisNormalized.y * otherNormalized.x - thisNormalized.x * otherNormalized.y;
}


Vector2 Vector2::GetPerpendicular()
{
    return Vector2(y, -x);
}


double Vector2::Len() const
{
    return sqrtf(x * x + y * y);
}
