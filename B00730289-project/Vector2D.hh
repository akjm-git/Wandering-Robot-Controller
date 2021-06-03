#ifndef _VECTOR2D_HH_
#define _VECTOR2D_HH_

#include <math.h>

class Vector2D {
public:
  
  Vector2D(): mdx(0), mdy(0) 
  {}
  Vector2D(double x, double y): mdx(x), mdy(y)
  {}
  Vector2D(const Vector2D & v): mdx(v.X()), mdy(v.Y())
  {}
  ~Vector2D() {};

  Vector2D & operator= (const Vector2D & v)
  {
    mdx = v.X();
    mdy = v.Y();
    return (*this);
  }

  Vector2D & operator += (const Vector2D & v)
  {
    mdx += v.X();
    mdy += v.Y();
    
    return (*this);
  }

  Vector2D & operator -= (const Vector2D & v)
  {
    mdx -= v.X();
    mdy -= v.Y();
    
    return (*this);
  }

  Vector2D & operator *= (double l)
  {
    mdx *= l;
    mdy *= l;
    
    return (*this);
  }

  Vector2D & operator /= (double l)
  {
    mdx /= l;
    mdy /= l;
    
    return (*this);
  }

  double X() const
  { return mdx; }
  double Y() const
  { return mdy; }

  void X(double x) 
  { mdx = x; }
  void Y(double y)
  { mdy = y; }
  
  double Length() const
  {
    return sqrt(mdx * mdx + mdy * mdy);
  }
  
  void Rotate(double theta)
  {
    double x = mdx;
    double y = mdy;
    
    mdx = x * cos(theta) - y * sin(theta);
    mdy = x * sin(theta) + y * cos(theta);
  }

  double Angle() const
  {
    return atan2(mdy, mdx);
  }

  void Normalise()
  { 
    double l = this->Length();
    
    mdx /= l;
    mdy /= l;
  }

private:
  double mdx, mdy;
};

Vector2D 
operator+(const Vector2D & u, const Vector2D & v)
{
  Vector2D retVal(u.X() + v.X(), u.Y() + v.Y());
  
  return retVal;
}

Vector2D 
operator-(const Vector2D & u, const Vector2D & v)
{
  Vector2D retVal(u.X() - v.X(), u.Y() - v.Y());
  
  return retVal;
}

Vector2D 
operator-(const Vector2D & u)
{
  Vector2D retVal(-u.X(), -u.Y());
  
  return retVal;
}

Vector2D 
operator*(const double a, const Vector2D & u)
{
  Vector2D retVal(a * u.X(), a * u.Y());
  
  return retVal;
}

double 
Dot(const Vector2D & u, const Vector2D & v)
{
  return u.X() * v.X() + u.Y() * v.Y();
}

Vector2D
operator/(const Vector2D & u, double a)
{
  Vector2D retVal(u.X() / a, u.Y() / a);
  
  return retVal;
}
#endif
