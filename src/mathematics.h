
double getAngleFrom(double x, double y)
{
  double a = abs(x);
  double b = abs(y);
  double c = sqrt(pow(abs(x), 2) + pow(abs(y), 2));

  double directionAngle = 0;

  if (x == 0 && y > 0)
  {
    directionAngle = 0;
  }
  else if (x == 0 && y < 0)
  {
    directionAngle = 180;
  }
  else if (x > 0 && y == 0)
  {
    directionAngle = 90;
  }
  else if (x < 0 && y == 0)
  {
    directionAngle = 270;
  }
  else
  {
    directionAngle = acos((c * c + a * a - b * b) / (2 * c * a)) * 180 / M_PI;

    if (y > 0)
    {
      if (x > 0)
      {
        directionAngle = 90 - directionAngle;
      }
      else if (x < 0)
      {
        directionAngle = 270 + directionAngle;
      }
    }
    else if (y < 0)
    {
      if (x > 0)
      {
        directionAngle = 90 + directionAngle;
      }
      else if (x < 0)
      {
        directionAngle = 270 - directionAngle;
      }
    }
  }
  return directionAngle;
}