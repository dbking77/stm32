
// convert pulse time in in microseconds to distace in meters
float convertSonarData(int time_usec)
{
  return float(time_usec) * 0.0001727891156462585f;
}

struct SonarFilter
{
  float average;

  SonarFilter()
  {
    average = 0.0f;
  }

  float update(int time_usec)
  {
    float distance = convertSonarData(time_usec);
    average += 0.5 * (distance - average);
    return average;
  }
};
