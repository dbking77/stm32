#ifndef MEASUREMENTS_HPP_GUARD_29374239874
#define MEASUREMENTS_HPP_GUARD_29374239874

struct Measurements
{
  float front,right,left,back;

  static float min(float d1, float d2);

  void min(const Measurements &m2)
  {
    front = min(front,m2.front);
    right = min(right,m2.right);
    left = min(left,m2.left);
  }
};

#endif
