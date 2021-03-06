#ifndef SIMPLE_PLANNER_HPP_GUARD_2374982379432
#define SIMPLE_PLANNER_HPP_GUARD_2374982379432

#include "measurements.hpp"

struct Commands
{
  float forward;
  float rotate;
  float weapon;
};

struct SimplePlanner
{
  enum {SP_STATE_IDLE=0, SP_STATE_CENTER=1, SP_STATE_SCAN=2, SP_STATE_ATTACK=3, SP_STATE_SPIN=4, SP_STATE_DELAY=5, SP_STATE_FORWARD=6} state;
  enum {SP_MOVE_IDLE=0, SP_MOVE_BACK=1, SP_MOVE_TURN_RIGHT=3, SP_MOVE_TURN_LEFT=4, SP_MOVE_SHOOT=5} move;
  int delay;
  bool spin_direction;
  void plan(Commands &cmds, const Measurements &measurements);
  void center(Commands &cmds, const Measurements &measurements);
  void center2(Commands &cmds, const Measurements &measurements);
  void spin(Commands &cmds, const Measurements &measurements);
  
  void cmdForward(int d) {state = SP_STATE_FORWARD; delay = d;}
  void cmdCenter() {state = SP_STATE_CENTER; move = SP_MOVE_IDLE;}
  void cmdDelay(int d) {state = SP_STATE_DELAY; delay = d;}
  void cmdSpin() {state = SP_STATE_SPIN; delay = 40;}
  const char* getStateStr() const;
  const char* getMoveStr() const;  
};

#endif //SIMPLE_PLANNER_HPP_GUARD_2374982379432
