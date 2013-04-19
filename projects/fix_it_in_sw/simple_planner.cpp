#include "simple_planner.hpp"

//#include <iostream>
#include <math.h>

void SimplePlanner::center(Commands &cmds, const Measurements &measurements)
{
  // we want to move forward towards center of ring (if possible)

  float rl_offset = measurements.right - measurements.left;

  cmds.rotate = 0.0f;
  cmds.forward = 0.0f;  

  if (measurements.front < 0.25f)
  {
    // too close to wall, must back up
    cmds.forward = -0.5f;
    move = SP_MOVE_BACK;
  }
  else 
  {
    if ((move == SP_MOVE_BACK) || ((move==SP_MOVE_IDLE) && (measurements.front < 0.8f)))
    {      
      if (rl_offset < -0.01f)
      {
        cmds.rotate = 0.5f;
        move = SP_MOVE_TURN_LEFT;
      }    
      else
      {
        cmds.rotate = -0.5f;
        move = SP_MOVE_TURN_RIGHT;
      }
    }
    else if ((move == SP_MOVE_TURN_LEFT) || (move == SP_MOVE_TURN_RIGHT))
    {
      if (measurements.front > 0.8f)
      {
        //std::cout << "shoot" << std::endl;
        move = SP_MOVE_SHOOT;
        //cmds.forward = 0.5;
      }
      else
      {
        if (move == SP_MOVE_TURN_LEFT)
        {
          cmds.rotate = 0.5f;
        }
        else
        {
          cmds.rotate = -0.5f;
        }
      }
    }
    else if (move == SP_MOVE_SHOOT)
    {      
      // move forward towards center
      cmds.rotate = -rl_offset * 3.0f;
      float abs_rl_offset = fabsf(rl_offset);
      //std::cout << "abs_RL_offset " << abs_rl_offset << std::endl;
      if (abs_rl_offset < 0.6f)
      {
        //std::cout << " not done" << abs_rl_offset << std::endl;
        cmds.forward = 0.5f;
        if (measurements.front < 1.0f)
        {
          //std::cout << "Done " << rl_offset << " " << abs_rl_offset << std::endl;
          state = SP_STATE_SPIN;
        }
      }
    }
    else
    {
      if ((measurements.front > 0.9f) || (fabs(rl_offset) > 0.4f))
      {
        move = SP_MOVE_SHOOT;
      }
      else
      {
        state = SP_STATE_SPIN;
      }
    }
  }
}

void SimplePlanner::spin(Commands &cmds, const Measurements &measurements)
{
  if ((measurements.front < -0.7f) || (measurements.front > 0.7f) ||
      (measurements.right < -0.7f) || (measurements.right > 0.7f) ||
      (measurements.left  < -0.7f) || (measurements.left > 0.7f)
      )
  {
    cmdCenter();
  }
  
  
}

void SimplePlanner::plan(Commands &cmds, const Measurements &measurements)
{
  switch (state)
  {
  case SP_STATE_IDLE:
    cmdCenter();
    break;

  case SP_STATE_CENTER:
    center(cmds, measurements);
    break;
  
  case SP_STATE_SPIN:
    spin(cmds, measurements);
    break;

  default:
    state = SP_STATE_IDLE;
  }
}


const char* SimplePlanner::getStateStr() const
{
  switch (state)
  {
  case SP_STATE_IDLE:   return "idle";
  case SP_STATE_CENTER: return "center";
  case SP_STATE_SCAN:   return "scan";
  case SP_STATE_ATTACK: return "attack";
  default : return "??";
  }
}
  
const char* SimplePlanner::getMoveStr() const
{
  switch (move)
  {
  case SP_MOVE_BACK:       return "back";
  case SP_MOVE_TURN_RIGHT: return "right";
  case SP_MOVE_TURN_LEFT:  return "left";
  case SP_MOVE_SHOOT:      return "shoot";
  default: return "??";
  }
}
