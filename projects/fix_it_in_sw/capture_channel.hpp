#ifndef CAPTURE_CHANNEL_HPP_GUARD_9719037412074
#define CAPTURE_CHANNEL_HPP_GUARD_9719037412074

// acts like a state machine for a single capture channel,
// it keeps track of what next event should be, last event time, and errors
class CaptureChannel
{
  
  uint32_t last_falling_edge;
  uint32_t last_rising_edge;
  
  enum State {CC_STATE_INIT=0, CC_STATE_WAITING_FOR_FALLING=1, CC_STATE_WAITING_FOR_RISING=2} state;
  uint32_t pulse_width;
  bool pulse_width_valid;
  uint32_t pulse_count; // count of valid pulses
  uint32_t overflow_count; // count of overflows that have occurred since last valid edge
  
  uint32_t pulse_period; // more for debugging 

  uint32_t errors;
  uint32_t updates; 

public:
  enum Edge {CC_RISING_EDGE=1, CC_FALLING_EDGE=0};

  CaptureChannel()
  {
    state = CC_STATE_INIT;
    pulse_width_valid = false;
  }

  // returns pulse width (in counter ticks), or 0 if no pulses have been captured or too many overflow
  // have occurred without new pulse being seen
  uint32_t getPulseWidth()
  {
    if (pulse_width_valid)
    {
      return pulse_width;
    }
    else 
    {
      return 0;
    }
  }

  // should be called when counter overflow ocurrs, used for determining 
  // if pulses have stopped occurring.
  void overflow()
  {
    ++overflow_count;
    if (overflow_count >= 4)
    {
      pulse_width_valid = false;
    }
  }

  // should be called when capture event occurs
  // with counter of capture event time, and edge (rising/falling) that was captured
  void event(uint32_t cnt, Edge edge)
  {
    switch (state)
    {
    case CC_STATE_INIT:
      if (edge == CC_RISING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_FALLING;
      }
      // else do nothing, still waiting for first rising edge
      break;
    case CC_STATE_WAITING_FOR_FALLING:
      if (edge == CC_FALLING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_RISING;
        if (overflow_count < 2)
        {          
          // because there is a race condition between overflow and input capture event, 
          // it is difficult to use overflow to properly caculate pulse width longer than period
          // however, it is possible to calculate pulse width if it is assumed that the pulse width
          // is shorter than the overflow period
          pulse_width = (cnt - last_rising_edge) & 0xFFFF;
          pulse_width_valid = true;
          ++pulse_count;
        }
        else 
        {
          // too many overflows have occurred since last update
          // pulse width calculation will be incorrect. 
          error();
        }          
      }
      else
      {
        // unexpected edge 
        error();
      }
      break;
    case CC_STATE_WAITING_FOR_RISING:
      if (edge == CC_RISING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_FALLING;
        pulse_period = (cnt - last_rising_edge) & 0xFFFF;
        overflow_count = 0;
      }
      else
      {
        error();
      }
      break;
    default:
      error();
    }

    if (edge == CC_RISING_EDGE)
    {
      last_rising_edge = cnt;
    }
    else
    {
      last_falling_edge = cnt;
    }    
    ++updates;
  }

  void error()
  {
    state = CC_STATE_INIT;
    ++errors;    
  }

};



#endif //CAPTURE_CHANNEL_HPP_GUARD_9719037412074
