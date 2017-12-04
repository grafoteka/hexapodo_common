#ifndef ELEVATOR_HPP_INCLUDED
#define ELEVATOR_HPP_INCLUDED

#include <tinyfsm.hpp>


// ----------------------------------------------------------------------------
// Event declarations
//

struct joy_event : tinyfsm::Event { };

// ----------------------------------------------------------------------------
// Elevator (FSM base class) declaration
//

class hexapod_modes
: public tinyfsm::Fsm<hexapod_modes>
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:

  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };

  void react(joy        const &);

  virtual void entry(void) { };  /* entry actions in some states */
  void         exit(void)  { };  /* no exit actions at all */

protected:

  static int current_state;
  static int dest_state;
};


#endif