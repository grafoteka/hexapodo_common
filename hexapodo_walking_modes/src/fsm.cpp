#include <tinyfsm/tinyfsm.hpp>
#include <fsm.h>


// ----------------------------------------------------------------------------
// Base state: default implementations
//
void hexapod_modes::react(change_mode const &) {
  cout << "Call event ignored" << endl;
}

int hexapod_modes::state_current = 0;
int hexapod_modes::state_dest = 0;
int hexapod_modes::state_last = 0;
