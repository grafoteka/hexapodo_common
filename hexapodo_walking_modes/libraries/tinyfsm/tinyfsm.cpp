#include <tinyfsm/tinyfsm.hpp>

#include "fsmlist.hpp"

#include <iostream>

using namespace std;

class Idle; // forward declaration


// ----------------------------------------------------------------------------
// Transition functions
//

static void CallMaintenance() {
  cout << "*** calling maintenance ***" << endl;
}

static void CallFirefighters() {
  cout << "*** calling firefighters ***" << endl;
}


// ----------------------------------------------------------------------------
// State: Panic
//

class stand_by_floor
: public Elevator
{
  void entry() override {
    send_event(MotorStop());
  }
};


// ----------------------------------------------------------------------------
// State: Moving
//

class stand_by_up
: public Elevator
{
  void react(FloorSensor const & e) override {
    cout << "Reached floor " << e.floor << endl;

    int floor_expected = current_floor + Motor::getDirection();
    if(floor_expected != e.floor)
    {
      cout << "Floor sensor defect (expected " << floor_expected << ", got " << e.floor << ")" << endl;
      transit<Panic>(CallMaintenance);
    }

    current_floor = e.floor;
    if(e.floor == dest_floor)
      transit<Idle>();
  };
};


// ----------------------------------------------------------------------------
// State: altern_tripod
//

class altern_tripod
: public Elevator
{
  void entry() override {
    send_event(MotorStop());
  }

  void react(Call const & e) override {
    dest_floor = e.floor;

    if(dest_floor == current_floor)
      return;

    /* lambda function used for transition action */
    auto action = [] {
      if(dest_floor > current_floor)
        send_event(MotorUp());
      else if(dest_floor < current_floor)
        send_event(MotorDown());
    };

    transit<Moving>(action);
  };
};


// ----------------------------------------------------------------------------
// State: wave
//

class wave
: public hexapod_modes
{
  void entry() override {
    ROS_INFO("Inicializacion WAVE");
  }

  void react(joy const & e) override {
    dest_floor = e.floor;

    if(dest_floor == current_floor)
      return;

    /* lambda function used for transition action */
    auto action = [] {
      if(dest_floor > current_floor)
        send_event(MotorUp());
      else if(dest_floor < current_floor)
        send_event(MotorDown());
    };

    transit<Moving>(action);
  };
};

// ----------------------------------------------------------------------------
// Base state: default implementations
//

void hexapod_modes::react(Call const &) {
  cout << "Call event ignored" << endl;
}

int hexapod_modes::current_state = 0;
int hexapod_modes::dest_state = 0;


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(hexapod_modes, stand_by_floor)
