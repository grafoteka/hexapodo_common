class hexapodo(){
	
	private:
		const int tripods_quantity = 2;
		const int tripods_length = 3;

		std::vector<int> tripod_one = {0, 3, 4};
		std::vector<int> tripod_two = {1, 2, 5};

		int robot_legs_configuration[tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

	public:



}


class movimientos(){

	private:
		void forward();
		void turn();
		void forward_and_turn();

}

movimientos::forward(bool exit, state_actual)
{
	bool flag;
	switch (state_actual){
		case origin:
            if(!exit)
            {
              ROS_INFO("Origin");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_1;
                state_last = origin;
            }
        	break;

        case phase_1:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_1");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_2;
                state_last = phase_1;
                ROS_INFO("Move to TRANSITION");
            }
            if(exit)
            {
              ROS_INFO("phase_1 exit");
              for(int i = 0; i < tripods_quantity; i++)
              {
                  for(int j = 0; j < tripods_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_legs_configuration[0][j]]->setPosition(take_off_angle);
                    }
                    else
                      pata[robot_legs_configuration[1][j]]->setPosition(take_land_angle);
                  }
              }
            }
        	break;

        case phase_2:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_2");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_1;
                state_last = phase_2;
            }
            if(exit)
            {
              for(int i = 0; i < tripods_quantity; i++)
              {
                  for(int j = 0; j < tripods_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_legs_configuration[0][j]]->setPosition(take_land_angle);
                    }
                    else
                      pata[robot_legs_configuration[1][j]]->setPosition(take_off_angle);
                  }
              }
            }
        	break;

        case transition:
            if(state_last == origin && state_next == phase_1){
                ROS_INFO("transition from origin");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from origin");
                    state_actual = phase_1;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_1 && state_next == phase_2){
              ROS_INFO("transition from phase 1");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_2;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_2 && state_next == phase_1){
              ROS_INFO("transition from phase 2");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_1;
                    state_next = transition;
                    state_last = transition;
                }
            }
        	break;
	}
}