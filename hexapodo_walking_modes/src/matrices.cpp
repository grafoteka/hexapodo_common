bool move_legs(bool start_movement, float phase_one_velocity, float phase_two_velocity)
{
    // Angulos
    float total_angle = 60; 
    float total_angle_rads = total_angle * PI / 180;
    float take_off_angle = total_angle_rads / 2;
    float take_land_angle = 2 * PI - (total_angle_rads / 2);

    // Fases
    int phase;
    int phase_mod = phase % 2;

    // Patas del robot
    const int tripods_quantity = 2; const int tripods_length = 3;
    std::vector<int> tripod_one = {0, 3, 4};
    std::vector<int> tripod_two = {1, 2, 5};
    
    //Matriz de la posicion de las patas del robot
    int robot_legs_configuration[tripods_quantity][tripods_length]; // = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

    for(int i = 0; i < tripods_quantity; i++)
    {
        for(int j = 0; j < tripods_length; j++)
        {
            if(i = 0)
                robot_legs_configuration[i][j] = tripod_one.at(j);
            else
                robot_legs_configuration[i][j] = tripod_two.at(j);
        }
    }
    //Matriz de que las patas han llegado a su consigna
    static std::std::vector<bool> leg_in_position_vector = {false, false, false, false, false, false};
    static bool leg_in_position[tripods_quantity][tripods_length] = {{false, false, false},{false, false, false}};
    

    /**** INICIO DEL MOVIMIENTO DE LAS PATAS SI NO SE ENCUENTRAN YA EN POSICION ****/
    if(start_movement)
    {
        /* SET VELOCITIES */
        for(int i = 0, tripods_quantity, i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                if (leg_in_position[i][j] == false)
                {
                    if(phase_mod == 0)
                    {
                        //Si estamos en el primer ciclo, el tripode 1 va por el suelo y el tripode 2 por el aire
                        pata[robot_legs_configuration[0][j]]->setVelocity(phase_one_velocity);
                        pata[robot_legs_configuration[1][j]]->setVelocity(phase_two_velocity);
                    }
                    else
                    {
                        pata[robot_legs_configuration[0][j]]->setVelocity(phase_two_velocity);
                        pata[robot_legs_configuration[1][j]]->setVelocity(phase_one_velocity);
                        
                    }
                }
            }
        }
    }
    

    /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
    else
    {
        for(int i = 0, tripods_quantity, i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                float tripod_one_angle, tripod_two_angle;
                float distance;
                float leg_actual_pos = fmod(patas_posicion_actual.at(tripod_one.at(i)), 2 * PI);

                if(phase_mod == 0) 
                {
                    if(i == 0)
                        angle = take_off_angle; //Angulo para tripode_1 y phase_0
                    else //(i == 1)
                        angle = take_land_angle;    //Angulo para el tripode_1 y phase_1
                    // tripod_one_angle = take_off_angle;
                    // tripod_two_angle = take_land_angle;
                }
                else
                {
                    if(i == 0)
                        angle = take_land_angle;
                    else //(i = 1)
                        angle = take_off_angle;
                    // tripod_one_angle = take_land_angle;
                    // tripod_two_angle = take_off_angle;
                }

                distance = fabs(angle - leg_actual_pos);

                if(distance < 0.05)
                {
                    leg_in_position[i][j] = true;
                    pata[robot_legs_configuration[i][j]]->setVelocity(0.0);

                    // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
                    if(i == 0)
                        leg_in_position_vector.at(j) = true;
                    else
                        leg_in_position_vector.at(j+3) = true;
                }
            }
        }
    }

    /* COMPROBACION DE QUE LOS TRIPODES ESTAN EN LA POSICION DESEADA */
    // Si estan todas las patas en posicion, entonces se resetea el estado de sus valores 
    // Y tambien se devuelve el estado de la variable FLAG de la maquina de estados
    bool position_reached = std::all_of(leg_in_position_vector.begin(), leg_in_position_vector.end(), [](bool reached) {return reached;});

    if(position_reached)
    {
        for(int i = 0, tripods_quantity, i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                leg_in_position[i][j] = false;
            
                if(i == 0)
                    leg_in_position_vector.at(j) = false;
                else
                    leg_in_position_vector.at(j+3) = false;
        }

        phase++;

        return true;
    }

    else
        return false;
    
}


/*********************************************
    MAQUINA DE ESTADOS DEL TRIPODE ALTERNO
**********************************************/

robot_modes altern_tripod_walking(float phase_one_velocity, float phase_two_velocity, bool exit)
{
    bool flag = false;

    switch(state_actual)
    {
        case origin:
            if(!exit)
            {
                move_legs(true, phase_one_velocity, phase_two_velocity);
                state_actual = transition;
                state_next = phase_1;
                state_last = origin;
            }

        break;

        case phase_1:
            if(!exit && state_next == transition)
                flag = move_legs(false, phase_one_velocity, phase_two_velocity);
                state_actual = transition;
                state_next = phase_2;
                state_last = phase_1;
        break;

        case phase_2:
            if(!exit && state_next == transition)
                flag = move_legs(false, phase_one_velocity, phase_two_velocity);
                state_actual = transition;
                state_next = phase_1;
                state_last = phase_2;
        break;

        case transition:
            if(state_old == origin && state_next == phase_1)
                flag = move_legs(false, phase_one_velocity, phase_two_velocity);
            if(flag)
            {
                state_actual = phase_1;
                state_next = transition;
                state_last = transition;
            }

            if(state_old == phase_1 && state_next == phase_2)
                flag = move_legs(false, phase_one_velocity, phase_two_velocity);
            if(flag)
            {
                state_actual = phase_2;
                state_next = transition;
                state_last = transition;
            }

            if(state_old == phase_2 && state_next == phase_1)
                flag = move_legs(false, phase_one_velocity, phase_two_velocity);
            if(flag)
            {
                state_actual = phase_1;
                state_next = transition;
                state_last = transition;
            }

        break;

    }
}







//####################################################################################
//  Programa principal
//####################################################################################

int main(int argc, char **argv)
{

    //---------------------------------------------------------------------------------
    //  Setup
    //---------------------------------------------------------------------------------

    //Inicio del nodo en ROS y el node handle
    ros::init(argc, argv, "gait_template");
    ros::NodeHandle nh;

    //Suscripción al tópico del JointStates y cmd_vel
    //JointStates entrega el estado de cada articulacion
    ros::Subscriber sub = nh.subscribe("/rhex/joint_states", 1, legs_position_callback);
    //cmd_vel da los valores de velocidad lineal y angular
    ros::Subscriber velocity = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_callback);

    ros::Rate rate(50.0);
    
    // Vector que define que las 6 patas no se encuentran en el origen 0.0
    // Esto es para la primera vez que se inicialice el robot, que pase del suelo
    // a la posición de espera.
    legs_joint_state.position.resize(6);
    legs_in_origin.resize(6);
    for(int i = 0; i < 6; i++)
    {
    legs_in_origin.at(i) = false;
    }

    state_next = phase_1;

    //Se inicializan las patas
    setupPatas(nh);

    //---------------------------------------------------------------------------------
    //  Rutinas de una sola ejecucion
    //---------------------------------------------------------------------------------


    //---------------------------------------------------------------------------------
    //  Bucle del programa principal
    //---------------------------------------------------------------------------------

    bool avance_old;

    while(ros::ok()){

    switch (robot_modes_actual) {
      case ground:
        robot_modes_actual = stand_up({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        break;

      case stand_by:
        ROS_INFO("Estoy en Stand By");

        if (avance)
          robot_modes_actual = altern_tripod;
        break;

      case altern_tripod:
        ROS_INFO("Estoy en altern Tripod");
        if(avance)
        {
            altern_tripod_walking(phase_one_velocity, phase_two_velocity, false);
            avance_old = avance;
        }

        if (avance != avance_old)
        {
            altern_tripod_walking(phase_one_velocity, phase_two_velocity, true);          
        }
        break;

      default:
        ROS_INFO("Default");
        if(avance)
            robot_modes_actual = ground;
        break;

    }
    
    rate.sleep();
    ros::spinOnce();
  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}