

hexapodo::hexapodo()
{
	tripods_quantity = 2;
	tripods_length = 3;

	tripod_one = {0, 3, 4};
	tripod_two = {1, 2, 5};

	robot_configuration = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

}