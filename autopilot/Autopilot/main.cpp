/*
 * main.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include <gcs/channel/mavlink_bridge.hpp>
#include <Arduino.h>
#include <system/system/System.hpp>
#include <system/tasks/ControlCyclicTask.hpp>
#include <gcs/channel/SerialChannel.hpp>


system::ControlCyclicTask taskControl;

int main(int argc, char **argv)
{
	/* Initialize Arduino library */
	arduino_init();

	/* End by starting scheduler */
	vTaskStartScheduler();

}

