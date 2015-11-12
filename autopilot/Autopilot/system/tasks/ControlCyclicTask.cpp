/*
 * ControlCyclicTask.cpp
 *
 *  Created on: 23 juil. 2013
 *      Author: Aberzen
 */

#include <stdio.h>

#include "ControlCyclicTask.hpp"
#include <system/system/System.hpp>

namespace system {

ControlCyclicTask::ControlCyclicTask()
: infra::CyclicTask((const signed char*)"CTRL",
		(unsigned portSHORT)750,
		(unsigned portBASE_TYPE)2,
		FSW_TASK_CTRL_PERIOD_TICK,
        0)
{
}

ControlCyclicTask::~ControlCyclicTask() {
}

/** @brief Non returning function executed as the task body function. */
void ControlCyclicTask::init(void)
{
	/* Initialize the system */
	system.initialize();

	/* Call super class */
	Task::init();
}

void ControlCyclicTask::runCycle(void)
{
	/* Handle missing cycles */
//	if (_missed != 0)
//	{
//		fdir::FdirManager& fdir = system.getFdir();
//		fdir.signalMissingCycles(_missed);
//		_missed = 0;
//	}

	system.execute();


}

} /* namespace test */
