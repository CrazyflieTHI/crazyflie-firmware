/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie simulation firmware
 *
 * Copyright (c) 2018  Eric Goubault, Sylvie Putot, Franck Djeumou
 *             Cosynux , LIX , France
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * main_sitl.c - Containing the main function.for the SITL simulation based on main.c
 */

#include <unistd.h>

/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <stdarg.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <string.h>

/* Project includes */
#include "gazebolink.h"
#include "config.h"
#include "system.h"
#include "usec_time.h"
#include "platform.h"
#include "configblock_sim.h"
#include "configblock.h"


/* When configSUPPORT_STATIC_ALLOCATION is set to 1 the application writer can
 * use a callback function to optionally provide the memory required by the idle
 * and timer tasks.  This is the stack that will be used by the timer task.  It is
 * declared here, as a global, so it can be checked by a test that is implemented
 * in a different file. */
StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

/* Notes if the trace is running or not. */
#if ( projCOVERAGE_TEST == 1 )
    static BaseType_t xTraceRunning = pdFALSE;
#else
    static BaseType_t xTraceRunning = pdTRUE;
#endif

void vApplicationIdleHook( void );

/* Necessary for gprof profiler to work */
void handle_signal(int signum) {
    exit(0);
}

int main(int argc, char **argv) 
{
    signal(SIGINT, handle_signal);

    uint8_t crazyflieId = atoi(argv[1]);
    setCrazyflieId(crazyflieId);
    uint64_t crazyflie_address = configblockGetRadioAddress();
    printf("SITL: CF id : %d , address : %lx \n", crazyflieId , crazyflie_address );

  //Initialize the platform.
  int err = platformInit();
  if (err != 0) {
    printf("\nWrong platform!\n\n");
    // The firmware is running on the wrong hardware. Halt
    while(1);
  }

  //Launch the system task that will initialize and start everything
  systemLaunch();

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  //Should never reach this point!
  while(1);

  return 0;
}

/*
* FreeRTOS assert name
*/
void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
    printf("ASSERT: %s : %d\n", pcFileName, (int)ulLine);
    while(1);
}

unsigned long ulGetRunTimeCounterValue(void)
{
    return 0;
}

void vConfigureTimerForRunTimeStats(void)
{
    return;
}

/* For memory management */
void vApplicationMallocFailedHook(void)
{
    while(1);
}

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    * added here, but the tick hook is called from an interrupt context, so
    * code must not attempt to block, and only the interrupt safe FreeRTOS API
    * functions can be used (those that end in FromISR()). */
}

void traceOnEnter()
{
    #if ( TRACE_ON_ENTER == 1 )
        int xReturn;
        struct timeval tv = { 0L, 0L };
        fd_set fds;

        FD_ZERO( &fds );
        FD_SET( STDIN_FILENO, &fds );

        xReturn = select( STDIN_FILENO + 1, &fds, NULL, NULL, &tv );

        if( xReturn > 0 )
        {
            if( xTraceRunning == pdTRUE )
            {
                prvSaveTraceFile();
            }

            /* clear the buffer */
            char buffer[ 0 ];
            read( STDIN_FILENO, &buffer, 1 );
        }
    #endif /* if ( TRACE_ON_ENTER == 1 ) */
}

void vLoggingPrintf( const char * pcFormat,
                     ... )
{
    va_list arg;

    va_start( arg, pcFormat );
    vprintf( pcFormat, arg );
    va_end( arg );
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    /* This function will be called once only, when the daemon task starts to
     * execute    (sometimes called the timer task).  This is useful if the
     * application includes initialisation code that would benefit from executing
     * after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

static void prvSaveTraceFile( void ) {}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void handle_sigint( int signal )
{
    exit( 2 );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* Check pcTaskName for the name of the offending task, or pxCurrentTCB
	if pcTaskName has itself been corrupted. */
	( void ) pxTask;
	( void ) pcTaskName;
	for( ;; );
}


void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     * task.  It is essential that code added to this hook function never attempts
     * to block in any way (for example, call xQueueReceive() with a block time
     * specified, or call vTaskDelay()).  If application tasks make use of the
     * vTaskDelete() API function to delete themselves then it is also important
     * that vApplicationIdleHook() is permitted to return to its calling function,
     * because it is the responsibility of the idle task to clean up memory
     * allocated by the kernel to any task that has since deleted itself. */


    usleep( 15000 );

}
