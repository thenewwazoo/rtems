/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tmacros.h>

const char rtems_test_name[] = "SP 51";

/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code sc;
  rtems_id          mutex;

  TEST_BEGIN();

  puts( "Create semaphore - priority ceiling locked - violate ceiling" );
  sc = rtems_semaphore_create(
    rtems_build_name( 'S', 'E', 'M', '1' ),
    0,
    RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY_CEILING | RTEMS_PRIORITY,
    (RTEMS_MAXIMUM_PRIORITY - 4u),
    &mutex
  );
  fatal_directive_status(sc, RTEMS_INVALID_PRIORITY, "rtems_semaphore_create");

  puts( "Create semaphore - priority ceiling unlocked" );
  sc = rtems_semaphore_create(
    rtems_build_name( 'S', 'E', 'M', '1' ),
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY_CEILING | RTEMS_PRIORITY,
    (RTEMS_MAXIMUM_PRIORITY - 4u),
    &mutex
  );
  directive_failed( sc, "rtems_semaphore_create" );

  puts( "Obtain semaphore -- violate ceiling" );
  sc = rtems_semaphore_obtain( mutex, RTEMS_DEFAULT_OPTIONS, 0 );
  fatal_directive_status(
    sc, RTEMS_INVALID_PRIORITY, "rtems_semaphore_obtain" );

  puts( "Release semaphore we did not obtain" );
  sc = rtems_semaphore_release( mutex );
  fatal_directive_status(
    sc, RTEMS_NOT_OWNER_OF_RESOURCE, "rtems_semaphore_release" );

  TEST_END();
  rtems_test_exit( 0 );
}


/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_INIT
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS         1
#define CONFIGURE_MAXIMUM_SEMAPHORES    1

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#include <rtems/confdefs.h>

/****************  END OF CONFIGURATION INFORMATION  ****************/
