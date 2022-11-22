/* ULP Example: using ADC in deep sleep

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program
   measures input voltage on the given ADC channel 'adc_oversampling_factor'
   times. Measurements are accumulated and average value is calculated.
   Average value is compared to the two thresholds: 'low_thr' and 'high_thr'.
   If the value is less than 'low_thr' or more than 'high_thr', ULP wakes up
   the chip from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"

	/* ADC1 channel 6, GPIO34 */
	.set adc_channel, 0

	/* Configure the number of ADC samples to average on each measurement.
	   For convenience, make it a power of 2. */
	.set adc_oversampling_factor_log, 2
	.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	.global charging
charging:
	.long 0

	/* Low threshold of ADC reading.
	   Set by the main program. */
	.global low_thr
low_thr:
	.long 0

	/* High threshold of ADC reading.
	   Set by the main program. */
	.global high_thr
high_thr:
	.long 0

	/* Counter of measurements done */
	.global sample_counter
sample_counter:
	.long 0

	.global last_result
last_result:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	.global loop

	loop:
	/* increment sample counter */
	//move r3, sample_counter
	//ld r2, r3, 0
	//add r2, r2, 1
	//st r2, r3, 0

	//measure and add value to accumulator
	//adc r1, 1, adc_channel + 1
	//add r0, r0, r1

	//averaged value is now in r0; store it into last_result
	//move r3, last_result
	//st r0, r3, 0

	stage_rst

 	// First flash
  	WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+13,1,1)
  	/* Wait some cycles to have visible trace on the scope */
  	wait_loop_1:
 	WAIT 65535
 	/* increment loop counter and check exit condition */
 	stage_inc 1
 	jumps wait_loop_1, 10, lt

 	/* Set the RTC_GPIO13 output LOW
     to signal that ULP is now up */
  	WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+13,1,0)
  	/* Wait some cycles to have visible trace on the scope */
 	wait_loop_2:
 	WAIT 65535
 	/* increment loop counter and check exit condition */
 	stage_inc 1
 	jumps wait_loop_2, 20, lt

 	// Second flash
  	WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+13,1,1)
  	/* Wait some cycles to have visible trace on the scope */
  	wait_loop_3:
 	WAIT 65535
 	/* increment loop counter and check exit condition */
 	stage_inc 1
 	jumps wait_loop_3, 30, lt

 	/* Set the RTC_GPIO13 output LOW
     to signal that ULP is now up */
  	WRITE_RTC_REG(RTC_GPIO_OUT_REG,RTC_GPIO_OUT_DATA_S+13,1,0)
  	/* Wait some cycles to have visible trace on the scope */
 	wait_loop_4:
 	WAIT 65535
 	/* increment loop counter and check exit condition */
 	stage_inc 1
 	jumps wait_loop_4, 40, lt

 	/* Wait some cycles to have visible trace on the scope */
 	wait_loop_5:
 	WAIT 65535
 	/* increment loop counter and check exit condition */
 	stage_inc 1
 	jumps wait_loop_5, 160, lt

    //measure and add value to accumulator
	//adc r1, 1, adc_channel + 1
	//add r2, r2, r1

	//averaged value is now in r0; store it into last_result
	//move r2, last_result
	//st r1, r2, 0

    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S+5, 16)

  	move r3, r0
	and r0, r3, 1

    JUMP loop, EQ
	JUMP exit, EQ

	.global exit
exit:
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
    halt

	.global wake_up
wake_up:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump exit, eq

	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt
