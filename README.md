IR Logger
=========

This is a sample implementation of a low-power-consumption
infrared receiver for RC5 and/or RC6 for Atmel 8-bit MCUs. The sample
code just logs the raw commands on a serial console. The intended
use case is that you adapt this code for your own project.

Implementation Notes
--------------------

The goal is to let the MCU go into deep sleep, waking up only
when an IR signal is received. The sample code expects an IR
sensor attached to the INT0 pin. It uses timer 0 to measure the
intervals between level changes and for the timeout at end of
signal. All work is done in interrupt handlers. The sample code
runs in a tight loop while waiting for the signal, because there
is nothing else it can do, but your code may continue to run,
checking the queue at convenient times, or starting a routine
from the interrupt handler itself.
