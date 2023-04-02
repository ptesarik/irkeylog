/* Preprocessor defines that can be tweaked from the Makefile:
 *
 * F_CPU            [Hz] CPU clock frequency.
 *                  Default: NO DEFAULT!!!
 */

#ifndef F_CPU
# error "F_CPU must be set in the Makefile!"
#endif

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*****************************************************************
 * Configuration section
 */

/* Serial port baud rate */
#define BAUD_RATE	115200

/* Timer prescaler - these two defines must match */
#define TMR_PRESCALE	64
#define TMR_TCCR0B_CS	0x03

/* Duration tolerance in percents */
#define IR_TOLERANCE	25

/* Extra debugging code */
#define IR_DEBUG	1

/*
 * End of configuration section
 ****************************************************************/

#define USEC_PER_SEC	1000000UL
#define DIV_ROUND(x, y)	(((x)+((y)/2))/(y))
#define USEC_TO_TICKS(x) DIV_ROUND((x)*(F_CPU/TMR_PRESCALE), USEC_PER_SEC)

/*****************************************************************
 * Definitions for the Philips RC5 protocol
 */

/* Duration of one bit in microseconds */
#define BIT_USEC_RC5	1778

/* Minimum duration of silence at end of signal */
#define EOS_USEC_RC5	(2*BIT_USEC_RC5)

/* Duration of one bit in timer ticks */
#define BIT_TICKS_RC5	USEC_TO_TICKS(BIT_USEC_RC5)

/* End of signal silence duration in timer ticks */
#define EOS_TICKS_RC5	USEC_TO_TICKS(EOS_USEC_RC5)

/*****************************************************************
 * Definitions for the Philips RC6 protocol
 */

/* Duration of one bit in microseconds */
#define BIT_USEC_RC6	895

/* Minimum duration of silence at end of signal */
#define EOS_USEC_RC6	(4*BIT_USEC_RC6)

/* Duration of one bit in timer ticks */
#define BIT_TICKS_RC6	USEC_TO_TICKS(BIT_USEC_RC6)

/* End of signal silence duration in timer ticks */
#define EOS_TICKS_RC6	USEC_TO_TICKS(EOS_USEC_RC6)

/*****************************************************************
 * Common variables for decoding
 */

enum ir_proto {
	RC_5,
	RC_6,
};

/* Decoded data types */
typedef uint16_t ir_addr_t;
typedef uint32_t ir_val_t;

/* Compute the min and max tolerated values */
#define IR_TOLERANCE_MIN(x)	(((x)*(100L-IR_TOLERANCE))/100L)
#define IR_TOLERANCE_MAX(x)	(((x)*(100L+IR_TOLERANCE))/100L)

/* 16-bit timer counter */
typedef uint16_t tcnt_t;

static volatile uint8_t cnt0_hi;	/* Timer overflow counter */
static tcnt_t cnt0_timeout;		/* Current timeout value */
static bool cnt0_pending;		/* Timeout pending after overflow */

static tcnt_t ir_timestamp;		/* Last IR transition */

/* The delta buffer must be big enough for the whole IR transmission
 * (including start/stop bits, etc.)
 */
#define DELTA_BUFLEN	64
static tcnt_t ir_delta[DELTA_BUFLEN];

static volatile uint8_t ir_index;	/* Current timestamp index */
static volatile uint8_t ir_start;	/* Current signal's start index */

static bool ir_toggle;
static ir_addr_t ir_addr;
static ir_val_t ir_val;

struct ir_decode {
	uint8_t idx;		/* Index in delta array */
	uint8_t end;		/* End index in delta array */
	tcnt_t off;		/* Time offset in current delta */
	char next;		/* Next bit in decoding */
}
struct ir_cmd {
	enum ir_proto proto;
	int8_t bits;
#if IR_DEBUG
	int8_t deltas;
#endif
	bool toggle;
	ir_addr_t addr;
	ir_val_t val;
};

#define CMD_QLEN	16
static struct ir_cmd cmd_queue[CMD_QLEN];
static volatile uint8_t cmd_head, cmd_tail;

static uint8_t ir_nextbit;	/* Next bit during decode */
enum ir_proto ir_proto;

/* Check timeout and return true if still pending */
static bool
check_timeout(void)
{
	if (cnt0_hi == cnt0_timeout >> 8) {
		OCR0A = cnt0_timeout & 0xff;
		TIFR0 = _BV(OCF0A);
		TIMSK0 |= _BV(OCIE0A);
		return false;
	}
	return true;
}

/* Set up timeout */
static void
set_timeout(tcnt_t when)
{
	cnt0_timeout = when;
	cnt0_pending = check_timeout();
}

/* Move to the next delta */
#define ir_next_delta(var)				\
	do {						\
		typeof(var) __tmp = (var);		\
		++__tmp;				\
		__tmp %= DELTA_BUFLEN;			\
		(var) = __tmp;				\
	} while (0)

/* Store one bit of data */
#define ir_store_bit(where, bit)		\
	do {					\
		typeof(where) __tmp = (where);	\
		__tmp <<= 1;			\
	        __tmp |= (bit);			\
		(where) = __tmp;		\
	} while(0)

/*****************************************************************
 * Decode function for the Philips RC5 protocol
 */

/* Philips RC-5 uses Manchester encoding.
 * Each bit must contain a transition in the middle, and the maximum
 * time to the next transition is equal to bit duration.
 */
static char
ir_decode_bit_rc5(struct ir_decode *dec)
{
	tcnt_t delta;

	delta = ir_delta[dec->idx] - dec->off;
	if (delta < IR_TOLERANCE_MIN(BIT_TICKS_RC5)/2 ||
	    delta > IR_TOLERANCE_MAX(BIT_TICKS_RC5)/2)
		return -1;
	ir_next_delta(&dec->idx);

	char bit = dec->next;
	if (dec->idx != dec->end) {
		delta = ir_delta[dec->idx];
		if (delta < IR_TOLERANCE_MIN(BIT_TICKS_RC5)/2)
			return -1;

		if (delta > IR_TOLERANCE_MAX(BIT_TICKS_RC5)/2) {
			dec->off = delta >> 1;
			dec->next ^= 1;
		} else {
			ir_next_delta(&dec->idx);
			dec->off = 0;
		}
	}
	return bit;
}

/* Start bit in RC-5 appears as a transition to HIGH.
 * The preceding LOW state is arbitrarily long, but the initial
 * offset is calculated as if it was exactly half a period to
 * allow use of the standard bit decoding routine.
 */
static short
ir_decode_rc5(uint8_t start, uint8_t end)
{
	char bit, bitnum;
	struct ir_decode dec;

	/* Initialize decoder state */
	dec.idx = start;
	dec.end = end;
	dec.off = ir_delta[start] - BIT_TICKS_RC5/2;
	dec.next = 1;

	/* Process Start bit */
	if (ir_decode_bit_rc5(&dec) < 0)
		return -1;

	/* Field bit must be inverted */
	if ( (bit = ir_decode_bit_rc5(&dec)) < 0)
		return -1;
	ir_store_bit(ir_val, bit ^ 1);

	/* Decode the Toggle bit */
	if ( (bit = ir_decode_bit_rc5(&dec)) < 0)
		return -1;
	ir_toggle = bit;

	bitnum = 3;
	while (dec.idx != dec.end) {
		if ( (bit = ir_decode_bit_rc5(&dec)) < 0)
			return -1;
		if (++bitnum < 9)
			ir_store_bit(ir_addr, bit);
		else
			ir_store_bit(ir_val, bit);
	}
	return bitnum;
}

/*****************************************************************
 * Decode functions for the Philips RC6 protocol
 */

/* Philips RC-6 uses Manchester encoding.
 * Each bit must contain a transition in the middle, and the maximum
 * time to the next transition is equal to bit duration.
 */
static char
ir_decode_bit_rc6(uint8_t *idx, uint8_t end)
{
	if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6)/2 ||
	    ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6)/2)
		return -1;
	ir_next_delta(*idx);

	char bit = ir_nextbit;
	if (*idx != end) {
		if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6)/2)
			return -1;

		if (ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6)/2) {
			ir_delta[*idx] >>= 1;
			ir_nextbit ^= 1;
		} else
			ir_next_delta(*idx);
	}
	return bit;
}

/* This is identical to ir_decode_bit, but a double-width bit follows,
 * so if there is no transition, the duration must be reduced to 2/3
 * instead of 1/2.
 *
 * Used only in RC-6 header.
 */
static char
ir_decode_bit_before_double(uint8_t *idx, uint8_t end)
{
	if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6)/2 ||
	    ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6)/2)
		return -1;
	ir_next_delta(*idx);

	char bit = ir_nextbit;
	if (*idx != end) {
		if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6)/2)
			return -1;

		if (ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6)/2) {
			ir_delta[*idx] = (ir_delta[*idx] << 1) / 3;
			ir_nextbit ^= 1;
		} else
			ir_next_delta(*idx);
	}
	return bit;
}

/* Again, almost identical to the above routine, but this time
 * for the double-width bit. A normal bit follows, so if there
 * is no transition after this bit, the duration must be reduced
 * to 1/3.
 *
 * Used only in RC-6 header.
 */
static char
ir_decode_double_bit(uint8_t *idx, uint8_t end)
{
	if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6) ||
	    ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6))
		return -1;
	ir_next_delta(*idx);

	char bit = ir_nextbit;
	if (*idx != end) {
		if (ir_delta[*idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6))
			return -1;

		if (ir_delta[*idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6)) {
			ir_delta[*idx] = ir_delta[*idx] / 3;
			ir_nextbit ^= 1;
		} else
			ir_next_delta(*idx);
	}
	return bit;
}

/* There are two start bits in RC-6, the first one is 4 times
 * the normal length. However, the MCU may need some time
 * to wake up for the first interrupt, so we only check that
 * the Start bit is not too long. It can be short.
 */
static short
ir_decode_rc6(uint8_t start, uint8_t end)
{
	uint8_t idx = start;
	char bit, bits;
	struct ir_decode dec;

	/* Initialize decoder state */
	dec.idx = start;
	dec.end = end;

	/* Process Start bit */
	if (ir_delta[dec.idx] < 3*IR_TOLERANCE_MIN(BIT_TICKS_RC6) ||
	    ir_delta[dec.idx] > 3*IR_TOLERANCE_MAX(BIT_TICKS_RC6))
		return -1;
	ir_next_delta(idx);

	if (ir_delta[idx] < IR_TOLERANCE_MIN(BIT_TICKS_RC6) ||
	    ir_delta[idx] > IR_TOLERANCE_MAX(BIT_TICKS_RC6))
		return -1;
	ir_next_delta(idx);

	/* Check second Start bit (must be always 1) */
	dec.off = 0;
	dec.next = 1;
	if (ir_decode_bit_rc6(&idx, end) != 1)
		return -1;

	/* Decode the Field bits - last one is special */
	if ( (bit = ir_decode_bit_rc6(&idx, end)) < 0)
		return -1;
	ir_store_bit(ir_val, bit);
	if ( (bit = ir_decode_bit_rc6(&idx, end)) < 0)
		return -1;
	ir_store_bit(ir_val, bit);
	if ( (bit = ir_decode_bit_before_double(&idx, end)) < 0)
		return -1;
	ir_store_bit(ir_val, bit);

	/* Decode the Toggle bit, but do not store it */
	if ( (bit = ir_decode_double_bit(&idx, end)) < 0)
		return -1;
	ir_toggle = bit;

	bits = 5;
	while (idx != end) {
		if ( (bit = ir_decode_bit_rc6(&idx, end)) < 0)
			return -1;
		if (++bits < 15)
			ir_store_bit(ir_addr, bit);
		else
			ir_store_bit(ir_val, bit);
	}
	return bits;
}

/* Call with interrupts disabled */
static tcnt_t
read_cnt0(void)
{
	uint8_t lo = TCNT0;
	uint8_t hi = cnt0_hi;

	if (lo < 0xff && (TIFR0 & _BV(TOV0)))
		++hi;
	return lo | ((tcnt_t)hi << 8);
}

ISR(INT0_vect)
{
	tcnt_t cnt = read_cnt0();

	/* Enable interrupts on any change of INT0 */
	EICRA |= _BV(ISC00);

	/* Calculate and store delta from last timestamp */
	ir_delta[ir_index] = cnt - ir_timestamp;
	ir_next_delta(ir_index);

	/* Update current timestamp */
	ir_timestamp = cnt;

	/* Set up timer for end of signal;
	 * RC5 has a shorter EOS marker, so try it first
	 */
	ir_proto = RC_5;
	set_timeout(cnt + EOS_TICKS_RC5);
}

ISR(TIMER0_COMPA_vect)
{
	uint8_t start;
	uint8_t end = ir_index;
	struct ir_cmd *cmd;
	short bits;

	/* Disable timer interrupts */
	TIMSK0 &= ~_BV(OCIE0A);

	/* Initialize decoding state */
	ir_addr = 0;
	ir_val = 0;
	start = ir_start;

	if (ir_proto == RC_5) {
		bits = ir_decode_rc5(start, end);
		if (bits < 0) {
			/* Format error: try RC-6 */
			ir_proto = RC_6;
			set_timeout(ir_timestamp + EOS_TICKS_RC6);
			return;
		}
	} else
		bits = ir_decode_rc6(start, end);

	/* Queue the decoded result */
	cmd = &cmd_queue[cmd_head];
#if IR_DEBUG
	cmd->deltas = (end - start) % DELTA_BUFLEN;
#endif
	cmd->bits = bits;
	cmd->toggle = ir_toggle;
	cmd->addr = ir_addr;
	cmd->val = ir_val;
	cmd_head = (cmd_head + 1) % CMD_QLEN;

	/* Start a new signal */
	ir_start = end;
}

ISR(TIMER0_OVF_vect)
{
	++cnt0_hi;
	if (cnt0_pending)
		cnt0_pending = check_timeout();
}

static void
uart_setup(void)
{
        UCSR0A = _BV(U2X0);	/* Double speed mode USART0 */
	UBRR0L = F_CPU / (BAUD_RATE * 8L) - 1;
        UBRR0H = (F_CPU / (BAUD_RATE * 8L) - 1) >> 8;

        UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); /* 8-bit characters */
        UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RXD and TXD */

        /* Supress line noise by enabling pull-up resistor on pin D0 */
        DDRD &= ~_BV(PIND0);
        PORTD |= _BV(PIND0);
}

static void
uart_putch(char ch)
{
        while (!(UCSR0A & _BV(UDRE0)))
		/* wait until Data Register accepts more data */;
        UDR0 = ch;
}

static void
uart_puts(const char *s)
{
	while (*s)
		uart_putch(*s++);
}

static void
uart_putdec(int8_t n)
{
	char str[4];
	char *p;

	if (n < 0) {
		uart_putch('-');
		n = -n;
	}

	p = &str[3];
	*p-- = '\0';
	do {
		*p-- = '0' + n % 10;
		n /= 10;
	} while (n);
	uart_puts(p + 1);
}

static void
uart_puthex16(uint16_t val)
{
	uint8_t shift = 16;
	char digit;

	while (shift) {
		shift -= 4;
		digit = (val >> shift) & 0xf;
		if (digit < 10)
			digit += '0';
		else
			digit += 'A' - 10;
		uart_putch(digit);
	}
}

static void
uart_puthex32(uint32_t val)
{
	uint8_t shift = 32;
	char digit;

	while (shift) {
		shift -= 4;
		digit = (val >> shift) & 0xf;
		if (digit < 10)
			digit += '0';
		else
			digit += 'A' - 10;
		uart_putch(digit);
	}
}

/* main program starts here */
int
main(void)
{
	/* If you go to sleep, then take a truly deep sleep */
	set_sleep_mode(SLEEP_MODE_STANDBY);

	/* Set up UART */
	uart_setup();

	/* Set counter to normal operation, configure prescaler */
	TCCR0A = 0x00;
	TCCR0B = TMR_TCCR0B_CS << CS00;
	TIMSK0 |= _BV(TOIE0);	/* Enable timer overflow interrupt */

	/* Enable INT0 interrupt */
	EIMSK |= _BV(INT0);

	/* And enable interrupts globally! */
	sei();

	for (;;) {
		cli();
		if (ir_index == ir_start) {
			/* Change INT0 to level-triggered, as that is the
			 * only mode that works in power-down mode.
			 */
			EICRA &= ~_BV(ISC00);

			sleep_enable();
			sei();
			sleep_cpu();
			sleep_disable();
		}
		sei();

		while (cmd_tail == cmd_head)
			/* do nothing */ ;

		/* De-queue oldest result */
		struct ir_cmd *cmd = &cmd_queue[cmd_tail];
		cmd_tail = (cmd_tail + 1) % CMD_QLEN;

		if (cmd->bits >= 0) {
			switch (cmd->proto) {
			case RC_5:
				uart_puts("RC5 ");
				break;

			case RC_6:
				uart_puts("RC6 ");
				break;

			default:
				uart_puts("??? ");
			}
			uart_putdec(cmd->bits);
			uart_puts(" bits: 0x");
			uart_puthex16(cmd->addr);
			uart_puts(" 0x");
			uart_puthex32(cmd->val);
			uart_puts(" toggle=");
			uart_putdec(cmd->toggle);
			uart_puts("\r\n");
		} else {
			uart_puts("format error");
#if IR_DEBUG
			uint8_t i;

			uart_puts(": ");
			uart_putdec(cmd->deltas);
			uart_puts(" deltas\r\n");
			for (i = 0; i < DELTA_BUFLEN; ++i) {
				uart_puthex16(ir_delta[i]);
				if ((i & 7) == 7) {
					if (i == ir_start)
						uart_putch('<');
					uart_puts("\r\n");
				} else if (i == ir_start)
					uart_putch('<');
				else
					uart_putch(' ');
			}
#endif
		}

		while (!(UCSR0A & _BV(TXC0)))
			/* wait until transmit complete */;
		UCSR0A |= _BV(TXC0);
	}
}
