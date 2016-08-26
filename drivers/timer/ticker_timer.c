#include <system_timer.h>

#include "../bluetooth/controller/util/defines.h"
#include "../bluetooth/controller/hal/clock.h"
#include "../bluetooth/controller/ll/ticker.h"

#ifdef CONFIG_TICKLESS_IDLE
#error "Tickless idle not yet implemented for Ticker timer"
#endif

static uint8_t ALIGNED(4) _ticker_nodes[1][TICKER_NODE_T_SIZE];
static uint8_t ALIGNED(4) _ticker_users[1][TICKER_USER_T_SIZE];
static uint8_t ALIGNED(4) _ticker_user_ops[2][TICKER_USER_OP_T_SIZE];

void ticker_timeout(uint32_t ticks_at_expire, uint32_t remainder, uint16_t lazy,
		    void *context)
{
	ARG_UNUSED(ticks_at_expire);
	ARG_UNUSED(remainder);
	ARG_UNUSED(lazy);
	ARG_UNUSED(context);

	_sys_clock_tick_announce();
}

int _sys_clock_driver_init(struct device *device)
{
	uint32_t us;
	int retval;

	ARG_UNUSED(device);

	clock_k32src_start(1);

	_ticker_users[0][0] = 2;

	retval = ticker_init(1, 1, &_ticker_nodes[0],
			     1, &_ticker_users[0],
			     2, &_ticker_user_ops[0]);
	if (retval) {
		return -1;
	}

	us = 1000000/CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	retval = ticker_start(1, 0, 0,
			      ticker_ticks_now_get(),
			      TICKER_US_TO_TICKS(us),
			      TICKER_US_TO_TICKS(us),
			      TICKER_REMAINDER(us),
			      0, 0,
			      ticker_timeout, 0,
			      0, 0);
	if ((retval == TICKER_STATUS_SUCCESS) ||
	    (retval == TICKER_STATUS_BUSY)) {
		retval = 0;
	}

	return retval;
}
