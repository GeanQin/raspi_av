#include <stdio.h>
#include <sysexits.h>

#include "raspi_video.h"
#include "bcm_host.h"

RASPIVID_STATE state;

static void default_status()
{
	memset(&state, 0, sizeof(RASPIVID_STATE));
	
}

int raspi_video_init()
{
	int ret = EX_OK;

	bcm_host_init();


	return ret;
}

int raspi_video_deinit()
{
	int ret = EX_OK;

	return ret;
}