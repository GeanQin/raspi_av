#include "RaspiVid.h"

int main()
{
    struct raspi_vid_cfg_t vid_cfg;
    
    vid_cfg.raspi_vid_width = 1920;
    vid_cfg.raspi_vid_height = 1080;
    vid_cfg.raspi_vid_bitrate = 800000;
    vid_cfg.raspi_vid_fps = 20;
    vid_cfg.raspi_vid_gop = 60;

    raspi_vid_init(vid_cfg);
    return 0;
}