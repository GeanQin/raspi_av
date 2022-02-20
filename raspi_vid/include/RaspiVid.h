#ifndef RASPI_VID_H
#define RASPI_VID_H

struct raspi_vid_cfg_t
{
    int raspi_vid_width;
    int raspi_vid_height;
    int raspi_vid_bitrate;
    int raspi_vid_fps;
    int raspi_vid_gop;
};

int raspi_vid_init(struct raspi_vid_cfg_t raspi_vid_cfg);

#endif