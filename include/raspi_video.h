#ifndef RASPI_VIDEO_H
#define RASPI_VIDEO_H

#include <stdbool.h>

#include "interface/mmal/mmal_pool.h"
#include "interface/mmal/mmal_types.h"
#include "interface/mmal/mmal_component.h"
#include "interface/mmal/util/mmal_connection.h"
#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"

typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
   char *cb_buff;                       /// Circular buffer
   int   cb_len;                        /// Length of buffer
   int   cb_wptr;                       /// Current write pointer
   int   cb_wrap;                       /// Has buffer wrapped at least once?
   int   cb_data;                       /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60*1000)
   int   iframe_buff[IFRAME_BUFSIZE];          /// buffer of iframe pointers
   int   iframe_buff_wpos;
   int   iframe_buff_rpos;
   char  header_bytes[29];
   int  header_wptr;
   FILE *imv_file_handle;               /// File handle to write inline motion vectors to.
   FILE *raw_file_handle;               /// File handle to write raw data to.
   int  flush_buffers;
   FILE *pts_file_handle;               /// File timestamps
} PORT_USERDATA;

typedef enum
{
   RAW_OUTPUT_FMT_YUV = 0,
   RAW_OUTPUT_FMT_RGB,
   RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;

typedef struct RASPIVID_STATE_S RASPIVID_STATE;

struct RASPIVID_STATE_S
{
	RASPICOMMONSETTINGS_PARAMETERS common_settings; // 通用设置
	int timeout;									// 获取多久的视频
	MMAL_FOURCC_T encoding;							// 视频编码 (MJPEG or H264)
	int bitrate;									// 比特率
	int framerate;									// 码率 (fps)
	int intraperiod;								// GOP (key frame rate)
	int quantisationParameter;						// 视频质量. 把bitrate设为0，将其变为可变比特率
	int bInlineHeaders;								// Insert inline headers to stream (SPS, PPS)
	int demoMode;									// Run app in demo mode
	int demoInterval;								// Interval between camera settings changes
	int immutableInput;								// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	// the camera output or the encoder output (with compression artifacts)
	int profile;	// H264 profile to use for encoding
	int level;		// H264 level to use for encoding
	int waitMethod; // Method for switching between pause and capture

	int onTime;	 // In timed cycle mode, the amount of time the capture is on per cycle
	int offTime; // In timed cycle mode, the amount of time the capture is off per cycle

	int segmentSize;   // Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;   // Point at which to wrap segment counter
	int segmentNumber; // Current segment counter
	int splitNow;	   // Split at next possible i-frame if set to 1.
	int splitWait;	   // Switch if user wants splited files

	RASPIPREVIEW_PARAMETERS preview_parameters;	  // Preview setup parameters
	RASPICAM_CAMERA_PARAMETERS camera_parameters; // Camera setup parameters

	MMAL_COMPONENT_T *camera_component;		// Pointer to the camera component
	MMAL_COMPONENT_T *splitter_component;	// Pointer to the splitter component
	MMAL_COMPONENT_T *encoder_component;	// Pointer to the encoder component
	MMAL_CONNECTION_T *preview_connection;	// Pointer to the connection from camera or splitter to preview
	MMAL_CONNECTION_T *splitter_connection; // Pointer to the connection from camera to splitter
	MMAL_CONNECTION_T *encoder_connection;	// Pointer to the connection from camera to encoder

	MMAL_POOL_T *splitter_pool; // Pointer to the pool of buffers used by splitter output port 0
	MMAL_POOL_T *encoder_pool;	// Pointer to the pool of buffers used by encoder output port

	PORT_USERDATA callback_data; // Used to move data to the encoder callback

	int bCapturing;		 // State of capture/pause
	int bCircularBuffer; // Whether we are writing to a circular buffer

	int inlineMotionVectors;	   // Encoder outputs inline Motion Vectors
	char *imv_filename;			   // filename of inline Motion Vectors output
	int raw_output;				   // Output raw video from camera as well
	RAW_OUTPUT_FMT raw_output_fmt; // The raw video format
	char *raw_filename;			   // Filename for raw video output
	int intra_refresh_type;		   // What intra refresh type to use. -1 to not set.
	int frame;
	char *pts_filename;
	int save_pts;
	int64_t starttime;
	int64_t lasttime;

	bool netListen;
	MMAL_BOOL_T addSPSTiming;
	int slices;
};

#endif