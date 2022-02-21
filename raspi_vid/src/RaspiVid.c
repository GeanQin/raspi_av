#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiHelpers.h"
#include "RaspiGPS.h"
#include "RaspiVid.h"

#include <semaphore.h>

#include <stdbool.h>

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000;	  // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000;  // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms

/// Capture/Pause switch method
/// Simply capture for time specified
enum
{
	WAIT_METHOD_NONE,	  /// Simply capture for time specified
	WAIT_METHOD_TIMED,	  /// Cycle between capture and pause for times specified
	WAIT_METHOD_KEYPRESS, /// Switch between capture and pause on keypress
	WAIT_METHOD_SIGNAL,	  /// Switch between capture and pause on signal
	WAIT_METHOD_FOREVER	  /// Run/record forever
};

// Forward
typedef struct RASPIVID_STATE_S RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
	FILE *file_handle;		/// File handle to write buffer data to.
	RASPIVID_STATE *pstate; /// pointer to our state in case required in callback
	int abort;				/// Set to 1 in callback if an error occurs to attempt to abort the capture
	int cb_len;				/// Length of buffer
	int cb_wptr;			/// Current write pointer
	int cb_wrap;			/// Has buffer wrapped at least once?
	int cb_data;			/// Valid bytes in buffer
#define IFRAME_BUFSIZE (60 * 1000)
	int iframe_buff[IFRAME_BUFSIZE]; /// buffer of iframe pointers
	int iframe_buff_wpos;
	int iframe_buff_rpos;
	char header_bytes[29];
	int header_wptr;
	int flush_buffers;
} PORT_USERDATA;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE_S
{
	RASPICOMMONSETTINGS_PARAMETERS common_settings; /// Common settings
	int timeout;									/// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
	MMAL_FOURCC_T encoding;							/// Requested codec video encoding (MJPEG or H264)
	int bitrate;									/// Requested bitrate
	int framerate;									/// Requested frame rate (fps)
	int intraperiod;								/// Intra-refresh period (key frame rate)
	int quantisationParameter;						/// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
	int bInlineHeaders;								/// Insert inline headers to stream (SPS, PPS)
	int demoMode;									/// Run app in demo mode
	int demoInterval;								/// Interval between camera settings changes
	int immutableInput;								/// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	/// the camera output or the encoder output (with compression artifacts)
	int profile;	/// H264 profile to use for encoding
	int level;		/// H264 level to use for encoding
	int waitMethod; /// Method for switching between pause and capture

	int onTime;	 /// In timed cycle mode, the amount of time the capture is on per cycle
	int offTime; /// In timed cycle mode, the amount of time the capture is off per cycle

	int segmentSize;   /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;   /// Point at which to wrap segment counter
	int segmentNumber; /// Current segment counter
	int splitNow;	   /// Split at next possible i-frame if set to 1.
	int splitWait;	   /// Switch if user wants splited files

	RASPIPREVIEW_PARAMETERS preview_parameters;	  /// Preview setup parameters
	RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

	MMAL_COMPONENT_T *camera_component;	   /// Pointer to the camera component
	MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
	MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder
	MMAL_POOL_T *encoder_pool;			   /// Pointer to the pool of buffers used by encoder output port

	PORT_USERDATA callback_data; /// Used to move data to the encoder callback

	int bCapturing;		 /// State of capture/pause
	int bCircularBuffer; /// Whether we are writing to a circular buffer

	int inlineMotionVectors; /// Encoder outputs inline Motion Vectors
	int intra_refresh_type;	 /// What intra refresh type to use. -1 to not set.
	int frame;
	int64_t starttime;
	int64_t lasttime;

	bool netListen;
	MMAL_BOOL_T addSPSTiming;
	int slices;
};

static MMAL_PORT_T *camera_video_port = NULL;
static MMAL_PORT_T *encoder_input_port = NULL;
static MMAL_PORT_T *encoder_output_port = NULL;
static RASPIVID_STATE state;
static int running = 0;

/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T profile_map[] =
	{
		{"baseline", MMAL_VIDEO_PROFILE_H264_BASELINE},
		{"main", MMAL_VIDEO_PROFILE_H264_MAIN},
		{"high", MMAL_VIDEO_PROFILE_H264_HIGH},
		//   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T level_map[] =
	{
		{"4", MMAL_VIDEO_LEVEL_H264_4},
		{"4.1", MMAL_VIDEO_LEVEL_H264_41},
		{"4.2", MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T initial_map[] =
	{
		{"record", 0},
		{"pause", 1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

static XREF_T intra_refresh_map[] =
	{
		{"cyclic", MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
		{"adaptive", MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
		{"both", MMAL_VIDEO_INTRA_REFRESH_BOTH},
		{"cyclicrows", MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
		//   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static struct
{
	char *description;
	int nextWaitMethod;
} wait_method_description[] =
	{
		{"Simple capture", WAIT_METHOD_NONE},
		{"Capture forever", WAIT_METHOD_FOREVER},
		{"Cycle on time", WAIT_METHOD_TIMED},
		{"Cycle on keypress", WAIT_METHOD_KEYPRESS},
		{"Cycle on signal", WAIT_METHOD_SIGNAL},
};

static int wait_method_description_size = sizeof(wait_method_description) / sizeof(wait_method_description[0]);

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status()
{
	// Default everything to zero
	memset(&state, 0, sizeof(RASPIVID_STATE));

	raspicommonsettings_set_defaults(&(state.common_settings));

	// Now set anything non-zero
	state.timeout = 0; // replaced with 5000ms later if unset
	state.common_settings.filename = "-";
	state.common_settings.width = 1920; // Default to 1080p
	state.common_settings.height = 1080;
	state.encoding = MMAL_ENCODING_H264;
	state.bitrate = 17000000; // This is a decent default bitrate for 1080p
	state.framerate = VIDEO_FRAME_RATE_NUM;
	state.intraperiod = -1; // Not set
	state.quantisationParameter = 0;
	state.demoMode = 0;
	state.demoInterval = 250; // ms
	state.immutableInput = 1;
	state.profile = MMAL_VIDEO_PROFILE_H264_HIGH;
	state.level = MMAL_VIDEO_LEVEL_H264_4;
	state.waitMethod = WAIT_METHOD_FOREVER;
	state.onTime = 5000;
	state.offTime = 5000;
	state.bCapturing = 0;
	state.bInlineHeaders = 0;
	state.segmentSize = 0; // 0 = not segmenting the file.
	state.segmentNumber = 1;
	state.segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
	state.splitNow = 0;
	state.splitWait = 0;
	state.inlineMotionVectors = 0;
	state.intra_refresh_type = -1;
	state.frame = 0;
	state.netListen = false;
	state.addSPSTiming = MMAL_FALSE;
	state.slices = 1;

	// Setup preview window defaults
	raspipreview_set_defaults(&(state.preview_parameters));

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&(state.camera_parameters));
}

/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status()
{
	int i;

	raspicommonsettings_dump_parameters(&(state.common_settings));

	fprintf(stderr, "bitrate %d, framerate %d, time delay %d\n", state.bitrate, state.framerate, state.timeout);
	fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state.profile, profile_map, profile_map_size));
	fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state.level, level_map, level_map_size));
	fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state.quantisationParameter, state.bInlineHeaders ? "Yes" : "No");
	fprintf(stderr, "H264 Fill SPS Timings %s\n", state.addSPSTiming ? "Yes" : "No");
	fprintf(stderr, "H264 Intra refresh type %s, period %d\n", raspicli_unmap_xref(state.intra_refresh_type, intra_refresh_map, intra_refresh_map_size), state.intraperiod);
	fprintf(stderr, "H264 Slices %d\n", state.slices);

	// Not going to display segment data unless asked for it.
	if (state.segmentSize)
		fprintf(stderr, "Segment size %d, segment wrap value %d, initial segment number %d\n", state.segmentSize, state.segmentWrap, state.segmentNumber);

	fprintf(stderr, "Wait method : ");
	for (i = 0; i < wait_method_description_size; i++)
	{
		if (state.waitMethod == wait_method_description[i].nextWaitMethod)
			fprintf(stderr, "%s", wait_method_description[i].description);
	}
	fprintf(stderr, "\nInitial state '%s'\n", raspicli_unmap_xref(state.bCapturing, initial_map, initial_map_size));
	fprintf(stderr, "\n\n");

	raspicamcontrol_dump_parameters(&(state.camera_parameters));
}

/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(RASPIVID_STATE *pState, char *filename)
{
	FILE *new_handle = NULL;
	char *tempname = NULL;

	if (pState->segmentSize || pState->splitWait)
	{
		// Create a new filename string

		// If %d/%u or any valid combination e.g. %04d is specified, assume segment number.
		bool bSegmentNumber = false;
		const char *pPercent = strchr(filename, '%');
		if (pPercent)
		{
			pPercent++;
			while (isdigit(*pPercent))
				pPercent++;
			if (*pPercent == 'u' || *pPercent == 'd')
				bSegmentNumber = true;
		}

		if (bSegmentNumber)
		{
			asprintf(&tempname, filename, pState->segmentNumber);
		}
		else
		{
			char temp_ts_str[100];
			time_t t = time(NULL);
			struct tm *tm = localtime(&t);
			strftime(temp_ts_str, 100, filename, tm);
			asprintf(&tempname, "%s", temp_ts_str);
		}

		filename = tempname;
	}

	if (filename)
	{
		bool bNetwork = false;
		int sfd = -1, socktype;

		if (!strncmp("tcp://", filename, 6))
		{
			bNetwork = true;
			socktype = SOCK_STREAM;
		}
		else if (!strncmp("udp://", filename, 6))
		{
			if (pState->netListen)
			{
				fprintf(stderr, "No support for listening in UDP mode\n");
				exit(131);
			}
			bNetwork = true;
			socktype = SOCK_DGRAM;
		}

		if (bNetwork)
		{
			unsigned short port;
			filename += 6;
			char *colon;
			if (NULL == (colon = strchr(filename, ':')))
			{
				fprintf(stderr, "%s is not a valid IPv4:port, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
						filename);
				exit(132);
			}
			if (1 != sscanf(colon + 1, "%hu", &port))
			{
				fprintf(stderr,
						"Port parse failed. %s is not a valid network file name, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
						filename);
				exit(133);
			}
			char chTmp = *colon;
			*colon = 0;

			struct sockaddr_in saddr = {};
			saddr.sin_family = AF_INET;
			saddr.sin_port = htons(port);
			if (0 == inet_aton(filename, &saddr.sin_addr))
			{
				fprintf(stderr, "inet_aton failed. %s is not a valid IPv4 address\n",
						filename);
				exit(134);
			}
			*colon = chTmp;

			if (pState->netListen)
			{
				int sockListen = socket(AF_INET, SOCK_STREAM, 0);
				if (sockListen >= 0)
				{
					int iTmp = 1;
					setsockopt(sockListen, SOL_SOCKET, SO_REUSEADDR, &iTmp, sizeof(int)); // no error handling, just go on
					if (bind(sockListen, (struct sockaddr *)&saddr, sizeof(saddr)) >= 0)
					{
						while ((-1 == (iTmp = listen(sockListen, 0))) && (EINTR == errno))
							;
						if (-1 != iTmp)
						{
							fprintf(stderr, "Waiting for a TCP connection on %s:%" SCNu16 "...",
									inet_ntoa(saddr.sin_addr), ntohs(saddr.sin_port));
							struct sockaddr_in cli_addr;
							socklen_t clilen = sizeof(cli_addr);
							while ((-1 == (sfd = accept(sockListen, (struct sockaddr *)&cli_addr, &clilen))) && (EINTR == errno))
								;
							if (sfd >= 0)
								fprintf(stderr, "Client connected from %s:%" SCNu16 "\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));
							else
								fprintf(stderr, "Error on accept: %s\n", strerror(errno));
						}
						else // if (-1 != iTmp)
						{
							fprintf(stderr, "Error trying to listen on a socket: %s\n", strerror(errno));
						}
					}
					else // if (bind(sockListen, (struct sockaddr *) &saddr, sizeof(saddr)) >= 0)
					{
						fprintf(stderr, "Error on binding socket: %s\n", strerror(errno));
					}
				}
				else // if (sockListen >= 0)
				{
					fprintf(stderr, "Error creating socket: %s\n", strerror(errno));
				}

				if (sockListen >= 0)   // regardless success or error
					close(sockListen); // do not listen on a given port anymore
			}
			else // if (pState->netListen)
			{
				if (0 <= (sfd = socket(AF_INET, socktype, 0)))
				{
					fprintf(stderr, "Connecting to %s:%hu...", inet_ntoa(saddr.sin_addr), port);

					int iTmp = 1;
					while ((-1 == (iTmp = connect(sfd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in)))) && (EINTR == errno))
						;
					if (iTmp < 0)
						fprintf(stderr, "error: %s\n", strerror(errno));
					else
						fprintf(stderr, "connected, sending video...\n");
				}
				else
					fprintf(stderr, "Error creating socket: %s\n", strerror(errno));
			}

			if (sfd >= 0)
				new_handle = fdopen(sfd, "w");
		}
		else
		{
			new_handle = fopen(filename, "wb");
		}
	}

	if (pState->common_settings.verbose)
	{
		if (new_handle)
			fprintf(stderr, "Opening output file \"%s\"\n", filename);
		else
			fprintf(stderr, "Failed to open new file \"%s\"\n", filename);
	}

	if (tempname)
		free(tempname);

	return new_handle;
}

/**
 * Update any annotation data specific to the video.
 * This simply passes on the setting from cli, or
 * if application defined annotate requested, updates
 * with the H264 parameters
 *
 * @param state Pointer to state control struct
 *
 */
static void update_annotation_data()
{
	// So, if we have asked for a application supplied string, set it to the H264 or GPS parameters
	if (state.camera_parameters.enable_annotate & ANNOTATE_APP_TEXT)
	{
		char *text;

		if (state.common_settings.gps)
		{
			text = raspi_gps_location_string();
		}
		else
		{
			const char *refresh = raspicli_unmap_xref(state.intra_refresh_type, intra_refresh_map, intra_refresh_map_size);

			asprintf(&text, "%dk,%df,%s,%d,%s,%s",
					 state.bitrate / 1000, state.framerate,
					 refresh ? refresh : "(none)",
					 state.intraperiod,
					 raspicli_unmap_xref(state.profile, profile_map, profile_map_size),
					 raspicli_unmap_xref(state.level, level_map, level_map_size));
		}

		raspicamcontrol_set_annotate(state.camera_component, state.camera_parameters.enable_annotate, text,
									 state.camera_parameters.annotate_text_size,
									 state.camera_parameters.annotate_text_colour,
									 state.camera_parameters.annotate_bg_colour,
									 state.camera_parameters.annotate_justify,
									 state.camera_parameters.annotate_x,
									 state.camera_parameters.annotate_y);

		free(text);
	}
	else
	{
		raspicamcontrol_set_annotate(state.camera_component, state.camera_parameters.enable_annotate, state.camera_parameters.annotate_string,
									 state.camera_parameters.annotate_text_size,
									 state.camera_parameters.annotate_text_colour,
									 state.camera_parameters.annotate_bg_colour,
									 state.camera_parameters.annotate_justify,
									 state.camera_parameters.annotate_x,
									 state.camera_parameters.annotate_y);
	}
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_BUFFER_HEADER_T *new_buffer;
	static int64_t base_time = -1;
	static int64_t last_second = -1;

	// All our segment times based on the receipt of the first encoder callback
	if (base_time == -1)
		base_time = get_microseconds64() / 1000;

	// We pass our file handle and other stuff in via the userdata field.

	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{
		int bytes_written = buffer->length;
		int64_t current_time = get_microseconds64() / 1000;

		vcos_assert(pData->file_handle);

		// For segmented record mode, we need to see if we have exceeded our time/size,
		// but also since we have inline headers turned on we need to break when we get one to
		// ensure that the new stream has the header in it. If we break on an I-frame, the
		// SPS/PPS header is actually in the previous chunk.
		if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
			((pData->pstate->segmentSize && current_time > base_time + pData->pstate->segmentSize) ||
			 (pData->pstate->splitWait && pData->pstate->splitNow)))
		{
			FILE *new_handle;

			base_time = current_time;

			pData->pstate->splitNow = 0;
			pData->pstate->segmentNumber++;

			// Only wrap if we have a wrap point set
			if (pData->pstate->segmentWrap && pData->pstate->segmentNumber > pData->pstate->segmentWrap)
				pData->pstate->segmentNumber = 1;

			if (pData->pstate->common_settings.filename && pData->pstate->common_settings.filename[0] != '-')
			{
				new_handle = open_filename(pData->pstate, pData->pstate->common_settings.filename);

				if (new_handle)
				{
					fclose(pData->file_handle);
					pData->file_handle = new_handle;
				}
			}
		}
		if (buffer->length)
		{
			mmal_buffer_header_mem_lock(buffer);

			bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
			if (pData->flush_buffers)
			{
				fflush(pData->file_handle);
				fdatasync(fileno(pData->file_handle));
			}

			mmal_buffer_header_mem_unlock(buffer);

			if (bytes_written != buffer->length)
			{
				vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
				pData->abort = 1;
			}
		}

		// See if the second count has changed and we need to update any annotation
		if (current_time / 1000 != last_second)
		{
			update_annotation_data(pData->pstate);
			last_second = current_time / 1000;
		}
	}
	else
	{
		vcos_log_error("Received a encoder buffer callback with no state");
	}

	// release buffer back to the pool
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled)
	{
		MMAL_STATUS_T status;

		new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the encoder port");
	}
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component()
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
	MMAL_STATUS_T status;

	/* Create the component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create camera component");
		goto error;
	}

	status = raspicamcontrol_set_stereo_mode(camera->output[0], &(state.camera_parameters.stereo_mode));
	status += raspicamcontrol_set_stereo_mode(camera->output[1], &(state.camera_parameters.stereo_mode));
	status += raspicamcontrol_set_stereo_mode(camera->output[2], &(state.camera_parameters.stereo_mode));

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set stereo mode : error %d", status);
		goto error;
	}

	MMAL_PARAMETER_INT32_T camera_num =
		{{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state.common_settings.cameraNum};

	status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not select camera : error %d", status);
		goto error;
	}

	if (!camera->output_num)
	{
		status = MMAL_ENOSYS;
		vcos_log_error("Camera doesn't have output ports");
		goto error;
	}

	status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state.common_settings.sensor_mode);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Could not set sensor mode : error %d", status);
		goto error;
	}

	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	// Enable the camera, and tell it its control callback function
	status = mmal_port_enable(camera->control, default_camera_control_callback);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to enable control port : error %d", status);
		goto error;
	}

	//  set up the camera configuration
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
			{
				{MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
				.max_stills_w = state.common_settings.width,
				.max_stills_h = state.common_settings.height,
				.stills_yuv422 = 0,
				.one_shot_stills = 0,
				.max_preview_video_w = state.common_settings.width,
				.max_preview_video_h = state.common_settings.height,
				.num_preview_video_frames = 3 + vcos_max(0, (state.framerate - 30) / 10),
				.stills_capture_circular_buffer_height = 0,
				.fast_preview_resume = 0,
				.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}

	// Now set up the port formats

	// Set the encode format on the Preview port
	// HW limitations mean we need the preview to be the same size as the required recorded output

	format = preview_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	if (state.camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
												{5, 1000},
												{166, 1000}};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}
	else if (state.camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
												{166, 1000},
												{999, 1000}};
		mmal_port_parameter_set(preview_port, &fps_range.hdr);
	}

	// enable dynamic framerate if necessary
	if (state.camera_parameters.shutter_speed)
	{
		if (state.framerate > 1000000. / state.camera_parameters.shutter_speed)
		{
			state.framerate = 0;
			if (state.common_settings.verbose)
				fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
		}
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(state.common_settings.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state.common_settings.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state.common_settings.width;
	format->es->video.crop.height = state.common_settings.height;
	format->es->video.frame_rate.num = state.framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(preview_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera viewfinder format couldn't be set");
		goto error;
	}

	// Set the encode format on the video  port

	format = video_port->format;
	format->encoding_variant = MMAL_ENCODING_I420;

	if (state.camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
												{5, 1000},
												{166, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}
	else if (state.camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
												{167, 1000},
												{999, 1000}};
		mmal_port_parameter_set(video_port, &fps_range.hdr);
	}

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->es->video.width = VCOS_ALIGN_UP(state.common_settings.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state.common_settings.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state.common_settings.width;
	format->es->video.crop.height = state.common_settings.height;
	format->es->video.frame_rate.num = state.framerate;
	format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

	status = mmal_port_format_commit(video_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera video format couldn't be set");
		goto error;
	}

	// Ensure there are enough buffers to avoid dropping frames
	if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	// Set the encode format on the still  port

	format = still_port->format;

	format->encoding = MMAL_ENCODING_OPAQUE;
	format->encoding_variant = MMAL_ENCODING_I420;

	format->es->video.width = VCOS_ALIGN_UP(state.common_settings.width, 32);
	format->es->video.height = VCOS_ALIGN_UP(state.common_settings.height, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = state.common_settings.width;
	format->es->video.crop.height = state.common_settings.height;
	format->es->video.frame_rate.num = 0;
	format->es->video.frame_rate.den = 1;

	status = mmal_port_format_commit(still_port);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera still format couldn't be set");
		goto error;
	}

	/* Ensure there are enough buffers to avoid dropping frames */
	if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
		still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

	/* Enable component */
	status = mmal_component_enable(camera);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("camera component couldn't be enabled");
		goto error;
	}

	// Note: this sets lots of parameters that were not individually addressed before.
	raspicamcontrol_set_all_parameters(camera, &(state.camera_parameters));

	state.camera_component = camera;

	update_annotation_data(state);

	if (state.common_settings.verbose)
		fprintf(stderr, "Camera component done\n");

	return status;

error:

	if (camera)
		mmal_component_destroy(camera);

	return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component()
{
	if (state.camera_component)
	{
		mmal_component_destroy(state.camera_component);
		state.camera_component = NULL;
	}
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component()
{
	MMAL_COMPONENT_T *encoder = 0;
	MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
	MMAL_STATUS_T status;
	MMAL_POOL_T *pool;

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to create video encoder component");
		goto error;
	}

	if (!encoder->input_num || !encoder->output_num)
	{
		status = MMAL_ENOSYS;
		vcos_log_error("Video encoder doesn't have input/output ports");
		goto error;
	}

	encoder_input = encoder->input[0];
	encoder_output = encoder->output[0];

	// We want same format on input and output
	mmal_format_copy(encoder_output->format, encoder_input->format);

	// Only supporting H264 at the moment
	encoder_output->format->encoding = state.encoding;

	if (state.encoding == MMAL_ENCODING_H264)
	{
		if (state.level == MMAL_VIDEO_LEVEL_H264_4)
		{
			if (state.bitrate > MAX_BITRATE_LEVEL4)
			{
				fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
				state.bitrate = MAX_BITRATE_LEVEL4;
			}
		}
		else
		{
			if (state.bitrate > MAX_BITRATE_LEVEL42)
			{
				fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
				state.bitrate = MAX_BITRATE_LEVEL42;
			}
		}
	}
	else if (state.encoding == MMAL_ENCODING_MJPEG)
	{
		if (state.bitrate > MAX_BITRATE_MJPEG)
		{
			fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
			state.bitrate = MAX_BITRATE_MJPEG;
		}
	}

	encoder_output->format->bitrate = state.bitrate;

	if (state.encoding == MMAL_ENCODING_H264)
		encoder_output->buffer_size = encoder_output->buffer_size_recommended;
	else
		encoder_output->buffer_size = 256 << 10;

	if (encoder_output->buffer_size < encoder_output->buffer_size_min)
		encoder_output->buffer_size = encoder_output->buffer_size_min;

	encoder_output->buffer_num = encoder_output->buffer_num_recommended;

	if (encoder_output->buffer_num < encoder_output->buffer_num_min)
		encoder_output->buffer_num = encoder_output->buffer_num_min;

	// We need to set the frame rate on output to 0, to ensure it gets
	// updated correctly from the input framerate when port connected
	encoder_output->format->es->video.frame_rate.num = 0;
	encoder_output->format->es->video.frame_rate.den = 1;

	// Commit the port changes to the output port
	status = mmal_port_format_commit(encoder_output);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set format on video encoder output port");
		goto error;
	}

	// Set the rate control parameter
	if (0)
	{
		MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set ratecontrol");
			goto error;
		}
	}

	if (state.encoding == MMAL_ENCODING_H264 &&
		state.intraperiod != -1)
	{
		MMAL_PARAMETER_UINT32_T param = {{MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state.intraperiod};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set intraperiod");
			goto error;
		}
	}

	if (state.encoding == MMAL_ENCODING_H264 && state.slices > 1 && state.common_settings.width <= 1280)
	{
		int frame_mb_rows = VCOS_ALIGN_UP(state.common_settings.height, 16) >> 4;

		if (state.slices > frame_mb_rows) // warn user if too many slices selected
		{
			fprintf(stderr, "H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state.slices, frame_mb_rows, frame_mb_rows);
			// Continue rather than abort..
		}
		int slice_row_mb = frame_mb_rows / state.slices;
		if (frame_mb_rows - state.slices * slice_row_mb)
			slice_row_mb++; // must round up to avoid extra slice if not evenly divided

		status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set number of slices");
			goto error;
		}
	}

	if (state.encoding == MMAL_ENCODING_H264 &&
		state.quantisationParameter)
	{
		MMAL_PARAMETER_UINT32_T param = {{MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set initial QP");
			goto error;
		}

		MMAL_PARAMETER_UINT32_T param2 = {{MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param2.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set min QP");
			goto error;
		}

		MMAL_PARAMETER_UINT32_T param3 = {{MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state.quantisationParameter};
		status = mmal_port_parameter_set(encoder_output, &param3.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set max QP");
			goto error;
		}
	}

	if (state.encoding == MMAL_ENCODING_H264)
	{
		MMAL_PARAMETER_VIDEO_PROFILE_T param;
		param.hdr.id = MMAL_PARAMETER_PROFILE;
		param.hdr.size = sizeof(param);

		param.profile[0].profile = state.profile;

		if ((VCOS_ALIGN_UP(state.common_settings.width, 16) >> 4) * (VCOS_ALIGN_UP(state.common_settings.height, 16) >> 4) * state.framerate > 245760)
		{
			if ((VCOS_ALIGN_UP(state.common_settings.width, 16) >> 4) * (VCOS_ALIGN_UP(state.common_settings.height, 16) >> 4) * state.framerate <= 522240)
			{
				fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
				state.level = MMAL_VIDEO_LEVEL_H264_42;
			}
			else
			{
				vcos_log_error("Too many macroblocks/s requested");
				status = MMAL_EINVAL;
				goto error;
			}
		}

		param.profile[0].level = state.level;

		status = mmal_port_parameter_set(encoder_output, &param.hdr);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Unable to set H264 profile");
			goto error;
		}
	}

	if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state.immutableInput) != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to set immutable input flag");
		// Continue rather than abort..
	}

	if (state.encoding == MMAL_ENCODING_H264)
	{
		// set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state.bInlineHeaders) != MMAL_SUCCESS)
		{
			vcos_log_error("failed to set INLINE HEADER FLAG parameters");
			// Continue rather than abort..
		}

		// set flag for add SPS TIMING
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state.addSPSTiming) != MMAL_SUCCESS)
		{
			vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
			// Continue rather than abort..
		}

		// set INLINE VECTORS flag to request motion vector estimates
		if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state.inlineMotionVectors) != MMAL_SUCCESS)
		{
			vcos_log_error("failed to set INLINE VECTORS parameters");
			// Continue rather than abort..
		}

		// Adaptive intra refresh settings
		if (state.intra_refresh_type != -1)
		{
			MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T param;
			param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
			param.hdr.size = sizeof(param);

			// Get first so we don't overwrite anything unexpectedly
			status = mmal_port_parameter_get(encoder_output, &param.hdr);
			if (status != MMAL_SUCCESS)
			{
				vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
				// Set some defaults, don't just pass random stack data
				param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
			}

			param.refresh_mode = state.intra_refresh_type;

			// if (state.intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
			//    param.cir_mbs = 10;

			status = mmal_port_parameter_set(encoder_output, &param.hdr);
			if (status != MMAL_SUCCESS)
			{
				vcos_log_error("Unable to set H264 intra-refresh values");
				goto error;
			}
		}
	}

	//  Enable component
	status = mmal_component_enable(encoder);

	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Unable to enable video encoder component");
		goto error;
	}

	/* Create pool of buffer headers for the output port to consume */
	pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

	if (!pool)
	{
		vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
	}

	state.encoder_pool = pool;
	state.encoder_component = encoder;

	if (state.common_settings.verbose)
		fprintf(stderr, "Encoder component done\n");

	return status;

error:
	if (encoder)
		mmal_component_destroy(encoder);

	state.encoder_component = NULL;

	return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component()
{
	// Get rid of any port buffers first
	if (state.encoder_pool)
	{
		mmal_port_pool_destroy(state.encoder_component->output[0], state.encoder_pool);
	}

	if (state.encoder_component)
	{
		mmal_component_destroy(state.encoder_component);
		state.encoder_component = NULL;
	}
}

/**
 * Pause for specified time, but return early if detect an abort request
 *
 * @param state Pointer to state control struct
 * @param pause Time in ms to pause
 * @param callback Struct contain an abort flag tested for early termination
 *
 */
static int pause_and_test_abort(int pause)
{
	int wait;

	if (!pause)
		return 0;

	// Going to check every ABORT_INTERVAL milliseconds
	for (wait = 0; wait < pause; wait += ABORT_INTERVAL)
	{
		vcos_sleep(ABORT_INTERVAL);
		if (state.callback_data.abort)
			return 1;
	}

	return 0;
}

int raspi_vid_init(struct raspi_vid_cfg_t raspi_vid_cfg)
{
	int exit_code = EX_OK;
	MMAL_STATUS_T mmal_status = MMAL_SUCCESS;

	bcm_host_init();
	default_status();
	if (raspi_vid_cfg.raspi_vid_width > 0)
	{
		state.common_settings.width = raspi_vid_cfg.raspi_vid_width;
	}
	if (raspi_vid_cfg.raspi_vid_height > 0)
	{
		state.common_settings.height = raspi_vid_cfg.raspi_vid_height;
	}
	if (raspi_vid_cfg.raspi_vid_bitrate > 0)
	{
		state.bitrate = raspi_vid_cfg.raspi_vid_bitrate;
	}
	if (raspi_vid_cfg.raspi_vid_fps > 0)
	{
		state.framerate = raspi_vid_cfg.raspi_vid_fps;
	}
	if (raspi_vid_cfg.raspi_vid_gop > 0)
	{
		state.intraperiod = raspi_vid_cfg.raspi_vid_gop;
	}

	if ((mmal_status = create_camera_component()) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to create camera component", __func__);
		exit_code = EX_SOFTWARE;
	}
	else if ((mmal_status = create_encoder_component()) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to create encode component", __func__);
		raspipreview_destroy(&(state.preview_parameters));
		destroy_camera_component();
		exit_code = EX_SOFTWARE;
	}
	else
	{
		camera_video_port = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
		encoder_input_port = state.encoder_component->input[0];
		encoder_output_port = state.encoder_component->output[0];

		mmal_status = connect_ports(camera_video_port, encoder_input_port, &(state.encoder_connection));
		if (mmal_status != MMAL_SUCCESS)
		{
			state.encoder_connection = NULL;
			vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
			goto error;
		}

		state.callback_data.pstate = &state;
		state.callback_data.abort = 0;
		state.callback_data.file_handle = stdout;
		encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&(state.callback_data);

		mmal_status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);
		if (mmal_status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to setup encoder output");
			goto error;
		}

		int num = mmal_queue_length(state.encoder_pool->queue);
		int q;
		for (q = 0; q < num; q++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

			if (!buffer)
				vcos_log_error("Unable to get a required buffer %d from pool queue", q);

			if (mmal_port_send_buffer(encoder_output_port, buffer) != MMAL_SUCCESS)
				vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}

		state.bCapturing = !state.bCapturing;
		// 开始捕获视频, 非阻塞
		mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, state.bCapturing);

		while (1)
		{
			sleep(1);
		}

	error:
		mmal_status_to_int(mmal_status);

		// Disable all our ports that are not handled by connections
		check_disable_port(encoder_output_port);

		if (state.encoder_connection)
			mmal_connection_destroy(state.encoder_connection);

		/* Disable components */
		if (state.encoder_component)
			mmal_component_disable(state.encoder_component);

		if (state.camera_component)
			mmal_component_disable(state.camera_component);

		destroy_encoder_component();
		destroy_camera_component();
	}

	if (mmal_status != MMAL_SUCCESS)
		raspicamcontrol_check_configuration(128);

	return exit_code;
}
