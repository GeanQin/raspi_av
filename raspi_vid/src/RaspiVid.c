#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sysexits.h>

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
#include "RaspiHelpers.h"
#include "RaspiVid.h"

#define MMAL_CAMERA_VIDEO_PORT 1

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
	int immutableInput;								/// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
	/// the camera output or the encoder output (with compression artifacts)
	int profile;	/// H264 profile to use for encoding
	int level;		/// H264 level to use for encoding

	int onTime;	 /// In timed cycle mode, the amount of time the capture is on per cycle
	int offTime; /// In timed cycle mode, the amount of time the capture is off per cycle

	int segmentSize;   /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
	int segmentWrap;   /// Point at which to wrap segment counter
	int segmentNumber; /// Current segment counter
	int splitNow;	   /// Split at next possible i-frame if set to 1.
	int splitWait;	   /// Switch if user wants splited files

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
	state.common_settings.filename = "-";
	state.common_settings.width = 1920; // Default to 1080p
	state.common_settings.height = 1080;
	state.encoding = MMAL_ENCODING_H264;
	state.bitrate = 17000000; // This is a decent default bitrate for 1080p
	state.framerate = VIDEO_FRAME_RATE_NUM;
	state.intraperiod = -1; // Not set
	state.quantisationParameter = 0;
	state.immutableInput = 1;
	state.profile = MMAL_VIDEO_PROFILE_H264_HIGH;
	state.level = MMAL_VIDEO_LEVEL_H264_4;
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

	// Set up the camera_parameters to default
	raspicamcontrol_set_defaults(&(state.camera_parameters));
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
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

	if (pData)
	{
		int bytes_written = buffer->length;
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
	MMAL_PORT_T *video_port = NULL;
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

	video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

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
