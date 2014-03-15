/*
 * Copyright (C) 2011 The Android Open Source Project
 * Copyright (C) 2012 Wolfson Microelectronics plc
 * Copyright (C) 2013 The CyanogenMod Project
 *               Daniel Hillenbrand <codeworkx@cyanogenmod.com>
 *               Guillaume "XpLoDWilD" Lesniak <xplodgui@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_hw_primary"
#define LOG_NDEBUG 0

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <dlfcn.h>

#include <cutils/log.h>
#include <cutils/properties.h>
#include <cutils/str_parms.h>

#include <hardware/audio.h>
#include <hardware/hardware.h>

#include <system/audio.h>

#include <tinyalsa/asoundlib.h>

#include <audio_utils/resampler.h>
#include <audio_utils/echo_reference.h>
#include <hardware/audio_effect.h>
#include <audio_effects/effect_aec.h>
#include <audio_route/audio_route.h>

#include "routing.h"

#include "ril_interface.h"

#define PCM_CARD 0

#define PCM_DEVICE 0
#define PCM_DEVICE_VOICE 1
#define PCM_DEVICE_SCO 2
#define PCM_DEVICE_IN 3

#define MIXER_CARD 0

#define CAPTURE_START_RAMP_MS 8

#define MAX_SUPPORTED_CHANNEL_MASKS 1

#define ARRAY_SIZE(a) (sizeof((a)) / sizeof((a[0])))

#define XML_FILE_PATH "/system/etc/sound/m0"
#define DEFAULT_OUT_SAMPLING_RATE 44100

/* minimum sleep time in out_write() when write threshold is not reached */
#define MIN_WRITE_SLEEP_US 5000

struct pcm_config pcm_config_fast = { //pcm_config_tones
    .channels = 2,
    .rate = 44100,
    .period_size = 192, //SHORT_PERIOD_SIZE
    .period_count = 2,  //PLAYBACK_SHORT_PERIOD_COUNT
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_deep_long = { //pcm_config_mm
    .channels = 2,
    .rate = 44100,
    .period_size = 880, //DEEP_BUFFER_LONG_PERIOD_SIZE
    .period_count = 8,  //PLAYBACK_DEEP_BUFFER_LONG_PERIOD_COUNT
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_deep_short = {
    .channels = 2,
    .rate = 44100,
    .period_size = 1056, //DEEP_BUFFER_SHORT_PERIOD_SIZE
    .period_count = 4,   //PLAYBACK_DEEP_BUFFER_SHORT_PERIOD_COUNT
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_in = { //pcm_config_capture
    .channels = 2,
    .rate = 44100,
    .period_size = 1024, //CAPTURE_PERIOD_SIZE
    .period_count = 4,   //CAPTURE_PERIOD_COUNT
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_sco = {
    .channels = 1,
    .rate = 8000,
    .period_size = 128,
    .period_count = 2,
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_sco_wide = {
    .channels = 1,
    .rate = 16000,
    .period_size = 128,
    .period_count = 2,
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_voice = { //pcm_config_vx
    .channels = 2,
    .rate = 8000, //VX_NB_SAMPLING_RATE
    .period_size = 160,
    .period_count = 2,
    .format = PCM_FORMAT_S16_LE,
};

struct pcm_config pcm_config_voice_wide = {
    .channels = 2,
    .rate = 16000,
    .period_size = 160,
    .period_count = 2,
    .format = PCM_FORMAT_S16_LE,
};

enum pcm_type {
    PCM_NORMAL = 0,
    PCM_SPDIF,
    PCM_HDMI,
    PCM_TOTAL,
};

enum output_type {
    OUTPUT_DEEP_BUF,
    OUTPUT_LOW_LATENCY,
    OUTPUT_TOTAL
};

struct m0_audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock;       /* see note below on mutex acquisition order */
    audio_devices_t out_device;
    audio_devices_t in_device;
    bool mic_mute;
    struct audio_route *ar;
    audio_source_t input_source;
    int cur_route_id;     /* current route ID: combination of input source
                           * and output device IDs */
    audio_mode_t mode;

    audio_channel_mask_t in_channel_mask;

    /* Call audio */
    struct pcm *pcm_voice_rx;
    struct pcm *pcm_voice_tx;

    /* SCO audio */
    struct pcm *pcm_sco_rx;
    struct pcm *pcm_sco_tx;

    float voice_volume;
    struct m0_stream_in *active_input;
    struct echo_reference_itfe *echo_reference;
    bool in_call;
    bool tty_mode;
    bool bluetooth_nrec;
    bool wb_amr;
    bool screen_off;

    /* RIL */
    struct ril_handle ril;

    struct m0_stream_out *outputs[OUTPUT_TOTAL];
};

struct m0_stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock;       /* see note below on mutex acquisition order */
    struct pcm *pcm[PCM_TOTAL];
    struct pcm_config config;
    enum output_type output_type;
    unsigned int pcm_device;
    bool standby;
    audio_devices_t device;

    struct resampler_itfe *resampler;
    struct echo_reference_itfe *echo_reference;
    int16_t *buffer;
    size_t buffer_frames;
    int write_threshold;
    bool use_long_periods;

    audio_channel_mask_t channel_mask;
    /* Array of supported channel mask configurations. +1 so that the last entry is always 0 */
    audio_channel_mask_t supported_channel_masks[MAX_SUPPORTED_CHANNEL_MASKS + 1];

    struct m0_audio_device *dev;
};

#define MAX_PREPROCESSORS 3 /* maximum one AGC + one NS + one AEC per input stream */

struct effect_info_s {
    effect_handle_t effect_itfe;
    size_t num_channel_configs;
    channel_config_t* channel_configs;
};

#define NUM_IN_AUX_CNL_CONFIGS 2
channel_config_t in_aux_cnl_configs[NUM_IN_AUX_CNL_CONFIGS] = {
    { AUDIO_CHANNEL_IN_FRONT , AUDIO_CHANNEL_IN_BACK},
    { AUDIO_CHANNEL_IN_STEREO , AUDIO_CHANNEL_IN_RIGHT}
};

struct m0_stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock;       /* see note below on mutex acquisition order */
    struct pcm_config config;
    struct pcm *pcm;
    bool standby;

    int16_t *read_buf;
    size_t read_buf_size;
    size_t read_buf_frames;

    int16_t *proc_buf_in;
    int16_t *proc_buf_out;
    size_t proc_buf_size;
    size_t proc_buf_frames;

    int16_t *ref_buf;
    size_t ref_buf_size;
    size_t ref_buf_frames;

    unsigned int requested_rate;
    struct resampler_itfe *resampler;
    struct resampler_buffer_provider buf_provider;
    struct echo_reference_itfe *echo_reference;
    bool need_echo_reference;
    int16_t *buffer;
    size_t frames_in;
    int read_status;

    audio_source_t input_source;
    audio_io_handle_t io_handle;
    audio_devices_t device;

    uint16_t ramp_vol;
    uint16_t ramp_step;
    uint16_t ramp_frames;

    int num_preprocessors;
    struct effect_info_s preprocessors[MAX_PREPROCESSORS];
    audio_channel_mask_t channel_mask;

    bool aux_channels_changed;
    uint32_t main_channels;
    uint32_t aux_channels;

    struct m0_audio_device *dev;
};

#define STRING_TO_ENUM(string) { #string, string }

struct string_to_enum {
    const char *name;
    uint32_t value;
};

const struct string_to_enum out_channels_name_to_enum_table[] = {
    STRING_TO_ENUM(AUDIO_CHANNEL_OUT_STEREO),
    STRING_TO_ENUM(AUDIO_CHANNEL_OUT_5POINT1),
    STRING_TO_ENUM(AUDIO_CHANNEL_OUT_7POINT1),
};

/* Routing functions */

static int get_output_device_id(audio_devices_t device)
{
	ALOGV("%s: device=%d popcount(device)=%d", __func__, device, popcount(device));
    if (device == AUDIO_DEVICE_NONE)
        return OUT_DEVICE_NONE;

    if (popcount(device) == 2) {
        if ((device == (AUDIO_DEVICE_OUT_SPEAKER |
                        AUDIO_DEVICE_OUT_WIRED_HEADSET)) ||
                (device == (AUDIO_DEVICE_OUT_SPEAKER |
                        AUDIO_DEVICE_OUT_WIRED_HEADPHONE)))
            return OUT_DEVICE_SPEAKER_AND_HEADSET;
        else if (device == (AUDIO_DEVICE_OUT_SPEAKER |
                        AUDIO_DEVICE_OUT_EARPIECE))
            return OUT_DEVICE_SPEAKER_AND_EARPIECE;
        else
            return OUT_DEVICE_NONE;
    }

    if (popcount(device) != 1)
        return OUT_DEVICE_NONE;

    switch (device) {
    case AUDIO_DEVICE_OUT_SPEAKER:
        return OUT_DEVICE_SPEAKER;
    case AUDIO_DEVICE_OUT_EARPIECE:
        return OUT_DEVICE_EARPIECE;
    case AUDIO_DEVICE_OUT_WIRED_HEADSET:
        return OUT_DEVICE_HEADSET;
    case AUDIO_DEVICE_OUT_WIRED_HEADPHONE:
        return OUT_DEVICE_HEADPHONES;
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET:
    case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT:
        return OUT_DEVICE_BT_SCO;
    default:
        return OUT_DEVICE_NONE;
    }
}

static int get_input_source_id(audio_source_t source)
{
	ALOGV("%s: source=%d", __func__, source);
    switch (source) {
    case AUDIO_SOURCE_DEFAULT:
        return IN_SOURCE_NONE;
    case AUDIO_SOURCE_MIC:
        return IN_SOURCE_MIC;
    case AUDIO_SOURCE_CAMCORDER:
        return IN_SOURCE_CAMCORDER;
    case AUDIO_SOURCE_VOICE_RECOGNITION:
        return IN_SOURCE_VOICE_RECOGNITION;
    case AUDIO_SOURCE_VOICE_COMMUNICATION:
        return IN_SOURCE_VOICE_COMMUNICATION;
    case AUDIO_SOURCE_VOICE_CALL:
        return IN_SOURCE_VOICE_CALL;
    default:
        return IN_SOURCE_NONE;
    }
}

static void adev_set_call_audio_path(struct m0_audio_device *adev);

/*
 * NOTE: when multiple mutexes have to be acquired, always take the
 * audio_device mutex first, followed by the stream_in and/or
 * stream_out mutexes.
 */

static void select_devices(struct m0_audio_device *adev)
{
    int output_device_id = get_output_device_id(adev->out_device);
    int input_source_id = get_input_source_id(adev->input_source);
    const char *output_route = NULL;
    const char *input_route = NULL;
    int new_route_id;

    audio_route_reset(adev->ar);

    new_route_id = (1 << (input_source_id + OUT_DEVICE_CNT)) + (1 << output_device_id);
    ALOGV("%s: output_device_id=%d input_source_id=%d new_route_id=%d", __func__, output_device_id, input_source_id, new_route_id);

    if (new_route_id == adev->cur_route_id) {
    	ALOGV("%s nothing to do", __func__);
        return;
    }
    adev->cur_route_id = new_route_id;

    if (input_source_id != IN_SOURCE_NONE) {
        if (output_device_id != OUT_DEVICE_NONE) {
            input_route =
                route_configs[input_source_id][output_device_id]->input_route;
            output_route =
                route_configs[input_source_id][output_device_id]->output_route;
        } else {
            switch(adev->in_device) {
            case AUDIO_DEVICE_IN_WIRED_HEADSET & ~AUDIO_DEVICE_BIT_IN:
                output_device_id = OUT_DEVICE_HEADSET;
                break;
            case AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET & ~AUDIO_DEVICE_BIT_IN:
                output_device_id = OUT_DEVICE_BT_SCO;
                break;
            default:
                output_device_id = OUT_DEVICE_SPEAKER;
                break;
            }
            input_route =
                route_configs[input_source_id][output_device_id]->input_route;
        }
    } else {
        if (output_device_id != OUT_DEVICE_NONE) {
            output_route =
                route_configs[IN_SOURCE_MIC][output_device_id]->output_route;
        }
    }
    ALOGV("select_devices() out_device '%#x' input_src '%d' output_route '%s' input_route '%s'",
          adev->out_device, adev->input_source,
          output_route ? output_route : "none",
          input_route ? input_route : "none");

    if (output_route)
        audio_route_apply_path(adev->ar, output_route);
    if (input_route)
        audio_route_apply_path(adev->ar, input_route);

    audio_route_update_mixer(adev->ar);

    adev_set_call_audio_path(adev);
}

/* BT SCO functions */

/* must be called with hw device mutex locked, OK to hold other mutexes */
static void start_bt_sco(struct m0_audio_device *adev)
{
    struct pcm_config *sco_config;

    if (adev->pcm_sco_rx || adev->pcm_sco_tx) {
        ALOGW("%s: SCO PCMs already open!\n", __func__);
        return;
    }

    ALOGV("%s: Opening SCO PCMs", __func__);

    if (adev->wb_amr)
        sco_config = &pcm_config_sco_wide;
    else
        sco_config = &pcm_config_sco;

    adev->pcm_sco_rx = pcm_open(PCM_CARD, PCM_DEVICE_SCO, PCM_OUT,
            sco_config);
    if (adev->pcm_sco_rx && !pcm_is_ready(adev->pcm_sco_rx)) {
        ALOGE("%s: cannot open PCM SCO RX stream: %s",
              __func__, pcm_get_error(adev->pcm_sco_rx));
        goto err_sco_rx;
    }

    adev->pcm_sco_tx = pcm_open(PCM_CARD, PCM_DEVICE_SCO, PCM_IN,
            sco_config);
    if (adev->pcm_sco_tx && !pcm_is_ready(adev->pcm_sco_tx)) {
        ALOGE("%s: cannot open PCM SCO TX stream: %s",
              __func__, pcm_get_error(adev->pcm_sco_tx));
        goto err_sco_tx;
    }

    pcm_start(adev->pcm_sco_rx);
    pcm_start(adev->pcm_sco_tx);

    return;

err_sco_tx:
    pcm_close(adev->pcm_sco_tx);
err_sco_rx:
    pcm_close(adev->pcm_sco_rx);
}

/* must be called with hw device mutex locked, OK to hold other mutexes */
static void end_bt_sco(struct m0_audio_device *adev)
{
    ALOGV("%s: Closing SCO PCMs", __func__);

    if (adev->pcm_sco_rx) {
        pcm_stop(adev->pcm_sco_rx);
        pcm_close(adev->pcm_sco_rx);
        adev->pcm_sco_rx = NULL;
    }

    if (adev->pcm_sco_tx) {
        pcm_stop(adev->pcm_sco_tx);
        pcm_close(adev->pcm_sco_tx);
        adev->pcm_sco_tx = NULL;
    }
}

/* Samsung RIL functions */

/* must be called with hw device mutex locked, OK to hold other mutexes */
static int start_voice_call(struct m0_audio_device *adev)
{
    struct pcm_config *voice_config;

    if (adev->pcm_voice_rx || adev->pcm_voice_tx) {
        ALOGW("%s: Voice PCMs already open!\n", __func__);
        return 0;
    }

    ALOGV("%s: Opening voice PCMs", __func__);

    if (adev->wb_amr)
        voice_config = &pcm_config_voice_wide;
    else
        voice_config = &pcm_config_voice;

    /* Open modem PCM channels */
    adev->pcm_voice_rx = pcm_open(PCM_CARD, PCM_DEVICE_VOICE, PCM_OUT,
            voice_config);
    if (adev->pcm_voice_rx && !pcm_is_ready(adev->pcm_voice_rx)) {
        ALOGE("%s: cannot open PCM voice RX stream: %s",
              __func__, pcm_get_error(adev->pcm_voice_rx));
        goto err_voice_rx;
    }

    adev->pcm_voice_tx = pcm_open(PCM_CARD, PCM_DEVICE_VOICE, PCM_IN,
            voice_config);
    if (adev->pcm_voice_tx && !pcm_is_ready(adev->pcm_voice_tx)) {
        ALOGE("%s: cannot open PCM voice TX stream: %s",
              __func__, pcm_get_error(adev->pcm_voice_tx));
        goto err_voice_tx;
    }

    pcm_start(adev->pcm_voice_rx);
    pcm_start(adev->pcm_voice_tx);

    return 0;

err_voice_tx:
    pcm_close(adev->pcm_voice_tx);
    adev->pcm_voice_tx = NULL;
err_voice_rx:
    pcm_close(adev->pcm_voice_rx);
    adev->pcm_voice_rx = NULL;

    return -ENOMEM;
}

/* must be called with hw device mutex locked, OK to hold other mutexes */
static void end_voice_call(struct m0_audio_device *adev)
{
    ALOGV("%s: Closing voice PCMs", __func__);

    if (adev->pcm_voice_rx) {
        pcm_stop(adev->pcm_voice_rx);
        pcm_close(adev->pcm_voice_rx);
        adev->pcm_voice_rx = NULL;
    }

    if (adev->pcm_voice_tx) {
        pcm_stop(adev->pcm_voice_tx);
        pcm_close(adev->pcm_voice_tx);
        adev->pcm_voice_tx = NULL;
    }
}

static void adev_set_wb_amr_callback(void *data, int enable)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)data;

    ALOGV("%s: setting to: %d", __func__, enable);

    pthread_mutex_lock(&adev->lock);
    if (adev->wb_amr != enable) {
        adev->wb_amr = enable;

        /* reopen the modem PCMs at the new rate */
        if (adev->in_call) {
#if 0
            /* TODO: set rate properly */
            end_voice_call(adev);
            select_devices(adev);
            start_voice_call(adev);
#endif
        }
    }
    pthread_mutex_unlock(&adev->lock);
}

static void adev_set_call_audio_path(struct m0_audio_device *adev)
{
    enum ril_audio_path device_type;

    switch(adev->out_device) {
        case AUDIO_DEVICE_OUT_SPEAKER:
            device_type = SOUND_AUDIO_PATH_SPEAKER;
            break;
        case AUDIO_DEVICE_OUT_EARPIECE:
            device_type = SOUND_AUDIO_PATH_HANDSET;
            break;
        case AUDIO_DEVICE_OUT_WIRED_HEADSET:
            device_type = SOUND_AUDIO_PATH_HEADSET;
            break;
        case AUDIO_DEVICE_OUT_WIRED_HEADPHONE:
            device_type = SOUND_AUDIO_PATH_HEADPHONE;
            break;
        case AUDIO_DEVICE_OUT_BLUETOOTH_SCO:
        case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET:
        case AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT:
            if (adev->bluetooth_nrec) {
                device_type = SOUND_AUDIO_PATH_BLUETOOTH;
            } else {
                device_type = SOUND_AUDIO_PATH_BLUETOOTH_NO_NR;
            }
            break;
        default:
            /* if output device isn't supported, use handset by default */
            device_type = SOUND_AUDIO_PATH_HANDSET;
            break;
    }

    ALOGV("%s: ril_set_call_audio_path(%d)", __func__, device_type);
    ril_set_call_audio_path(&adev->ril, device_type);
}

/* Helper functions */

static int start_output_stream(struct m0_stream_out *out)
{
    struct m0_audio_device *adev = out->dev;

    ALOGV("%s: starting stream", __func__);

    out->pcm[PCM_CARD] = pcm_open(PCM_CARD, out->pcm_device,
                                  PCM_OUT, &out->config);
    if (out->pcm[PCM_CARD] && !pcm_is_ready(out->pcm[PCM_CARD])) {
        ALOGE("pcm_open(PCM_CARD) failed: %s",
                pcm_get_error(out->pcm[PCM_CARD]));
        pcm_close(out->pcm[PCM_CARD]);
        return -ENOMEM;
    }

    /* in call routing must go through set_parameters */
    if (!adev->in_call) {
        adev->out_device |= out->device;
        select_devices(adev);
    }

    if (out->device & AUDIO_DEVICE_OUT_ALL_SCO)
        start_bt_sco(adev);

    ALOGV("%s: stream out device: %d, actual: %d",
          __func__, out->device, adev->out_device);

    return 0;
}

/* must be called with hw device and input stream mutexes locked */
static int start_input_stream(struct m0_stream_in *in)
{
    struct m0_audio_device *adev = in->dev;

    adev->active_input = in;

    in->pcm = pcm_open(PCM_CARD, PCM_DEVICE_IN, PCM_IN, &pcm_config_in);

    if (in->pcm && !pcm_is_ready(in->pcm)) {
        ALOGE("pcm_open() failed: %s", pcm_get_error(in->pcm));
        pcm_close(in->pcm);
        return -ENOMEM;
    }

    /* if no supported sample rate is available, use the resampler */
    if (in->resampler)
        in->resampler->reset(in->resampler);

    in->frames_in = 0;
    /* in call routing must go through set_parameters */
    if (!adev->in_call) {
        adev->input_source = in->input_source;
        adev->in_device = in->device;
        adev->in_channel_mask = in->channel_mask;

        select_devices(adev);
    }

    if (in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET)
        start_bt_sco(adev);

    /* initialize volume ramp */
    in->ramp_frames = (CAPTURE_START_RAMP_MS * in->requested_rate) / 1000;
    in->ramp_step = (uint16_t)(USHRT_MAX / in->ramp_frames);
    in->ramp_vol = 0;

    return 0;
}

static size_t get_input_buffer_size(unsigned int sample_rate,
                                    audio_format_t format,
                                    unsigned int channel_count)
{
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (pcm_config_in.period_size * sample_rate) / pcm_config_in.rate;
    size = ((size + 15) / 16) * 16;

    return size * channel_count * audio_bytes_per_sample(format);
}

static int get_next_buffer(struct resampler_buffer_provider *buffer_provider,
                                   struct resampler_buffer* buffer)
{
    struct m0_stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return -EINVAL;

    in = (struct m0_stream_in *)((char *)buffer_provider -
                                   offsetof(struct m0_stream_in, buf_provider));

    if (in->pcm == NULL) {
        buffer->raw = NULL;
        buffer->frame_count = 0;
        in->read_status = -ENODEV;
        return -ENODEV;
    }

    if (in->read_buf_frames == 0) {
        size_t size_in_bytes = pcm_frames_to_bytes(in->pcm, in->config.period_size);
        if (in->read_buf_size < in->config.period_size) {
            in->read_buf_size = in->config.period_size;
            in->read_buf = (int16_t *) realloc(in->read_buf, size_in_bytes);
            ALOG_ASSERT((in->read_buf != NULL),
                        "%s failed to reallocate read_buf", __func__);
            ALOGV("%s: read_buf %p extended to %d bytes",
                  __func__, in->read_buf, size_in_bytes);
        }

        in->read_status = pcm_read(in->pcm, (void*)in->read_buf, size_in_bytes);

        if (in->read_status != 0) {
            ALOGE("%s: pcm_read error %d", __func__, in->read_status);
            buffer->raw = NULL;
            buffer->frame_count = 0;
            return in->read_status;
        }
        in->read_buf_frames = in->config.period_size;
    }

    buffer->frame_count = (buffer->frame_count > in->read_buf_frames) ?
                                in->read_buf_frames : buffer->frame_count;
    buffer->i16 = in->read_buf + (in->config.period_size - in->read_buf_frames) *
                                                in->config.channels;

    return in->read_status;

}

static void release_buffer(struct resampler_buffer_provider *buffer_provider,
                                  struct resampler_buffer* buffer)
{
    struct m0_stream_in *in;

    if (buffer_provider == NULL || buffer == NULL)
        return;

    in = (struct m0_stream_in *)((char *)buffer_provider -
                                   offsetof(struct m0_stream_in, buf_provider));

    in->frames_in -= buffer->frame_count;
}

/* read_frames() reads frames from kernel driver, down samples to capture rate
 * if necessary and output the number of frames requested to the buffer specified */
static ssize_t read_frames(struct m0_stream_in *in, void *buffer, ssize_t frames)
{
    ssize_t frames_wr = 0;
    size_t frame_size = audio_stream_frame_size(&in->stream.common);

    while (frames_wr < frames) {
        size_t frames_rd = frames - frames_wr;
        if (in->resampler != NULL) {
            in->resampler->resample_from_provider(in->resampler,
                    (int16_t *)((char *)buffer +
                            frames_wr * frame_size),
                    &frames_rd);
        } else {
            struct resampler_buffer buf = {
                    { raw : NULL, },
                    frame_count : frames_rd,
            };
            get_next_buffer(&in->buf_provider, &buf);
            if (buf.raw != NULL) {
                memcpy((char *)buffer +
                           frames_wr * frame_size,
                        buf.raw,
                        buf.frame_count * frame_size);
                frames_rd = buf.frame_count;
            }
            release_buffer(&in->buf_provider, &buf);
        }
        /* in->read_status is updated by getNextBuffer() also called by
         * in->resampler->resample_from_provider() */
        if (in->read_status != 0)
            return in->read_status;

        frames_wr += frames_rd;
    }
    return frames_wr;
}

/* API functions */

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;

    return out->config.rate;
}

static int out_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return -ENOSYS;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;

    ALOGV("%s begin", __func__);
    /* take resampling into account and return the closest majoring
        multiple of 16 frames, as audioflinger expects audio buffers to
        be a multiple of 16 frames. Note: we use the default rate here
        from pcm_config_tones.rate. */
    /* TODO - Study this
        size_t size = (SHORT_PERIOD_SIZE * DEFAULT_OUT_SAMPLING_RATE) / pcm_config_tones.rate;
        size = ((size + 15) / 16) * 16;
        return size * audio_stream_frame_size((struct audio_stream *)stream); */

    ALOGV("%s end", __func__);

    return out->config.period_size *
            audio_stream_frame_size((struct audio_stream *)stream);
}

static audio_channel_mask_t out_get_channels(const struct audio_stream *stream)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;

    return out->channel_mask;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int out_set_format(struct audio_stream *stream, audio_format_t format)
{
    return -ENOSYS;
}

/* Return the set of output devices associated with active streams
 * other than out.  Assumes out is non-NULL and out->dev is locked.
 */
static audio_devices_t output_devices(struct m0_stream_out *out)
{
    struct m0_audio_device *dev = out->dev;
    enum output_type type;
    audio_devices_t devices = AUDIO_DEVICE_NONE;

    for (type = 0; type < OUTPUT_TOTAL; ++type) {
        struct m0_stream_out *other = dev->outputs[type];
        if (other && (other != out) && !other->standby) {
            /* safe to access other stream without a mutex,
             * because we hold the dev lock,
             * which prevents the other stream from being closed
             */
            devices |= other->device;
        }
    }

    return devices;
}

static int do_out_standby(struct m0_stream_out *out)
{
    struct m0_audio_device *adev = out->dev;
    int i;

    ALOGV("%s: output standby: %d", __func__, out->standby);

    if (!out->standby) {
        for (i = 0; i < PCM_TOTAL; i++) {
            if (out->pcm[i]) {
                pcm_close(out->pcm[i]);
                out->pcm[i] = NULL;
            }
        }
        out->standby = true;

#if 0
        if (out == adev->outputs[OUTPUT_HDMI]) {
            /* force standby on low latency output stream so that it can reuse HDMI driver if
            * necessary when restarted */
            force_non_hdmi_out_standby(adev);
        }
#endif

        if (out->device & AUDIO_DEVICE_OUT_ALL_SCO)
            end_bt_sco(adev);

        /* re-calculate the set of active devices from other streams */
        adev->out_device = output_devices(out);

        /* Skip resetting the mixer if no output device is active */
        if (adev->out_device)
            select_devices(adev);
    }
    return 0;
}

static int out_standby(struct audio_stream *stream)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;
    int ret;

    pthread_mutex_lock(&out->dev->lock);
    pthread_mutex_lock(&out->lock);

    ret = do_out_standby(out);

    pthread_mutex_unlock(&out->lock);
    pthread_mutex_unlock(&out->dev->lock);
    return ret;
}

static int out_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;
    struct m0_audio_device *adev = out->dev;
    //struct m0_stream_in *in;
    struct str_parms *parms;
    char value[32];
    int ret;
    unsigned int val;
    //bool force_input_standby = false;

    ALOGV("%s: key value pairs: %s", __func__, kvpairs);

    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        pthread_mutex_lock(&adev->lock);
        pthread_mutex_lock(&out->lock);
        if (((adev->out_device) != val) && (val != 0)) {
            ALOGV("%s adev->out_device=%d val=%d", __func__, adev->out_device, val);

            /* force output standby to stop SCO pcm stream if needed */
            if ((val & AUDIO_DEVICE_OUT_ALL_SCO) ^
                    (out->device & AUDIO_DEVICE_OUT_ALL_SCO)) {
                do_out_standby(out);
            }

            out->device = val;
            adev->out_device = val;
            select_devices(adev);

            /* start SCO stream if needed */
            if (val & AUDIO_DEVICE_OUT_ALL_SCO)
                start_bt_sco(adev);
        }
        pthread_mutex_unlock(&out->lock);
        pthread_mutex_unlock(&adev->lock);
    }

    str_parms_destroy(parms);
    return ret;
}

static char * out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str;
    char value[256];
    struct str_parms *reply = str_parms_create();
    size_t i, j;
    int ret;
    bool first = true;

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value, sizeof(value));
    if (ret >= 0) {
        value[0] = '\0';
        i = 0;
        /* the last entry in supported_channel_masks[] is always 0 */
        while (out->supported_channel_masks[i] != 0) {
            for (j = 0; j < ARRAY_SIZE(out_channels_name_to_enum_table); j++) {
                if (out_channels_name_to_enum_table[j].value == out->supported_channel_masks[i]) {
                    if (!first) {
                        strcat(value, "|");
                    }
                    strcat(value, out_channels_name_to_enum_table[j].name);
                    first = false;
                    break;
                }
            }
            i++;
        }
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value);
        str = strdup(str_parms_to_str(reply));
    } else {
        str = strdup(keys);
    }
    str_parms_destroy(query);
    str_parms_destroy(reply);
    return str;
}

static uint32_t out_get_latency(const struct audio_stream_out *stream)
{
    struct m0_stream_out *out = (struct m0_stream_out *)stream;

    ALOGV("%s", __func__);
    return (out->config.period_size * out->config.period_count * 1000) /
            out->config.rate;
}

static int out_set_volume(struct audio_stream_out *stream, float left,
                          float right)
{
    return -ENOSYS;
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    int ret;
    struct m0_stream_out *out = (struct m0_stream_out *)stream;
    struct m0_audio_device *adev = out->dev;
    size_t frame_size = audio_stream_frame_size(&out->stream.common);
    size_t in_frames = bytes / frame_size;
    size_t out_frames;
    bool use_long_periods;
    int kernel_frames;
    void *buf;
    //int i;

    ALOGV("%s", __func__);
    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the output stream mutex - e.g.
     * executing out_set_parameters() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&out->lock);
    if (out->standby) {
        ret = start_output_stream(out);
        if (ret != 0) {
            pthread_mutex_unlock(&adev->lock);
            goto exit;
        }
        out->standby = false;
    }

    /* if (flags & AUDIO_OUTPUT_FLAG_DEEP_BUFFER) { */
        /* We'll start with short period size, and during out_write() will check
         * whether to change to pcm_config_deep_long if screen is off */
     /* out->config = pcm_config_deep_short;
        out->pcm_device = PCM_DEVICE;
        output_type = OUTPUT_DEEP_BUF;
    } else {
        out->config = pcm_config_fast;
        out->pcm_device = PCM_DEVICE;
        output_type = OUTPUT_LOW_LATENCY;
    } */

    use_long_periods = (out->output_type == OUTPUT_DEEP_BUF) &&
                        adev->screen_off && !adev->active_input;

    pthread_mutex_unlock(&adev->lock);

    if (use_long_periods != out->use_long_periods) {
        size_t period_size;
        size_t period_count;

        if (use_long_periods) {
            period_size = pcm_config_deep_long.period_size; //DEEP_BUFFER_LONG_PERIOD_SIZE;
            period_count = pcm_config_deep_long.period_count; //PLAYBACK_DEEP_BUFFER_LONG_PERIOD_COUNT;
        } else {
            period_size = pcm_config_deep_short.period_size; //DEEP_BUFFER_SHORT_PERIOD_SIZE;
            period_count = pcm_config_deep_short.period_count; //PLAYBACK_DEEP_BUFFER_SHORT_PERIOD_COUNT;
        }
        out->write_threshold = period_size * period_count;
        pcm_set_avail_min(out->pcm[PCM_NORMAL], period_size);
        out->use_long_periods = use_long_periods;
    }

    /* only use resampler if required */
    //if (out->config[PCM_NORMAL].rate != DEFAULT_OUT_SAMPLING_RATE) {
    if (out->config.rate != DEFAULT_OUT_SAMPLING_RATE) {
        out_frames = out->buffer_frames;
        out->resampler->resample_from_input(out->resampler,
                                            (int16_t *)buffer,
                                            &in_frames,
                                            (int16_t *)out->buffer,
                                            &out_frames);
        buf = (void *)out->buffer;
    } else {
        out_frames = in_frames;
        buf = (void *)buffer;
    }

    /* do not allow more than out->write_threshold frames in kernel pcm driver buffer */
    do {
        struct timespec time_stamp;

        if (pcm_get_htimestamp(out->pcm[PCM_NORMAL],
                               (unsigned int *)&kernel_frames, &time_stamp) < 0)
            break;
        kernel_frames = pcm_get_buffer_size(out->pcm[PCM_NORMAL]) - kernel_frames;

        if (kernel_frames > out->write_threshold) {
            unsigned long time = (unsigned long)
                    (((int64_t)(kernel_frames - out->write_threshold) * 1000000) /
                    		DEFAULT_OUT_SAMPLING_RATE); //MM_FULL_POWER_SAMPLING_RATE
            if (time < MIN_WRITE_SLEEP_US)
                time = MIN_WRITE_SLEEP_US;
            usleep(time);
        }
    } while (kernel_frames > out->write_threshold);

    ret = pcm_mmap_write(out->pcm[PCM_NORMAL], buf, out_frames * frame_size);

exit:
    pthread_mutex_unlock(&out->lock);

    if (ret != 0) {
        usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
               out_get_sample_rate(&stream->common));
    }

    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    return -EINVAL;
}

static int out_add_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream, effect_handle_t effect)
{
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream,
                                        int64_t *timestamp)
{
    return -EINVAL;
}

/** audio_stream_in implementation **/
static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;

    return in->requested_rate;
}

static int in_set_sample_rate(struct audio_stream *stream, uint32_t rate)
{
    return 0;
}

static audio_channel_mask_t in_get_channels(const struct audio_stream *stream)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;

    return in->channel_mask;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;

    return get_input_buffer_size(in->requested_rate,
                                 AUDIO_FORMAT_PCM_16_BIT,
                                 popcount(in_get_channels(stream)));
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(struct audio_stream *stream, audio_format_t format)
{
    return -ENOSYS;
}

static int do_in_standby(struct m0_stream_in *in)
{
    struct m0_audio_device *adev = in->dev;

    if (!in->standby) {
        pcm_close(in->pcm);
        in->pcm = NULL;

        if (in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET)
            end_bt_sco(adev);

        in->dev->input_source = AUDIO_SOURCE_DEFAULT;
        in->dev->in_device = AUDIO_DEVICE_NONE;
        in->dev->in_channel_mask = 0;
        select_devices(adev);
        in->standby = true;
    }

    return 0;
}

static int in_standby(struct audio_stream *stream)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;
    int ret;

    pthread_mutex_lock(&in->dev->lock);
    pthread_mutex_lock(&in->lock);

    ret = do_in_standby(in);

    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&in->dev->lock);

    return ret;
}

static int in_dump(const struct audio_stream *stream, int fd)
{
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;
    struct m0_audio_device *adev = in->dev;
    struct str_parms *parms;
    char value[32];
    int ret;
    unsigned int val;
    bool apply_now = false;

    parms = str_parms_create_str(kvpairs);

    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_INPUT_SOURCE,
                            value, sizeof(value));
    if (ret >= 0) {
        val = atoi(value);
        /* no audio source uses val == 0 */
        if ((in->input_source != val) && (val != 0)) {
            in->input_source = val;
            apply_now = !in->standby;
        }
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    if (ret >= 0) {
        /* strip AUDIO_DEVICE_BIT_IN to allow bitwise comparisons */
        val = atoi(value) & ~AUDIO_DEVICE_BIT_IN;
        /* no audio device uses val == 0 */
        if ((in->device != val) && (val != 0)) {
            /* force output standby to start or stop SCO pcm stream if needed */
            if ((val & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) ^
                    (in->device & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET)) {
                do_in_standby(in);
            }
            in->device = val;
            apply_now = !in->standby;
        }
    }

    if (apply_now) {
        adev->input_source = in->input_source;
        adev->in_device = in->device;
        select_devices(adev);
    }

    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&adev->lock);

    str_parms_destroy(parms);
    return ret;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    return strdup("");
}

static int in_set_gain(struct audio_stream_in *stream, float gain)
{
    return 0;
}

static void get_capture_delay(struct m0_stream_in *in,
                       size_t frames,
                       struct echo_reference_buffer *buffer)
{

    /* read frames available in kernel driver buffer */
    size_t kernel_frames;
    struct timespec tstamp;
    long buf_delay;
    long rsmp_delay;
    long kernel_delay;
    long delay_ns;

    if (pcm_get_htimestamp(in->pcm, &kernel_frames, &tstamp) < 0) {
        buffer->time_stamp.tv_sec  = 0;
        buffer->time_stamp.tv_nsec = 0;
        buffer->delay_ns           = 0;
        ALOGW("%s: pcm_htimestamp error", __func__);
        return;
    }

    /* read frames available in audio HAL input buffer
     * add number of frames being read as we want the capture time of first sample
     * in current buffer */
    /* frames in in->buffer are at driver sampling rate while frames in in->proc_buf are
     * at requested sampling rate */
    buf_delay = (long)(((int64_t)(in->read_buf_frames) * 1000000000) / in->config.rate +
                       ((int64_t)(in->proc_buf_frames) * 1000000000) /
                           in->requested_rate);

    /* add delay introduced by resampler */
    rsmp_delay = 0;
    if (in->resampler) {
        rsmp_delay = in->resampler->delay_ns(in->resampler);
    }

    kernel_delay = (long)(((int64_t)kernel_frames * 1000000000) / in->config.rate);

    delay_ns = kernel_delay + buf_delay + rsmp_delay;

    buffer->time_stamp = tstamp;
    buffer->delay_ns   = delay_ns;
    ALOGV("%s: time_stamp = [%ld].[%ld], delay_ns: [%d],"
         " kernel_delay:[%ld], buf_delay:[%ld], rsmp_delay:[%ld], kernel_frames:[%d], "
         "in->read_buf_frames:[%d], in->proc_buf_frames:[%d], frames:[%d]",
         __func__, buffer->time_stamp.tv_sec , buffer->time_stamp.tv_nsec, buffer->delay_ns,
         kernel_delay, buf_delay, rsmp_delay, kernel_frames,
         in->read_buf_frames, in->proc_buf_frames, frames);

}

static int32_t update_echo_reference(struct m0_stream_in *in, size_t frames)
{
    struct echo_reference_buffer b;
    b.delay_ns = 0;

    ALOGV("%s: frames = [%d], in->ref_frames_in = [%d],  "
          "b.frame_count = [%d]",
         __func__, frames, in->ref_buf_frames, frames - in->ref_buf_frames);
    if (in->ref_buf_frames < frames) {
        if (in->ref_buf_size < frames) {
            in->ref_buf_size = frames;
            in->ref_buf = (int16_t *)realloc(in->ref_buf, pcm_frames_to_bytes(in->pcm, frames));
            ALOG_ASSERT((in->ref_buf != NULL),
                        "%s failed to reallocate ref_buf", __func__);
            ALOGV("%s: ref_buf %p extended to %d bytes",
                      __func__, in->ref_buf, pcm_frames_to_bytes(in->pcm, frames));
        }
        b.frame_count = frames - in->ref_buf_frames;
        b.raw = (void *)(in->ref_buf + in->ref_buf_frames * in->config.channels);

        get_capture_delay(in, frames, &b);

        if (in->echo_reference->read(in->echo_reference, &b) == 0)
        {
            in->ref_buf_frames += b.frame_count;
            ALOGD("%s: in->ref_buf_frames:[%d], "
                    "in->ref_buf_size:[%d], frames:[%d], b.frame_count:[%d]",
                 __func__, in->ref_buf_frames, in->ref_buf_size, frames, b.frame_count);
        }
    } else
        ALOGW("%s: NOT enough frames to read ref buffer", __func__);
    return b.delay_ns;
}

static int set_preprocessor_param(effect_handle_t handle,
                           effect_param_t *param)
{
    uint32_t size = sizeof(int);
    uint32_t psize = ((param->psize - 1) / sizeof(int) + 1) * sizeof(int) +
                        param->vsize;

    int status = (*handle)->command(handle,
                                   EFFECT_CMD_SET_PARAM,
                                   sizeof (effect_param_t) + psize,
                                   param,
                                   &size,
                                   &param->status);
    if (status == 0)
        status = param->status;

    return status;
}

static int set_preprocessor_echo_delay(effect_handle_t handle,
                                     int32_t delay_us)
{
    uint32_t buf[sizeof(effect_param_t) / sizeof(uint32_t) + 2];
    effect_param_t *param = (effect_param_t *)buf;

    param->psize = sizeof(uint32_t);
    param->vsize = sizeof(uint32_t);
    *(uint32_t *)param->data = AEC_PARAM_ECHO_DELAY;
    *((int32_t *)param->data + 1) = delay_us;

    return set_preprocessor_param(handle, param);
}

static void push_echo_reference(struct m0_stream_in *in, size_t frames)
{
    /* read frames from echo reference buffer and update echo delay
     * in->ref_buf_frames is updated with frames available in in->ref_buf */
    int32_t delay_us = update_echo_reference(in, frames)/1000;
    int i;
    audio_buffer_t buf;

    if (in->ref_buf_frames < frames)
        frames = in->ref_buf_frames;

    buf.frameCount = frames;
    buf.raw = in->ref_buf;

    for (i = 0; i < in->num_preprocessors; i++) {
        if ((*in->preprocessors[i].effect_itfe)->process_reverse == NULL)
            continue;

        (*in->preprocessors[i].effect_itfe)->process_reverse(in->preprocessors[i].effect_itfe,
                                               &buf,
                                               NULL);
        set_preprocessor_echo_delay(in->preprocessors[i].effect_itfe, delay_us);
    }

    in->ref_buf_frames -= buf.frameCount;
    if (in->ref_buf_frames) {
        memcpy(in->ref_buf,
               in->ref_buf + buf.frameCount * in->config.channels,
               in->ref_buf_frames * in->config.channels * sizeof(int16_t));
    }
}

static void in_apply_ramp(struct m0_stream_in *in, int16_t *buffer, size_t frames)
{
    size_t i;
    uint16_t vol = in->ramp_vol;
    uint16_t step = in->ramp_step;

    frames = (frames < in->ramp_frames) ? frames : in->ramp_frames;

    if (in->channel_mask == AUDIO_CHANNEL_IN_MONO) {
        for (i = 0; i < frames; i++) {
            buffer[i] = (int16_t)((buffer[i] * vol) >> 16);
            vol += step;
        }
    } else {
        for (i = 0; i < frames; i++) {
            buffer[2*i] = (int16_t)((buffer[2*i] * vol) >> 16);
            buffer[2*i + 1] = (int16_t)((buffer[2*i + 1] * vol) >> 16);
            vol += step;
        }
    }

    in->ramp_vol = vol;
    in->ramp_frames -= frames;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    int ret = 0;
    struct m0_stream_in *in = (struct m0_stream_in *)stream;
    struct m0_audio_device *adev = in->dev;
    size_t frames_rq = bytes / audio_stream_frame_size(&stream->common);

    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the input stream mutex - e.g.
     * executing in_set_parameters() while holding the hw device
     * mutex
     */
    pthread_mutex_lock(&adev->lock);
    pthread_mutex_lock(&in->lock);
    if (in->standby) {
        ret = start_input_stream(in);
        if (ret == 0)
            in->standby = 0;
    }
    pthread_mutex_unlock(&adev->lock);

    if (ret < 0)
        goto exit;

    /*if (in->num_preprocessors != 0)
        ret = process_frames(in, buffer, frames_rq);
      else */
    ret = read_frames(in, buffer, frames_rq);

    if (ret > 0)
        ret = 0;

    if (in->ramp_frames > 0)
        in_apply_ramp(in, buffer, frames_rq);

    /*
     * Instead of writing zeroes here, we could trust the hardware
     * to always provide zeroes when muted.
     */
    if (ret == 0 && adev->mic_mute)
        memset(buffer, 0, bytes);

exit:
    if (ret < 0)
        usleep(bytes * 1000000 / audio_stream_frame_size(&stream->common) /
               in_get_sample_rate(&stream->common));

    pthread_mutex_unlock(&in->lock);
    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream)
{
    return 0;
}

#define GET_COMMAND_STATUS(status, fct_status, cmd_status) \
            do {                                           \
                if (fct_status != 0)                       \
                    status = fct_status;                   \
                else if (cmd_status != 0)                  \
                    status = cmd_status;                   \
            } while(0)

static int in_configure_reverse(struct m0_stream_in *in)
{
    int32_t cmd_status;
    uint32_t size = sizeof(int);
    effect_config_t config;
    int32_t status = 0;
    int32_t fct_status = 0;
    int i;

    if (in->num_preprocessors > 0) {
        config.inputCfg.channels = in->main_channels;
        config.outputCfg.channels = in->main_channels;
        config.inputCfg.format = AUDIO_FORMAT_PCM_16_BIT;
        config.outputCfg.format = AUDIO_FORMAT_PCM_16_BIT;
        config.inputCfg.samplingRate = in->requested_rate;
        config.outputCfg.samplingRate = in->requested_rate;
        config.inputCfg.mask =
                ( EFFECT_CONFIG_SMP_RATE | EFFECT_CONFIG_CHANNELS | EFFECT_CONFIG_FORMAT );
        config.outputCfg.mask =
                ( EFFECT_CONFIG_SMP_RATE | EFFECT_CONFIG_CHANNELS | EFFECT_CONFIG_FORMAT );

        for (i = 0; i < in->num_preprocessors; i++)
        {
            if ((*in->preprocessors[i].effect_itfe)->process_reverse == NULL)
                continue;
            fct_status = (*(in->preprocessors[i].effect_itfe))->command(
                                                        in->preprocessors[i].effect_itfe,
                                                        EFFECT_CMD_SET_CONFIG_REVERSE,
                                                        sizeof(effect_config_t),
                                                        &config,
                                                        &size,
                                                        &cmd_status);
            GET_COMMAND_STATUS(status, fct_status, cmd_status);
        }
    }
    return status;
}

#define MAX_NUM_CHANNEL_CONFIGS 10

static void in_read_audio_effect_channel_configs(struct m0_stream_in *in,
                                                 struct effect_info_s *effect_info)
{
    /* size and format of the cmd are defined in hardware/audio_effect.h */
    effect_handle_t effect = effect_info->effect_itfe;
    uint32_t cmd_size = 2 * sizeof(uint32_t);
    uint32_t cmd[] = { EFFECT_FEATURE_AUX_CHANNELS, MAX_NUM_CHANNEL_CONFIGS };
    /* reply = status + number of configs (n) + n x channel_config_t */
    uint32_t reply_size =
            2 * sizeof(uint32_t) + (MAX_NUM_CHANNEL_CONFIGS * sizeof(channel_config_t));
    int32_t reply[reply_size];
    int32_t cmd_status;

    ALOG_ASSERT((effect_info->num_channel_configs == 0),
                "in_read_audio_effect_channel_configs() num_channel_configs not cleared");
    ALOG_ASSERT((effect_info->channel_configs == NULL),
                "in_read_audio_effect_channel_configs() channel_configs not cleared");

    /* if this command is not supported, then the effect is supposed to return -EINVAL.
     * This error will be interpreted as if the effect supports the main_channels but does not
     * support any aux_channels */
    cmd_status = (*effect)->command(effect,
                                EFFECT_CMD_GET_FEATURE_SUPPORTED_CONFIGS,
                                cmd_size,
                                (void*)&cmd,
                                &reply_size,
                                (void*)&reply);

    if (cmd_status != 0) {
        ALOGI("%s: fx->command returned %d", __func__, cmd_status);
        return;
    }

    if (reply[0] != 0) {
        ALOGW("%s: "
                "command EFFECT_CMD_GET_FEATURE_SUPPORTED_CONFIGS error %d num configs %d",
                __func__, reply[0], (reply[0] == -ENOMEM) ? reply[1] : MAX_NUM_CHANNEL_CONFIGS);
        return;
    }

    /* the feature is not supported */
    ALOGI("in_read_audio_effect_channel_configs()(): "
            "Feature supported and adding %d channel configs to the list", reply[1]);
    effect_info->num_channel_configs = reply[1];
    effect_info->channel_configs =
            (channel_config_t *) malloc(sizeof(channel_config_t) * reply[1]); /* n x configs */
    memcpy(effect_info->channel_configs, (reply + 2), sizeof(channel_config_t) * reply[1]);
}


static uint32_t in_get_aux_channels(struct m0_stream_in *in)
{
    int i;
    channel_config_t new_chcfg = {0, 0};

    if (in->num_preprocessors == 0)
        return 0;

    /* do not enable dual mic configurations when capturing from other microphones than
     * main or sub */
    if (!(in->device & (AUDIO_DEVICE_IN_BUILTIN_MIC | AUDIO_DEVICE_IN_BACK_MIC)))
        return 0;

    /* retain most complex aux channels configuration compatible with requested main channels and
     * supported by audio driver and all pre processors */
    for (i = 0; i < NUM_IN_AUX_CNL_CONFIGS; i++) {
        channel_config_t *cur_chcfg = &in_aux_cnl_configs[i];
        if (cur_chcfg->main_channels == in->main_channels) {
            size_t match_cnt;
            size_t idx_preproc;
            for (idx_preproc = 0, match_cnt = 0;
                 /* no need to continue if at least one preprocessor doesn't match */
                 idx_preproc < (size_t)in->num_preprocessors && match_cnt == idx_preproc;
                 idx_preproc++) {
                struct effect_info_s *effect_info = &in->preprocessors[idx_preproc];
                size_t idx_chcfg;

                for (idx_chcfg = 0; idx_chcfg < effect_info->num_channel_configs; idx_chcfg++) {
                    if (memcmp(effect_info->channel_configs + idx_chcfg,
                               cur_chcfg,
                               sizeof(channel_config_t)) == 0) {
                        match_cnt++;
                        break;
                    }
                }
            }
            /* if all preprocessors match, we have a candidate */
            if (match_cnt == (size_t)in->num_preprocessors) {
                /* retain most complex aux channels configuration */
                if (popcount(cur_chcfg->aux_channels) > popcount(new_chcfg.aux_channels)) {
                    new_chcfg = *cur_chcfg;
                }
            }
        }
    }

    ALOGI("in_get_aux_channels(): return %04x", new_chcfg.aux_channels);

    return new_chcfg.aux_channels;
}

static int in_configure_effect_channels(effect_handle_t effect,
                                        channel_config_t *channel_config)
{
    int status = 0;
    int fct_status;
    int32_t cmd_status;
    uint32_t reply_size;
    effect_config_t config;
    uint32_t cmd[(sizeof(uint32_t) + sizeof(channel_config_t) - 1) / sizeof(uint32_t) + 1];

    ALOGI("in_configure_effect_channels(): configure effect with channels: [%04x][%04x]",
            channel_config->main_channels,
            channel_config->aux_channels);

    config.inputCfg.mask = EFFECT_CONFIG_CHANNELS;
    config.outputCfg.mask = EFFECT_CONFIG_CHANNELS;
    reply_size = sizeof(effect_config_t);
    fct_status = (*effect)->command(effect,
                                EFFECT_CMD_GET_CONFIG,
                                0,
                                NULL,
                                &reply_size,
                                &config);
    if (fct_status != 0) {
        ALOGE("in_configure_effect_channels(): EFFECT_CMD_GET_CONFIG failed");
        return fct_status;
    }

    config.inputCfg.channels = channel_config->main_channels | channel_config->aux_channels;
    config.outputCfg.channels = config.inputCfg.channels;
    reply_size = sizeof(uint32_t);
    fct_status = (*effect)->command(effect,
                                    EFFECT_CMD_SET_CONFIG,
                                    sizeof(effect_config_t),
                                    &config,
                                    &reply_size,
                                    &cmd_status);
    GET_COMMAND_STATUS(status, fct_status, cmd_status);

    cmd[0] = EFFECT_FEATURE_AUX_CHANNELS;
    memcpy(cmd + 1, channel_config, sizeof(channel_config_t));
    reply_size = sizeof(uint32_t);
    fct_status = (*effect)->command(effect,
                                EFFECT_CMD_SET_FEATURE_CONFIG,
                                sizeof(cmd), //sizeof(uint32_t) + sizeof(channel_config_t),
                                cmd,
                                &reply_size,
                                &cmd_status);
    GET_COMMAND_STATUS(status, fct_status, cmd_status);

    /* some implementations need to be re-enabled after a config change */
    reply_size = sizeof(uint32_t);
    fct_status = (*effect)->command(effect,
                                  EFFECT_CMD_ENABLE,
                                  0,
                                  NULL,
                                  &reply_size,
                                  &cmd_status);
    GET_COMMAND_STATUS(status, fct_status, cmd_status);

    return status;
}

static int in_reconfigure_channels(struct m0_stream_in *in,
                                   effect_handle_t effect,
                                   channel_config_t *channel_config,
                                   bool config_changed) {

    int status = 0;

    ALOGI("%s: config_changed %d effect %p",
          __func__, config_changed, effect);

    /* if config changed, reconfigure all previously added effects */
    if (config_changed) {
        int i;
        for (i = 0; i < in->num_preprocessors; i++)
        {
            int cur_status = in_configure_effect_channels(in->preprocessors[i].effect_itfe,
                                                  channel_config);
            if (cur_status != 0) {
                ALOGI("%s: error %d configuring effect "
                        "%d with channels: [%04x][%04x]",
                        __func__,
                        cur_status,
                        i,
                        channel_config->main_channels,
                        channel_config->aux_channels);
                status = cur_status;
            }
        }
    } else if (effect != NULL && channel_config->aux_channels) {
        /* if aux channels config did not change but aux channels are present,
         * we still need to configure the effect being added */
        status = in_configure_effect_channels(effect, channel_config);
    }
    return status;
}

static void in_update_aux_channels(struct m0_stream_in *in,
                                   effect_handle_t effect)
{
    uint32_t aux_channels;
    channel_config_t channel_config;
    int status;

    aux_channels = in_get_aux_channels(in);

    channel_config.main_channels = in->main_channels;
    channel_config.aux_channels = aux_channels;
    status = in_reconfigure_channels(in,
                                     effect,
                                     &channel_config,
                                     (aux_channels != in->aux_channels));

    if (status != 0) {
        ALOGI("%s: in_reconfigure_channels error %d", __func__, status);
        /* resetting aux channels configuration */
        aux_channels = 0;
        channel_config.aux_channels = 0;
        in_reconfigure_channels(in, effect, &channel_config, true);
    }
    if (in->aux_channels != aux_channels) {
        in->aux_channels_changed = true;
        in->aux_channels = aux_channels;
        do_in_standby(in);
    }
}

static int in_add_audio_effect(const struct audio_stream *stream,
                               effect_handle_t effect)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;
    int status;
    effect_descriptor_t desc;

    pthread_mutex_lock(&in->dev->lock);
    pthread_mutex_lock(&in->lock);
    if (in->num_preprocessors >= MAX_PREPROCESSORS) {
        status = -ENOSYS;
        goto exit;
    }

    status = (*effect)->get_descriptor(effect, &desc);
    if (status != 0)
        goto exit;

    in->preprocessors[in->num_preprocessors].effect_itfe = effect;
    /* add the supported channel of the effect in the channel_configs */
    in_read_audio_effect_channel_configs(in, &in->preprocessors[in->num_preprocessors]);

    in->num_preprocessors++;

    /* check compatibility between main channel supported and possible auxiliary channels */
    in_update_aux_channels(in, effect);

    ALOGV("%s: effect type: %08x", __func__, desc.type.timeLow);

    if (memcmp(&desc.type, FX_IID_AEC, sizeof(effect_uuid_t)) == 0) {
        in->need_echo_reference = true;
        do_in_standby(in);
        in_configure_reverse(in);
    }

exit:

    ALOGW_IF(status != 0, "%s: error %d", __func__, status);
    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&in->dev->lock);
    return status;
}

static int in_remove_audio_effect(const struct audio_stream *stream,
                                  effect_handle_t effect)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;
    int i;
    int status = -EINVAL;
    effect_descriptor_t desc;

    pthread_mutex_lock(&in->dev->lock);
    pthread_mutex_lock(&in->lock);
    if (in->num_preprocessors <= 0) {
        status = -ENOSYS;
        goto exit;
    }

    for (i = 0; i < in->num_preprocessors; i++) {
        if (status == 0) { /* status == 0 means an effect was removed from a previous slot */
            in->preprocessors[i - 1].effect_itfe = in->preprocessors[i].effect_itfe;
            in->preprocessors[i - 1].channel_configs = in->preprocessors[i].channel_configs;
            in->preprocessors[i - 1].num_channel_configs = in->preprocessors[i].num_channel_configs;
            ALOGI("in_remove_audio_effect moving fx from %d to %d", i, i - 1);
            continue;
        }
        if (in->preprocessors[i].effect_itfe == effect) {
            ALOGI("in_remove_audio_effect found fx at index %d", i);
            free(in->preprocessors[i].channel_configs);
            status = 0;
        }
    }

    if (status != 0)
        goto exit;

    in->num_preprocessors--;
    /* if we remove one effect, at least the last preproc should be reset */
    in->preprocessors[in->num_preprocessors].num_channel_configs = 0;
    in->preprocessors[in->num_preprocessors].effect_itfe = NULL;
    in->preprocessors[in->num_preprocessors].channel_configs = NULL;

    /* check compatibility between main channel supported and possible auxiliary channels */
    in_update_aux_channels(in, NULL);

    status = (*effect)->get_descriptor(effect, &desc);
    if (status != 0)
        goto exit;

    ALOGI("%s: effect type: %08x", __func__, desc.type.timeLow);

    if (memcmp(&desc.type, FX_IID_AEC, sizeof(effect_uuid_t)) == 0) {
        in->need_echo_reference = false;
        do_in_standby(in);
    }

exit:

    ALOGW_IF(status != 0, "%s: error %d", __func__, status);
    pthread_mutex_unlock(&in->lock);
    pthread_mutex_unlock(&in->dev->lock);
    return status;
}

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;
    struct m0_stream_out *out;
    int ret;

    ALOGV("%s begin", __func__);

    *stream_out = NULL;

    out = (struct m0_stream_out *)calloc(1, sizeof(struct m0_stream_out));
    if (!out)
        return -ENOMEM;

    out->supported_channel_masks[0] = AUDIO_CHANNEL_OUT_STEREO;
    out->channel_mask = AUDIO_CHANNEL_OUT_STEREO;
    if (devices == AUDIO_DEVICE_NONE)
        devices = AUDIO_DEVICE_OUT_SPEAKER;
    out->device = devices;

    if (adev->outputs[OUTPUT_DEEP_BUF] != NULL) {
        ret = -ENOSYS;
        goto err_open;
    }

    /* TODO - Study this code */
    if (flags & AUDIO_OUTPUT_FLAG_DEEP_BUFFER) {
        /* We'll start with short period size, and during out_write() will check
         * whether to change to pcm_config_deep_long if screen is off */
        out->config = pcm_config_deep_short;
        out->pcm_device = PCM_DEVICE;
        out->output_type = OUTPUT_DEEP_BUF;
    } else {
        out->config = pcm_config_fast;
        out->pcm_device = PCM_DEVICE;
        out->output_type = OUTPUT_LOW_LATENCY;
    }

    out->stream.common.get_sample_rate = out_get_sample_rate; //Ok
    out->stream.common.set_sample_rate = out_set_sample_rate; //ok
    out->stream.common.get_buffer_size = out_get_buffer_size; //ok
    out->stream.common.get_channels = out_get_channels; //ok
    out->stream.common.get_format = out_get_format; //ok
    out->stream.common.set_format = out_set_format; //ok
    out->stream.common.standby = out_standby; //ok
    out->stream.common.dump = out_dump; //ok
    out->stream.common.set_parameters = out_set_parameters; //differs from i9300
    out->stream.common.get_parameters = out_get_parameters; //ok
    out->stream.common.add_audio_effect = out_add_audio_effect; //ok
    out->stream.common.remove_audio_effect = out_remove_audio_effect; //ok
    out->stream.get_latency = out_get_latency; //ok
    out->stream.set_volume = out_set_volume; //ok
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;

    /* This was in old i9300 hal. Since all sampling rates are 44100 Hz, there
     * is no need to resample.
     * ret = create_resampler(DEFAULT_OUT_SAMPLING_RATE,
                           MM_FULL_POWER_SAMPLING_RATE,
                           2,
                           RESAMPLER_QUALITY_DEFAULT,
                           NULL,
                           &out->resampler);
    if (ret != 0)
        goto err_open; */

    out->dev = adev;

    config->format = out_get_format(&out->stream.common);
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    out->standby = true;

    pthread_mutex_lock(&adev->lock);
    if (adev->outputs[out->output_type]) {
        pthread_mutex_unlock(&adev->lock);
        ret = -EBUSY;
        goto err_open;
    }
    adev->outputs[out->output_type] = out;
    pthread_mutex_unlock(&adev->lock);

    *stream_out = &out->stream;

    ALOGV("%s end", __func__);

    return 0;

err_open:
    free(out);
    *stream_out = NULL;
    ALOGV("%s end", __func__);

    return ret;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    struct m0_audio_device *adev;
    enum output_type type;

    out_standby(&stream->common);
    adev = (struct m0_audio_device *)dev;
    pthread_mutex_lock(&adev->lock);
    for (type = 0; type < OUTPUT_TOTAL; type++) {
        if (adev->outputs[type] == (struct m0_stream_out *) stream) {
            adev->outputs[type] = NULL;
            break;
        }
    }
    pthread_mutex_unlock(&adev->lock);
    free(stream);
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;
    struct str_parms *parms;
    char *str;
    char value[32];
    int ret;

    parms = str_parms_create_str(kvpairs);
#if 0
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_TTY_MODE, value, sizeof(value));
    if (ret >= 0) {
        int tty_mode;

        if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_OFF) == 0)
            tty_mode = TTY_MODE_OFF;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_VCO) == 0)
            tty_mode = TTY_MODE_VCO;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_HCO) == 0)
            tty_mode = TTY_MODE_HCO;
        else if (strcmp(value, AUDIO_PARAMETER_VALUE_TTY_FULL) == 0)
            tty_mode = TTY_MODE_FULL;
        else
            return -EINVAL;

        pthread_mutex_lock(&adev->lock);
        if (tty_mode != adev->tty_mode) {
            adev->tty_mode = tty_mode;
            if (adev->mode == AUDIO_MODE_IN_CALL)
                select_output_device(adev);
        }
        pthread_mutex_unlock(&adev->lock);
    }
#endif

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_KEY_BT_NREC, value, sizeof(value));
    if (ret >= 0) {
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0)
            adev->bluetooth_nrec = true;
        else
            adev->bluetooth_nrec = false;
    }

    ret = str_parms_get_str(parms, "screen_off", value, sizeof(value));
    if (ret >= 0) {
        if (strcmp(value, AUDIO_PARAMETER_VALUE_ON) == 0)
            adev->screen_off = false;
        else
            adev->screen_off = true;
    }

    ret = str_parms_get_str(parms, "noise_suppression", value, sizeof(value));
    if (ret >= 0) {
        if (strcmp(value, "true") == 0) {
            ALOGV("%s: enabling two mic control", __func__);
            ril_set_two_mic_control(&adev->ril, AUDIENCE, TWO_MIC_SOLUTION_ON);
            // TODO - Is sub mic enabled in audio route?
        } else {
            ALOGV("%s: disabling two mic control", __func__);
            ril_set_two_mic_control(&adev->ril, AUDIENCE, TWO_MIC_SOLUTION_OFF);
        }
    }

    str_parms_destroy(parms);
    return ret;
}

static char * adev_get_parameters(const struct audio_hw_device *dev,
                                  const char *keys)
{
    return strdup("");
}

static int adev_init_check(const struct audio_hw_device *dev)
{
    return 0;
}

static int adev_set_voice_volume(struct audio_hw_device *dev, float volume)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;

    adev->voice_volume = volume;

    if (adev->mode == AUDIO_MODE_IN_CALL)
        ril_set_call_volume(&adev->ril, SOUND_TYPE_VOICE, volume);

    return 0;
}

static int adev_set_master_volume(struct audio_hw_device *dev, float volume)
{
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;

    if (adev->mode == mode)
        return 0;

    pthread_mutex_lock(&adev->lock);
    adev->mode = mode;

    if (adev->mode == AUDIO_MODE_IN_CALL) {
        ALOGV("%s: Entering IN_CALL mode", __func__);
        if (!adev->in_call) {
            if (adev->out_device == AUDIO_DEVICE_NONE ||
                adev->out_device == AUDIO_DEVICE_OUT_SPEAKER) {
                adev->out_device = AUDIO_DEVICE_OUT_EARPIECE;
            }
            adev->input_source = AUDIO_SOURCE_VOICE_CALL;
            select_devices(adev);
            start_voice_call(adev);
            ril_set_call_clock_sync(&adev->ril, SOUND_CLOCK_START);
            adev_set_voice_volume(&adev->hw_device, adev->voice_volume);
            adev->in_call = true;
        }
    } else {
        ALOGV("%s: Leaving IN_CALL mode", __func__);
        if (adev->in_call) {
            adev->in_call = false;
            ril_set_call_clock_sync(&adev->ril, SOUND_CLOCK_STOP);
            end_voice_call(adev);
            if (adev->out_device & AUDIO_DEVICE_OUT_ALL_SCO)
                end_bt_sco(adev);
            adev->input_source = AUDIO_SOURCE_DEFAULT;
            select_devices(adev);
        }
    }
    pthread_mutex_unlock(&adev->lock);

    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;
    //enum ril_mute_state ril_state = state ? TX_MUTE : TX_UNMUTE;

    ALOGV("%s: set mic mute: %d\n", __func__, state);

    adev->mic_mute = state;

    /* TODO
     if (adev->in_call)
        ril_set_mute(&adev->ril, ril_state); */

    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;

    *state = adev->mic_mute;

    return 0;
}

static int check_input_parameters(uint32_t sample_rate, audio_format_t format, int channel_count)
{
    if (format != AUDIO_FORMAT_PCM_16_BIT)
        return -EINVAL;

    if ((channel_count < 1) || (channel_count > 2))
        return -EINVAL;

    switch(sample_rate) {
    case 8000:
    case 11025:
    case 16000:
    case 22050:
    case 24000:
    case 32000:
    case 44100:
    case 48000:
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev,
                                         const struct audio_config *config)
{
    size_t size;
    int channel_count = popcount(config->channel_mask);
    if (check_input_parameters(config->sample_rate, config->format, channel_count) != 0)
        return 0;

    return get_input_buffer_size(config->sample_rate, config->format, channel_count);
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **m0_stream_in)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)dev;
    struct m0_stream_in *in;
    int ret;

    *m0_stream_in = NULL;

    /* Respond with a request for stereo if a different format is given. */
    if (config->channel_mask != AUDIO_CHANNEL_IN_STEREO) {
        config->channel_mask = AUDIO_CHANNEL_IN_STEREO;
        return -EINVAL;
    }

    int channel_count = popcount(config->channel_mask);

    *m0_stream_in = NULL;

    if (check_input_parameters(config->sample_rate, config->format, channel_count) != 0)
            return -EINVAL;

    in = (struct m0_stream_in *)calloc(1, sizeof(struct m0_stream_in));
    if (!in)
        return -ENOMEM;

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    in->dev = adev;
    in->standby = true;
    in->requested_rate = config->sample_rate;
    in->input_source = AUDIO_SOURCE_DEFAULT;
    /* strip AUDIO_DEVICE_BIT_IN to allow bitwise comparisons */
    in->device = devices & ~AUDIO_DEVICE_BIT_IN;
    in->io_handle = handle;
    in->channel_mask = config->channel_mask;

    in->buffer = malloc(pcm_config_in.period_size * pcm_config_in.channels
                                               * audio_stream_frame_size(&in->stream.common));
    if (!in->buffer) {
        ret = -ENOMEM;
        goto err_malloc;
    }

    if (in->requested_rate != pcm_config_in.rate) {
        in->buf_provider.get_next_buffer = get_next_buffer;
        in->buf_provider.release_buffer = release_buffer;

        ret = create_resampler(pcm_config_in.rate,
                               in->requested_rate,
                               popcount(in->channel_mask),
                               RESAMPLER_QUALITY_DEFAULT,
                               &in->buf_provider,
                               &in->resampler);
        if (ret != 0) {
            ret = -EINVAL;
            goto err_resampler;
        }

        ALOGV("%s: Created resampler converting %d -> %d\n",
              __func__, pcm_config_in.rate, in->requested_rate);
    }

    ALOGV("%s: Requesting input stream with rate: %d, channels: 0x%x\n",
          __func__, config->sample_rate, config->channel_mask);

    *m0_stream_in = &in->stream;
    return 0;

err_resampler:
    free(in->buffer);
err_malloc:
    free(in);
    return ret;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                   struct audio_stream_in *stream)
{
    struct m0_stream_in *in = (struct m0_stream_in *)stream;

    in_standby(&stream->common);
    if (in->resampler) {
        release_resampler(in->resampler);
        in->resampler = NULL;
    }
    free(in->buffer);
    free(stream);
}

static int adev_dump(const audio_hw_device_t *device, int fd)
{
    return 0;
}

static int adev_close(hw_device_t *device)
{
    struct m0_audio_device *adev = (struct m0_audio_device *)device;

    audio_route_free(adev->ar);

    /* RIL */
    ril_close(&adev->ril);

    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
	struct m0_audio_device *adev;
    int ret;

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct m0_audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close; // Ok

    adev->hw_device.init_check = adev_init_check; // Ok
    adev->hw_device.set_voice_volume = adev_set_voice_volume; // Ok
    adev->hw_device.set_master_volume = adev_set_master_volume; // Ok
    adev->hw_device.set_mode = adev_set_mode; // Ok
    adev->hw_device.set_mic_mute = adev_set_mic_mute; // Ok
    adev->hw_device.get_mic_mute = adev_get_mic_mute; // Ok
    adev->hw_device.set_parameters = adev_set_parameters; // Ok
    adev->hw_device.get_parameters = adev_get_parameters; // Ok
    adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size; // Ok
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    adev->hw_device.open_input_stream = adev_open_input_stream;
    adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;
    adev->hw_device.set_master_mute = NULL;
    adev->hw_device.get_master_mute = NULL;

    adev->ar = audio_route_init(MIXER_CARD, XML_FILE_PATH);
    adev->input_source = AUDIO_SOURCE_DEFAULT;
    /* adev->cur_route_id initial value is 0 and such that first device
     * selection is always applied by select_devices() */

    adev->mode = AUDIO_MODE_NORMAL;
    //adev->pcm_modem_dl = NULL;
    //adev->pcm_modem_ul = NULL;
    adev->voice_volume = 1.0f;
    //adev->tty_mode = TTY_MODE_OFF;
    //adev->bluetooth_nrec = true;
    //adev->wb_amr = 0;

    /* RIL */
    ril_open(&adev->ril);
    /* register callback for wideband AMR setting */
    ril_register_set_wb_amr_callback(adev_set_wb_amr_callback, (void *)adev);

    *device = &adev->hw_device.common;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AUDIO_HARDWARE_MODULE_ID,
        .name = "M0 audio HW HAL",
        .author = "The CyanogenMod Project",
        .methods = &hal_module_methods,
    },
};
