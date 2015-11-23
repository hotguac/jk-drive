/*
  Copyright 2006-2011 David Robillard <d@drobilla.net>
  Copyright 2006 Steve Harris <steve@plugin.org.uk>
  Copyright 2015 Joe Kokosa

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.

  THIS SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

/** Include standard C headers */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/** Include Secret Rabbit Code        **/
/** for sample rate conversion        **/
/** see http://www.mega-nerd.com/SRC/ **/
#include <samplerate.h>

/**
   LV2 headers are based on the URI of the specification they come from, so a
   consistent convention can be used even for unofficial extensions.  The URI
   of the core LV2 specification is <http://lv2plug.in/ns/lv2core>, by
   replacing `http:/` with `lv2` any header in the specification bundle can be
   included, in this case `lv2.h`.
*/
#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

/**
   The URI is the identifier for a plugin, and how the host associates this
   implementation in code with its description in data. If this URI does not
   match that used in the data files, the host will fail to load the plugin.
*/
#define AMP_URI "http://mrkmusings.com/jk-drive"

/**
   In the code, ports are referred to by index.  An enumeration of port indices
   should be defined for readability. They need to match the definitions in the
   *.ttl file.
*/
typedef enum {
	AMP_DRIVE   = 0,
	AMP_GAIN    = 1,
	AMP_INPUT   = 2,
	AMP_OUTPUT  = 3,
	AMP_LATENCY = 4
} PortIndex;

/**
   Define a private structure for the plugin instance.  All data
   associated with a plugin instance is stored here, and is available to
   every instance method, being passed back through the 'instance' parameter.
**/
typedef struct {
	// Port buffers
	const float* gain;         // lv2 control port
	const float* drive;        // lv2 control port

	const float* lv2_input;    // lv2 audio port;
	float*       lv2_output;   // lv2 audio port;

	float*       latency;      // lv2 control port (output);

	float        prev_gain;    // save for smoothing
	float        prev_drive;   // save for smoothing

	float*       up_input;     // buffer in for up convert
	uint32_t     up_in_level;  // how many frames in buffer
	SRC_DATA*    up_data;      // save conversion config
	SRC_STATE*   up_state;     // save conversion state

	float*       process_input;     // buffer in for waveshaping
	float*       process_output;    // how many frames in buffer
	long int     process_level;     // buffer out for waveshaping

	float*       down_input;        // buffer for down conversion
	uint32_t     down_in_level;     // how many frames in buffer
	SRC_DATA*    down_data;         // save conversion config
	SRC_STATE*   down_state;        // save conversion state

	float*       generate_input;    // buffer to generate output
	uint32_t     generate_in_level; // how many frames in buffer

	int          first_run;         // do we need to prime the
	                                // up sample process
	double       save_rate;         // sample rate in fps

	float        dc_last_X;         // used for dc bias removal
	float        dc_last_Y;         // used for dc bias removal
} Amp;

#define OVER_SAMPLE_RATE 2.0f
#define BUFFER_LEN 8192
#define BUFFER_LEN_UP (8192 * (int) OVER_SAMPLE_RATE)

/* convert enums in samplerate.h choices are */
/* SRC_SINC_BEST_QUALITY SRC_SINC_MEDIUM_QUALITY SRC_SINC_FASTEST SRC_ZERO_ORDER_HOLD SRC_LINEAR */

#define CONVERTER SRC_SINC_FASTEST
/* #define JK_DRIVE_DEBUG 1 */

/** Define a macro for converting a gain in dB to a coefficient. */
#define DB_CO(g) ((g) > -90.0f ? powf(10.0f, (g) * 0.05f) : 0.0f)

/* Always add to end of input queue. Using routines are responsible */
/* for shifting data up */
static void
add_to_input(Amp* amp, uint32_t n_samples)
{
	const float* source = amp->lv2_input;
	float* dest = amp->up_input;
	uint32_t n_to_copy = n_samples;
	uint32_t up_in_level = amp->up_in_level;
	uint32_t pos;

	/* pad the beginning with silence when first starting */
	/* this primes the pump to allow the up sample convert */
	/* to produce enough output */
	if (amp->first_run != 0) {
		amp->first_run = 0;
		for (pos = 0; pos < n_samples; pos++) {
			dest[up_in_level] = 0.0f;
			up_in_level++;
		}
	}

	if (n_samples > BUFFER_LEN - up_in_level) {
		fprintf(stderr, "JK_DRIVE: dropping input samples\n");
		n_to_copy = BUFFER_LEN - up_in_level;
	}

	// check for a disconnected input port */
	for (pos = 0; pos < n_to_copy; pos++) {
		if (source == NULL) {
			dest[up_in_level] = 0.0f;
		} else {
			dest[up_in_level] = source[pos];
		}

		up_in_level++;
	}

	amp->up_in_level = up_in_level;
}

static void
up_sample(Amp* amp)
{
	int result = 0;
	SRC_DATA* src_data = amp->up_data;
	const char* message;

	src_data->data_in = amp->up_input;
	src_data->data_out = amp->process_input + amp->process_level;

	src_data->input_frames = amp->up_in_level;
	src_data->output_frames = BUFFER_LEN_UP - amp->process_level;

	src_data->end_of_input = 0;
	src_data->src_ratio = OVER_SAMPLE_RATE;

	result = src_process(amp->up_state,src_data);

	if (result != 0) {
		fprintf(stderr, "JK_DRIVE: error up sample processing %d\n",result);

		message = src_strerror(result);
		fprintf(stderr, "JK_DRIVE: %s\n", message);
	}

	amp->process_level += src_data->output_frames_gen;
	amp->up_in_level -= src_data->input_frames_used;

	/* handle case where we didn't use up all input samples */
	if (src_data->input_frames_used < src_data->input_frames) {
		memmove(amp->up_input,
			(amp->up_input + src_data->input_frames_used),
			amp->up_in_level*sizeof(float));
	}
}

static void
process(Amp* amp)
{
	const float max  = 0.99f;

	const float ramp_secs = 0.002f;

	float* dest;
	uint32_t down_in_level;
        uint32_t ramp_frames;
	uint32_t pos;
	float gain;
	float drive;
	float coef_drive;
	float coef_gain;

	const double rate = amp->save_rate;
	float prev_gain = amp->prev_gain;
	float prev_drive = amp->prev_drive;

	const float* input = amp->process_input;
	float* output = amp->process_output;

	uint32_t process_level = amp->process_level;

	if (amp->gain == NULL) {
		gain = 0.0f;
	} else {
		gain   = *(amp->gain);
	}

	if (amp->drive == NULL) {
		drive = 0.0f;
	} else {
		drive = *(amp->drive);
	}

	coef_drive = DB_CO(drive);
	coef_gain = DB_CO(gain);

	ramp_frames = (uint32_t) (rate * OVER_SAMPLE_RATE) * ramp_secs;

	// Make sure the ramp is finished by end of this cycle
	if (ramp_frames > process_level) {
		ramp_frames = process_level;
	}

	// Prevent divide by zero below
	if (ramp_frames == 0) {
		ramp_frames = 1;
	}

	const float gain_step = (gain - prev_gain) / ramp_frames;
	const float drive_step = (drive - prev_drive) /ramp_frames;

	// if either input or output buffer failed to allocated
	// then nothing to process
	if ((input == NULL) || (output == NULL)) {
		process_level = 0;
	}

	for (pos = 0; pos < process_level; pos++) {
		if (pos < ramp_frames) {
			prev_drive += drive_step;
			coef_drive = DB_CO(prev_drive);

			prev_gain += gain_step;
			coef_gain = DB_CO(prev_gain);
		}

		/* Do some wave shaping */
		/* Step 1 - add 2nd order harmonics */
		/*   y = (a*x + b*x*x) * coef_drive = x*(a + b*x)*coef_drive */
		output[pos] = input[pos] * (0.5f  + (0.5f * input[pos])) * coef_drive;

		/* Step 2 - remove DC offset from step 1 with highpass/notch */
		/* filtered_value = 0.996 × (last_filtered_value + sample - last_sample) */
		/* or y[n] = x[n] - x[n-1] + a * y[n-1] */
		float y = output[pos] - amp->dc_last_X + (0.998 * amp->dc_last_Y);
		amp->dc_last_X = output[pos];
		amp->dc_last_Y = y;
		output[pos] = y;

		/* Step 3 - do more waveshaping to add odd harmonics   */
		/* use logistics function to apply an 'S' curve        */
		/* see https://en.wikipedia.org/wiki/Logistic_function */
		/* k10 : 2.16 gives unity gain at x~0.4 use for 0db gain*/
		/* k10 : 2.40 gives unity gain at x~0.6 */
		/* k10 : 2.88 gives unity gain at x~0.8 use for 24db gain*/
		const float L = 1.96f;
		const float k10 = 2.4f;
		float fx;

		fx = (L / (1.0f + expf(output[pos] * -k10))) - (L / 2.0f);

		/* Apply output gain */
		output[pos] = fx * coef_gain;

		/* Prevent wrap around clipping by hard clipping */
		if (output[pos] > max) {
			output[pos] = max;
		} else if (output[pos] < -max) {
			output[pos] = -max;
		}
	}

	dest = amp->down_input;
	down_in_level = amp->down_in_level;

	if (process_level > BUFFER_LEN_UP - down_in_level) {
		fprintf(stderr, "JK_DRIVE: dropping processing samples\n");
		fprintf(stderr, "JK_DRIVE: needed %ul samples\n", process_level);
		fprintf(stderr, "JK_DRIVE: only %ul available\n", BUFFER_LEN - down_in_level);
		process_level = BUFFER_LEN - down_in_level;
	}

	for (pos = 0; pos < process_level; pos++) {
		dest[down_in_level] = output[pos];
		down_in_level++;
	}

	amp->down_in_level = down_in_level;
	amp->process_level = 0;
	amp->prev_gain = gain;
	amp->prev_drive = drive;
}

static void
down_sample(Amp* amp)
{
	const char* message;
	SRC_DATA* src_data = amp->down_data;

	src_data->data_in = amp->down_input;
	src_data->data_out = (amp->generate_input + amp->generate_in_level);

	src_data->input_frames = amp->down_in_level;
	src_data->output_frames = BUFFER_LEN - amp->generate_in_level;

	src_data->end_of_input = 0;
	src_data->src_ratio = 1.0f / OVER_SAMPLE_RATE;

	int result = src_process(amp->down_state,src_data);
	if (result) {
		fprintf(stderr,"JK_DRIVE: error down sample %d\n", result);
		message = src_strerror(result);
		fprintf(stderr,"JK_DRIVE: %s\n", message);
	}

	amp->generate_in_level += src_data->output_frames_gen;
	amp->down_in_level -= src_data->input_frames_used;

	/* handle case where we didn't use up all input samples */
	if (src_data->input_frames_used < src_data->input_frames) {
		memmove(amp->down_input,(amp->down_input + src_data->input_frames_used),amp->down_in_level*sizeof(float));
	}
}

static void
generate_output(Amp* amp, uint32_t n_samples)
{
	float* const output = amp->lv2_output;
	uint32_t n_to_copy = n_samples;
	uint32_t pos = 0;
	uint32_t in_level = amp->generate_in_level;

	if (in_level < n_to_copy) {
		n_to_copy = in_level;
	}

	/* check to make sure output is connected to a buffer */
	/* before writing to output */
	if (output != NULL) {
		for (pos = 0; pos < n_to_copy; pos++) {
			output[pos] = amp->generate_input[pos];
		}

		/* pad if necessary */
		for (pos = n_to_copy; pos < n_samples; pos++) {
			output[pos] = 0.0f;
		}
	}

	/* shift any unused input samples */
	if ((in_level > n_to_copy) && (n_to_copy > 0)) {
		in_level -= n_to_copy;
		memmove(amp->generate_input,(amp->generate_input + n_to_copy),in_level*sizeof(float));
	}

	amp->generate_in_level = in_level;
}

/**
   The `connect_port()` method is called by the host to connect a particular
   port to a buffer.  The plugin must store the data location, but data may not
   be accessed except in run().

   This method is in the ``audio'' threading class, and is called in the same
   context as run().
*/
static void
connect_port(LV2_Handle instance,
             uint32_t   port,
             void*      data)
{
	Amp* amp = (Amp*)instance;

	if (amp == NULL ) {
		return;
	}

	switch ((PortIndex)port) {
	case AMP_GAIN:
		amp->gain = (const float*)data;
		break;
	case AMP_DRIVE:
		amp->drive = (const float*)data;
		break;
	case AMP_INPUT:
 		amp->lv2_input = (const float*)data;
		break;
	case AMP_OUTPUT:
		amp->lv2_output = (float*)data;
		break;
	case AMP_LATENCY:
		amp->latency = data;
		break;
	}
}

/**
   The `run()` method is the main process function of the plugin.  It processes
   a block of audio in the audio context.  Since this plugin is
   `lv2:hardRTCapable`, `run()` must be real-time safe, so blocking (e.g. with
   a mutex) or memory allocation are not allowed.
*/
static void
run(LV2_Handle instance, uint32_t n_samples)
{
	Amp* amp = (Amp*)instance;

	// check that we have a valid instance before
	// checking for valid buffers
	if (amp == NULL) {
		fprintf(stderr, "JK_DRIVE: run() called with NULL instance parameter.\n");
		return;
	}

	// Check for valid buffers
	if ((amp->up_input == NULL) ||
	    (amp->process_input == NULL) ||
	    (amp->process_output == NULL) ||
	    (amp->down_input == NULL) ||
	    (amp->generate_input == NULL) ||
	    (amp->up_data == NULL) ||
	    (amp->down_data == NULL)) {
		return;
	}

	add_to_input(amp, n_samples);
	up_sample(amp);
	process(amp);
	down_sample(amp);
	generate_output(amp, n_samples);

	if (amp->latency != NULL) {
		*(amp->latency) = (float)n_samples;
	}
}

/**
   The `instantiate()` function is called by the host to create a new plugin
   instance.  The host passes the plugin descriptor, sample rate, and bundle
   path for plugins that need to load additional resources (e.g. waveforms).
   The features parameter contains host-provided features defined in LV2
   extensions, but this simple plugin does not use any.

   This function is in the ``instantiation'' threading class, so no other
   methods on this instance will be called concurrently with it.
*/
static LV2_Handle
instantiate(const LV2_Descriptor*     descriptor,
            double                    rate,
            const char*               bundle_path,
            const LV2_Feature* const* features
	    )
{
	int error;
	const char* message;

	Amp* amp = (Amp*)malloc(sizeof(Amp));

	if (amp != NULL) {
		/* we'll size the buffers once and hope they're big enough */
		amp->up_input = malloc(BUFFER_LEN * sizeof(float));
		amp->process_input = malloc(BUFFER_LEN_UP * sizeof(float));
		amp->process_output = malloc(BUFFER_LEN_UP * sizeof(float));
		amp->down_input = malloc(BUFFER_LEN_UP * sizeof(float));
		amp->generate_input = malloc(BUFFER_LEN * sizeof(float));

		amp->up_in_level = 0;
		amp->process_level = 0;
		amp->down_in_level = 0;
		amp->generate_in_level = 0;

		amp->up_data = malloc(sizeof(SRC_DATA));
		amp->down_data = malloc(sizeof(SRC_DATA));

		amp->first_run = 1;

		amp->up_state = src_new (CONVERTER, 1, &error) ;
		if (error != 0) {
			fprintf(stderr,"JK_DRIVE: Error on up new %d\n",error);
			message =  src_strerror(error);
			fprintf(stderr,"JK_DRIVE: %s\n",message);
		}

		amp->down_state = src_new (CONVERTER, 1, &error) ;
		if (error != 0) {
			fprintf(stderr,"JK_DRIVE: Error on down new %d\n",error);
			message =  src_strerror(error);
			fprintf(stderr,"JK_DRIVE: %s\n",message);
		}

		amp->save_rate  = rate;
	} else {
		fprintf(stderr, "JK_DRIVE: instantiate() called with NULL instance parameter.\n");
	}

	return (LV2_Handle)amp;
}

/**
   The `activate()` method is called by the host to initialise and prepare the
   plugin instance for running.  The plugin must reset all internal state
   except for buffer locations set by `connect_port()`.  Since this plugin has
   no other internal state, this method does nothing.

   This method is in the ``instantiation'' threading class, so no other
   methods on this instance will be called concurrently with it.
*/
static void
activate(LV2_Handle instance)
{
	Amp* amp = (Amp*)instance;

	if (amp == NULL) {
		fprintf(stderr, "JK_DRIVE: activate() called with NULL instance parameter.\n");
		return;
	}

	amp->first_run  = 1;
	amp->prev_gain  = 0.0f;
	amp->prev_drive = 0.0f;
	amp->dc_last_X  = 0.0f;
	amp->dc_last_Y  = 0.0f;
}

/**
   The `deactivate()` method is the counterpart to `activate()`, and is called by
   the host after running the plugin.  It indicates that the host will not call
   `run()` again until another call to `activate()` and is mainly useful for more
   advanced plugins with ``live'' characteristics such as those with auxiliary
   processing threads.  As with `activate()`, this plugin has no use for this
   information so this method does nothing.

   This method is in the ``instantiation'' threading class, so no other
   methods on this instance will be called concurrently with it.
*/
static void
deactivate(LV2_Handle instance)
{
	int result = 0;
	const char* message;

	Amp* amp = (Amp*)instance;
	if (amp == NULL) {
		fprintf(stderr, "JK_DRIVE: deactivate() called with NULL instance parameter.\n");
		return;
	}

	/* First flush the up sample convertor */
	result = src_reset(amp->up_state);

	if (result != 0) {
		message = src_strerror(result);
		fprintf(stderr, "JK_DRIVE: Deactivate up reset error %s\n", message);
	}

	amp->up_in_level = 0;
	amp->process_level = 0;

	/* Now flush the down sample convertor */
	result = src_reset(amp->down_state);
	if (result) {
		message = src_strerror(result);
		fprintf(stderr, "JK_DRIVE: Deactivate down reset error %s\n", message);
	}

	amp->down_in_level = 0;
	amp->generate_in_level = 0;
}


/**
   Destroy a plugin instance (counterpart to `instantiate()`).

   This method is in the ``instantiation'' threading class, so no other
   methods on this instance will be called concurrently with it.
*/
static void
cleanup(LV2_Handle instance)
{
	Amp* amp = (Amp*)instance;

	if (amp == NULL) {
		fprintf(stderr, "JK_DRIVE: cleanup() called with NULL instance parameter.\n");
		return;
	}

	(void) src_delete(amp->up_state);
	(void) src_delete(amp->down_state);

	free(amp->up_input);
	free(amp->process_input);
	free(amp->process_output);
	free(amp->down_input);
	free(amp->generate_input);

	free(amp->up_data);
	free(amp->down_data);

	free(instance);
}

/**
   The `extension_data()` function returns any extension data supported by the
   plugin.  Note that this is not an instance method, but a function on the
   plugin descriptor.  It is usually used by plugins to implement additional
   interfaces.  This plugin does not have any extension data, so this function
   returns NULL.

   This method is in the ``discovery'' threading class, so no other functions
   or methods in this plugin library will be called concurrently with it.
*/
static const void*
extension_data(const char* uri)
{
	return NULL;
}

/**
   Every plugin must define an `LV2_Descriptor`.  It is best to define
   descriptors statically to avoid leaking memory and non-portable shared
   library constructors and destructors to clean up properly.
*/
static const LV2_Descriptor descriptor = {
	AMP_URI,
	instantiate,
	connect_port,
	activate,
	run,
	deactivate,
	cleanup,
	extension_data
};

/**
   The `lv2_descriptor()` function is the entry point to the plugin library.  The
   host will load the library and call this function repeatedly with increasing
   indices to find all the plugins defined in the library.  The index is not an
   indentifier, the URI of the returned descriptor is used to determine the
   identify of the plugin.

   This method is in the ``discovery'' threading class, so no other functions
   or methods in this plugin library will be called concurrently with it.
*/
LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
	switch (index) {
	case 0:  return &descriptor;
	default: return NULL;
	}
}
