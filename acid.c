#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SRATE 48000.0
#define BPM 140
void dump(float left, float right);

typedef double phase_t;

bool phase_inc(phase_t *p, double freq) {
	bool cycled = false;
	*p += freq / SRATE;
	if (*p >= 1.0) {
		*p = fmod(*p, 1.0);
		cycled = true;
	}
	return cycled;
}

enum env_stage {
	ENV_RELEASE,
	ENV_ATTACK,
	ENV_DECAY,
};

struct env {
	enum env_stage stage;
	double current;
	bool last_gate;

	double attack;
	double decay;
	double release;
};

double env_tick(struct env *e, bool gate, bool accent, bool flt)
{
	double out = e->current;

	if (gate && !e->last_gate) {
		e->stage = ENV_ATTACK;
	} else if (!gate) {
		e->stage = ENV_RELEASE;
	}
	e->last_gate = gate;

	switch (e->stage) {
	case ENV_RELEASE:
		e->current = fmax(e->current - e->release, 0.0);
		break;
	case ENV_ATTACK:
		if (accent && flt) {
			e->current = fmin(e->current + 50.0 / SRATE, 1.0);
		} else {
			e->current = fmin(e->current + e->attack, 1.0);
		}
		if (e->current >= 1.0) {
			e->stage = ENV_DECAY;
		}
		break;
	case ENV_DECAY:
		if (accent && flt) {
			e->current = fmax(e->current - 2.0 / SRATE, 0.0);
		} else {
			e->current = fmax(e->current - e->decay, 0.0);
		}
		break;
	}

	return out;
}

double p2f(double pitch)
{
	return 440.0 * pow(2.0, pitch / 12.0);
}

struct voice {
	phase_t base;
	phase_t res;

	struct env amp;
	struct env filter;

	double feedback;
	double cutoff;
	double last_out;
	double env_amt;
};

double voice_tick(struct voice *v, double pitch, bool gate, bool accent) {
	double freq = p2f(pitch);
	double amp = (accent ? 1.0 : 0.7) * env_tick(&v->amp, gate, accent, false);
	double scale = 1.0 - v->base;
	double out = amp * scale * sin(M_PI * 2.0 * (v->res + v->feedback * v->last_out));
	double cycled = phase_inc(&v->base, freq);
	double fe = env_tick(&v->filter, gate, accent, true);
	double cutoff = p2f(v->cutoff + fe * (accent ? 1.5 : 1.0) * v->env_amt);

	phase_inc(&v->res, cutoff);
	if (cycled) {
		v->res = 0.0;
	}

	v->last_out = out;

	return out;
}

struct step303 {
	int pitch;
	bool accent;
	bool gate;
	bool up;
	bool down;
	bool slide;
};

struct seq303 {
	struct step303 steps[16];
	struct step303 last_step;
	int nsteps;
	int current_idx;
	double basepitch;
	bool last_clock;
};

double step_pitch(struct step303 step, double base)
{
	return step.pitch + step.up * 12.0 - step.down * 12.0 + base;
}

void tick_seq303(struct seq303 *s, phase_t phase, double *pitch, bool *gate, bool *accent)
{
	struct step303 step = s->steps[s->current_idx];
	double slide_phase = 2.0 * fmin(phase, 0.5);
	double cur_pitch = step_pitch(step, s->basepitch);
	double last_pitch = step_pitch(s->last_step, s->basepitch);
	double clock = phase < 0.5;

	if (s->last_step.slide) {
		*pitch = last_pitch + (cur_pitch - last_pitch) * slide_phase;
	} else {
		*pitch = cur_pitch;
	}
	*gate = step.gate && (phase < 0.5 || step.slide);
	*accent = step.accent;

	if (clock && !s->last_clock) {
		s->last_step = s->steps[s->current_idx];
		s->current_idx = (s->current_idx + 1) % s->nsteps;
	}
	s->last_clock = clock;
}

struct bassdrum {
	phase_t car;
	phase_t mod;
	double basepitch;
	struct env mod_env;
	struct env car_env;
	struct env pitch_env;
	double pitch_depth;
};

double dist(double value, double level)
{
	return fmin(fmax(-level, value), level) / level;
}
double bassdrum_tick(struct bassdrum *b, bool gate)
{
	double mod_env = 0.3 * env_tick(&b->mod_env, gate, false, false);
	double mod = mod_env * sin(M_PI * 2.0 * b->mod);
	double car_env = env_tick(&b->car_env, gate, false, false);
	double car = car_env * car_env * sin(M_PI * 2.0 * 2.0 * (b->car + mod));
	double pitch_env = b->pitch_depth * env_tick(&b->pitch_env, gate, false, false);

	phase_inc(&b->car, p2f(b->basepitch + pitch_env));
	phase_inc(&b->mod, p2f(b->basepitch + pitch_env));
	return dist(car, 0.4);
}

struct hihat {
	phase_t phase;
	double last_val;
	struct env env;
	double feedback;
	double pitch;
};

double hihat_tick(struct hihat *h, bool ch, bool oh)
{
	double env, out;
	double val = sin(M_PI * 2.0 * (h->phase + h->last_val * h->feedback));

	if (oh) {
		h->env.attack = 1.0;
		h->env.decay = 8.0 / SRATE;
		h->env.release = 8.0 / SRATE;
	} else if (ch) {
		h->env.attack = 1.0;
		h->env.decay = 32.0 / SRATE;
		h->env.release = 32.0 / SRATE;
	}
	env = env_tick(&h->env, ch || oh, false, false);
	out = val * env * env;
	h->last_val = val;
	phase_inc(&h->phase, p2f(h->pitch));
	return out;
}

struct song_step {
	bool bd_on;
	bool ch_on;
	bool oh_on;

	double c_set;
	double c_add;
};

#define DELAY_LENGTH (((int)SRATE) / 3)
struct delay {
	double buffer[DELAY_LENGTH];
	int idx;
	int length;
	double fb;
};

double delay_tick(struct delay *d, double in)
{
	double delayed = d->buffer[d->idx];
	d->buffer[d->idx] = in + d->fb * delayed;
	d->idx = (d->idx + 1) % d->length;
	return delayed;
}

int main(int argc, char **argv)
{
	phase_t measure_phase = 0.0;

	struct song_step song[] = {
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_set = -24.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, },
		{ .bd_on = false, .ch_on = false, .oh_on = false, },
		{ .bd_on = false, .ch_on = false, .oh_on = false, },

		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_set = -24.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, },
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 4.0},
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = -4.0},

		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_set = -24.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, },
		{ .bd_on = false, .ch_on = false, .oh_on = false,},
		{ .bd_on = false, .ch_on = false, .oh_on = false,},

		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 4.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 4.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 4.0 },
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 4.0 },

		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_set = -20.0 },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },

		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = 1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = 1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = 1.0},
		{ .bd_on = false, .ch_on = false, .oh_on = false, .c_add = 8.0},

		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = -8.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = -2.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, },
		{ .bd_on = true, .ch_on = true, .oh_on = false, },

		{ .bd_on = true, .ch_on = true, .oh_on = false, },
		{ .bd_on = true, .ch_on = true, .oh_on = false, },
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = 16.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = -16.0},

		{ .bd_on = true, .ch_on = true, .oh_on = false, },
		{ .bd_on = true, .ch_on = true, .oh_on = false, },
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = 20.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = -20.0},

		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = 2.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = 2.0},
		{ .bd_on = true, .ch_on = true, .oh_on = false, .c_add = 2.0},
		{ .bd_on = false, .ch_on = true, .oh_on = false, .c_add = 2.0 },

		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = -8.0 },
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 2.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 2.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 2.0},

		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 1.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 1.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 1.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = 1.0},

		{ .bd_on = false, .ch_on = false, .oh_on = true, .c_add = 2.0},
		{ .bd_on = false, .ch_on = false, .oh_on = true, .c_add = 2.0},
		{ .bd_on = false, .ch_on = false, .oh_on = true, .c_add = 2.0},
		{ .bd_on = false, .ch_on = false, .oh_on = true, .c_add = 2.0},

		{ .bd_on = false, .ch_on = true, .oh_on = true, },
		{ .bd_on = false, .ch_on = true, .oh_on = true, },
		{ .bd_on = false, .ch_on = true, .oh_on = true, },
		{ .bd_on = false, .ch_on = true, .oh_on = true, .c_add = -8.0},

		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_set = 30.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },

		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = -8.0 },

		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_set = 30.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },

		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = -8.0 },

		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},

		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -1.0},

		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_set = 25.0},
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },

		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, },
		{ .bd_on = true, .ch_on = false, .oh_on = false, .c_add = -8.0 },

		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_set = 25.0},
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },

		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, },
		{ .bd_on = true, .ch_on = true, .oh_on = true, .c_add = -8.0 },
	};
	int song_len = sizeof (song) / sizeof (song[0]);
	int song_step = 0;

	struct delay dl = {.fb = 0.2, .length = DELAY_LENGTH};
	struct delay dr = {.fb = 0.2, .length = 9 * DELAY_LENGTH / 10};
	struct voice v = {
		.cutoff = -24.0,
		.env_amt = 10.0,
		.feedback = 0.25,
		.amp = {
			.attack = 1.0,
			.decay = 1.0 / SRATE,
			.release = 20.0 / SRATE,
		},
		.filter = {
			.attack = 1.0,
			.decay = 1.0 / SRATE,
			.release = 20.0 / SRATE,
		},
	};

	struct bassdrum bd  = {
		.mod_env = {
			.attack = 1.0,
			.decay = 64.0 / SRATE,
			.release = 64.0 / SRATE,
		},
		.car_env = {
			.attack = 1.0,
			.decay = 4.0 / SRATE,
			.release = 4.0 / SRATE,
		},
		.pitch_env = {
			.attack = 1.0,
			.decay = 20.0 / SRATE,
			.release = 20.0 / SRATE,
		},
		.pitch_depth = 25.0,
		.basepitch = -50.0,
	};

	struct seq303 seq = {
		.basepitch = -24.0,
		.steps = {
			{ .pitch = 0, .accent = true, .gate = true, .slide = false },
			{ .pitch = 0, .accent = false, .gate = true, .slide = false, .down = true, },
			{ .pitch = 0, .accent = false, .gate = true, .slide = false },
			{ .pitch = 3, .accent = true, .gate = true, .slide = false },
			{ .pitch = 0, .accent = false, .gate = true, .slide = true, .up = true, },
			{ .pitch = 0, .accent = false, .gate = true, .slide = false },
			{ .pitch = 3, .accent = false, .gate = true, .slide = true },
			{ .pitch = 0, .accent = true, .gate = true, .slide = false, .down = true, },
			{ .pitch = 0, .accent = false, .gate = true, .slide = false },
		},
		.nsteps = 8,
		.current_idx = 0,
		.last_clock = true,
	};

	struct hihat hh = {
		.pitch = 40.0,
		.feedback = 0.48,
	};

	double cutoff_add = 0.0;
	for (;;) {
		bool gate = false;
		bool accent = false;
		double pitch = 0.0;

		double out = 0.0;
		double left = 0.0;
		double right = 0.0;

		double step_phase = fmod(16.0 * measure_phase, 1.0);
		double ampscale = 1.0 - 0.9 * bd.car_env.current;
		bool bd_gate, ch_gate, oh_gate;

		if (phase_inc(&measure_phase, 0.25 * BPM / 60.0)) {
			song_step = (song_step + 1) % song_len;
			if (song[song_step].c_set != 0.0) {
				v.cutoff = song[song_step].c_set;
			}
		}
		cutoff_add = song[song_step].c_add;

		tick_seq303(&seq, step_phase, &pitch, &gate, &accent);

		bd_gate = song[song_step].bd_on && ((seq.current_idx % 4) == 0) && step_phase < 0.5;
		oh_gate = song[song_step].oh_on && (((seq.current_idx + 2) % 4) == 0) && step_phase < 0.5;
		ch_gate = song[song_step].ch_on && step_phase < 0.5;

		out += ampscale * 0.8 * voice_tick(&v, pitch, gate, accent);
		left += ampscale * 0.2 * delay_tick(&dl, out);
		right += ampscale * 0.2 * delay_tick(&dr, out);
		out += bassdrum_tick(&bd, bd_gate);
		out += ampscale * 0.5 * hihat_tick(&hh, ch_gate, oh_gate);

		dump(left + out, right + out);
		v.cutoff += cutoff_add / SRATE;
	}
	return 0;
}

union out {
	float sample;
	uint8_t bytes[4];
};

void dump(float left, float right)
{
	union out o;
	o.sample = dist(left, 1.0);
	putchar(o.bytes[0]);
	putchar(o.bytes[1]);
	putchar(o.bytes[2]);
	putchar(o.bytes[3]);
	o.sample = dist(right, 1.0);
	putchar(o.bytes[0]);
	putchar(o.bytes[1]);
	putchar(o.bytes[2]);
	putchar(o.bytes[3]);
}
