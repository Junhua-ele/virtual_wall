#include "virtual_wall.h"
#include <string.h>


/* ============================================
 * internal function
 * ============================================ */

static double vw_clamp(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static double vw_abs(double x) {
    return (x >= 0) ? x : -x;
}

static int vw_validate_config(const vw_config* cfg) {
    if (cfg == 0) {
        return 0;
    }

    if (cfg->pos_wall <= cfg->neg_wall) {
        return 0;
    }

    if (cfg->spring < 0) {
        return 0;
    }

    if (cfg->damping < 0) {
        return 0;
    }

    if (cfg->output_limit <= 0) {
        return 0;
    }

    return 1;
}

static double vw_lpf_1st(double prev, double x, double dt, double tau)
{
    if(dt <= 0)
        return prev;
    
    if(tau <= 0)
        return x;

    double alpha = dt / (dt + tau);

    return prev + alpha * (x - prev);
}

static void vw_clear_runtime(vw_state* st) {
    if (st == 0) {
        return;
    }

    st->init       = 0;
    st->last_pos_raw = 0;
    st->pos          = 0;
    st->vel_raw      = 0;
    st->vel          = 0;
    st->spring_output     = 0;
    st->damping_output    = 0;
    st->wall_output       = 0;
    st->flags             = VW_FLAG_NONE;
}

static VW_Result vw_update_core(
    vw_handle* handle,
    double pos_raw,
    double vel_raw,
    double dt,
    int vel_estimated,
    vw_output* out
)
{
    double pos;
    double vel;
    double spring_out = 0;
    double damping_out = 0;
    double total_out = 0;
    double pen_pos = 0;
    double pen_neg = 0;
    int flag = VW_FLAG_NONE;

    if(handle == 0)
        return VW_ERR_NULL;
    
    if(dt <= 0)
        return VW_ERR_BAD_ARG;
    
    if(!handle->st.init)
    {
        handle->st.init = 1;
        handle->st.last_pos_raw = pos_raw;
        handle->st.vel_raw = 0;
        handle->st.vel = 0;
    }

    pos = pos_raw - handle->st.zero_offset;
    vel = vw_lpf_1st(handle->st.vel, vel_raw, dt, handle->cfg.velocity_lpf_tau);

    handle->st.pos = pos;
    handle->st.vel_raw = vel_raw;
    handle->st.vel = vel;
    handle->st.flags  = VW_FLAG_NONE;

    if (vel_estimated) {
        flag |= VW_FLAG_ESTIMATED_VEL;
    }

    if(handle->cfg.enabled)
    {
        /* positive wall */
        if(handle->cfg.enable_pos_wall && (pos > handle->cfg.pos_wall))
        {
            double inward_vel = (vel > 0) ? (vel) : 0;

            pen_pos = pos - handle->cfg.pos_wall;
            spring_out += -handle->cfg.spring  * pen_pos;
            damping_out += -handle->cfg.damping * inward_vel;
            flag |= VW_FLAG_POS_WALL_ACTIVE;
        }

        /* negative wall */
        if(handle->cfg.enable_neg_wall && (pos < handle->cfg.neg_wall))
        {
            double inward_vel = (vel < 0) ? (-vel) : 0;

            pen_pos = handle->cfg.neg_wall - pos;
            spring_out += handle->cfg.spring  * pen_pos;
            damping_out += handle->cfg.damping * inward_vel;
            flag |= VW_FLAG_NEG_WALL_ACTIVE;
        }
    }

    total_out = spring_out + damping_out;
    if (vw_abs(total_out) > handle->cfg.output_limit) {
        total_out = vw_clamp(
            total_out,
            -handle->cfg.output_limit,
             handle->cfg.output_limit
        );
        flag |= VW_FLAG_OUTPUT_SATURATED;
    }

    handle->st.spring_output  = spring_out;
    handle->st.damping_output = damping_out;
    handle->st.wall_output    = total_out;
    handle->st.flags          = flag;
    handle->st.last_pos_raw = pos_raw;

    if (out != 0) {
        out->position        = pos;
        out->velocity        = vel;
        out->penetration_pos = pen_pos;
        out->penetration_neg = pen_neg;
        out->spring_output   = spring_out;
        out->damping_output  = damping_out;
        out->wall_output     = total_out;
        out->flags           = flag;
    }

    return VW_OK;
}


/* ============================================
 * API
 * ============================================ */

void vw_default_config(vw_config* cfg) {
    if (cfg == 0) {
        return;
    }

    cfg->enabled               = 1;
    cfg->enable_pos_wall       = 1;
    cfg->enable_neg_wall       = 1;
    cfg->use_external_vel      = 1;

    cfg->pos_wall              = 1.0;
    cfg->neg_wall              = -1.0;

    cfg->spring           = 1.0;
    cfg->damping          = 0.05;

    cfg->output_limit          = 5.0;
    cfg->velocity_lpf_tau      = 0.01;
}

VW_Result vw_init(vw_handle* handle, const vw_config* cfg) {
    if (handle == 0 || cfg == 0) {
        return VW_ERR_NULL;
    }

    if (!vw_validate_config(cfg)) {
        return VW_ERR_BAD_ARG;
    }

    memset(handle, 0, sizeof(*handle));
    handle->cfg = *cfg;
    handle->st.zero_offset = 0;
    vw_clear_runtime(&handle->st);

    return VW_OK;
}

void vw_reset(vw_handle* handle) {
    if (handle == 0) {
        return;
    }

    {
        double zero = handle->st.zero_offset;
        vw_clear_runtime(&handle->st);
        handle->st.zero_offset = zero;
    }
}

VW_Result vw_set_zero(vw_handle* handle, double current_position_raw) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    handle->st.zero_offset = current_position_raw;
    vw_reset(handle);
    handle->st.zero_offset = current_position_raw;
    return VW_OK;
}

VW_Result vw_set_config(vw_handle* handle, const vw_config* cfg) {
    if (handle == 0 || cfg == 0) {
        return VW_ERR_NULL;
    }

    if (!vw_validate_config(cfg)) {
        return VW_ERR_BAD_ARG;
    }

    handle->cfg = *cfg;
    return VW_OK;
}

VW_Result vw_get_config(const vw_handle* handle, vw_config* cfg_out) {
    if (handle == 0 || cfg_out == 0) {
        return VW_ERR_NULL;
    }

    *cfg_out = handle->cfg;
    return VW_OK;
}

VW_Result vw_enable(vw_handle* handle, int enabled) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    handle->cfg.enabled = (enabled != 0) ? 1 : 0;
    return VW_OK;
}

VW_Result vw_set_walls(vw_handle* handle, double neg_wall, double pos_wall) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    if (pos_wall <= neg_wall) {
        return VW_ERR_BAD_ARG;
    }

    handle->cfg.neg_wall = neg_wall;
    handle->cfg.pos_wall = pos_wall;
    return VW_OK;
}

VW_Result vw_set_gains(vw_handle* handle, double spring_gain, double damping_gain) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    if (spring_gain < 0 || damping_gain < 0) {
        return VW_ERR_BAD_ARG;
    }

    handle->cfg.spring  = spring_gain;
    handle->cfg.damping = damping_gain;
    return VW_OK;
}

VW_Result vw_set_output_limit(vw_handle* handle, double limit) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    if (limit <= 0) {
        return VW_ERR_BAD_ARG;
    }

    handle->cfg.output_limit = limit;
    return VW_OK;
}

VW_Result vw_update(
    vw_handle* handle,
    double position_raw,
    double velocity_raw,
    double dt,
    vw_output* out
) {
    if (handle == 0) {
        return VW_ERR_NULL;
    }

    handle->cfg.use_external_vel = 1;
    return vw_update_core(handle, position_raw, velocity_raw, dt, 0, out);
}

VW_Result vw_update_no_velocity(
    vw_handle* handle,
    double position_raw,
    double dt,
    vw_output* out
) {
    double vel_est = 0;

    if (handle == 0) {
        return VW_ERR_NULL;
    }

    if (dt <= 0) {
        return VW_ERR_BAD_ARG;
    }

    if (!handle->st.init) {
        handle->st.init = 1;
        handle->st.last_pos_raw = position_raw;
        vel_est = 0;
    } else {
        vel_est = (position_raw - handle->st.last_pos_raw) / dt;
    }

    handle->cfg.use_external_vel = 0;
    return vw_update_core(handle, position_raw, vel_est, dt, 1, out);
}

VW_Result vw_get_wall_output(const vw_handle* handle, double* output) {
    if (handle == 0 || output == 0) {
        return VW_ERR_NULL;
    }

    *output = handle->st.wall_output;
    return VW_OK;
}

VW_Result vw_get_state(const vw_handle* handle, vw_state* state_out) {
    if (handle == 0 || state_out == 0) {
        return VW_ERR_NULL;
    }

    *state_out = handle->st;
    return VW_OK;
}

int example(void) {
    vw_handle wall;
    vw_config cfg;
    vw_output out;
    VW_Result ret;

    vw_default_config(&cfg);

    /* Please change the parameters */
    cfg.pos_wall =  1.2f;   /* +1.2 rad */
    cfg.neg_wall = -1.2f;   /* -1.2 rad */
    cfg.spring = 2.0f;
    cfg.damping = 0.08f;
    cfg.output_limit = 3.0f;
    cfg.velocity_lpf_tau = 0.01f;

    ret = vw_init(&wall, &cfg);
    if (ret != VW_OK) {
        printf("vw_init failed: %d\n", ret);
        return -1;
    }

    /* set the current pos as zero */
    vw_set_zero(&wall, 0.0f);

    while(1)
    {
        float dt = 0.001f; /* 1 kHz */
        float position = 0.0f;
        float velocity = 0.0f;

        int i;
        for (i = 0; i < 3000; ++i) {
            if (i < 1500) {
                velocity = 1.0f; /* rad/s */
            } else {
                velocity = 0.0f;
            }

            position += velocity * dt;

            /* use external velocity */
            ret = vw_update(&wall, position, velocity, dt, &out);
            if (ret != VW_OK) {
                printf("vw_update failed: %d\n", ret);
                break;
            }

            /* the final output */
            float user_cmd = 0.0f;
            float final_cmd = user_cmd + out.wall_output;

            if ((i % 100) == 0) {
                printf("t=%.3f, pos=%.3f, vel=%.3f, pen+=%.3f, wall=%.3f, final=%.3f, flags=0x%02X\n",
                        i * dt,
                        out.position,
                        out.velocity,
                        out.penetration_pos,
                        out.wall_output,
                        final_cmd,
                        (unsigned int)out.flags);
            }
        }
    }

    return 0;
}