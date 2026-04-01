#ifndef VIRTUAL_WALL_H_
#define VIRTUAL_WALL_H_

#include <stdint.h>


/* ============================================
 * result code
 * ============================================ */

typedef enum {
    VW_OK = 0,
    VW_ERR_NULL,
    VW_ERR_BAD_ARG
} VW_Result;


/* ============================================
 * status mark
 * ============================================ */

 typedef enum {
    VW_FLAG_NONE                = 0u,
    VW_FLAG_POS_WALL_ACTIVE     = 1u << 0, /* positive wall is active */
    VW_FLAG_NEG_WALL_ACTIVE     = 1u << 1, /* negative wall is active */
    VW_FLAG_OUTPUT_SATURATED    = 1u << 2, /* output is limited */
    VW_FLAG_ESTIMATED_VEL       = 1u << 3  /* velocity is estimated by internal lib */
 } VW_FLAG;


 /* ============================================
 * configure
 * ============================================ */

 typedef struct 
 {
    int enabled;                            /* switch, 0 = disabled, 1 = enabled */
    int enable_pos_wall;
    int enable_neg_wall;
    int use_external_vel;                   /* 1 = use external velocity, 0 = estimated by internal lib */

    double pos_wall;                        /* position of positive wall */
    double neg_wall;                        /* position of negative wall */

    double spring;                          /* spring gain */
    double damping;                         /* damping gain */

    double output_limit;

    double velocity_lpf_tau;                /* the constant of the 1-order low-pass filter */
 } vw_config;


/* ============================================
 * internal status
 * ============================================ */
 
typedef struct{
    int init;

    double zero_offset;
    double last_pos_raw;
    double pos;

    double vel_raw;
    double vel;

    double spring_output;
    double damping_output;
    double wall_output;

    int flags;
} vw_state;


/* ============================================
 * handle
 * ============================================ */

 typedef struct
 {
    vw_config cfg;
    vw_state st;
 } vw_handle;


 /* ============================================
 * output
 * ============================================ */
 typedef struct {
    double position;
    double velocity;

    double penetration_pos;
    double penetration_neg;

    double spring_output;
    double damping_output;
    double wall_output;

    int flags;
} vw_output;
 

/* ============================================
 * API
 * ============================================ */

/* set the default configuration */
extern void vw_default_config(vw_config* cfg);

/* initialize the hanlde */
extern VW_Result vw_init(vw_handle* handle, const vw_config* cfg);

extern void vw_reset(vw_handle* handle);

extern VW_Result vw_set_zero(vw_handle* handle, double current_position_raw);

extern VW_Result vw_set_config(vw_handle* handle, const vw_config* cfg);

extern VW_Result vw_get_config(const vw_handle* handle, vw_config* cfg_out);

extern VW_Result vw_enable(vw_handle* handle, int enabled);

extern VW_Result vw_set_walls(vw_handle* handle, double neg_wall, double pos_wall);
 
extern VW_Result vw_set_gains(vw_handle* handle, double spring_gain, double damping_gain);

extern VW_Result vw_set_output_limit(vw_handle* handle, double limit);

extern VW_Result vw_update(
    vw_handle* handle,
    double position_raw,
    double velocity_raw,
    double dt,
    vw_output* out
);

extern VW_Result vw_update_no_velocity(
    vw_handle* handle,
    double position_raw,
    double dt,
    vw_output* out
);

extern VW_Result vw_get_wall_output(const vw_handle* handle, double* output);

extern VW_Result vw_get_state(const vw_handle* handle, vw_state* state_out);

#endif