# Wrapper around C helper code
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging
import cffi


######################################################################
# c_helper.so compiling
######################################################################

COMPILE_CMD = ("gcc -Wall -g -O2 -shared -fPIC"
               " -flto -fwhole-program -fno-use-linker-plugin"
               " -o %s %s")
SOURCE_FILES = [
    'pyhelper.c', 'serialqueue.c', 'stepcompress.c', 'itersolve.c', 'trapq.c',
    'move.c', 'mathutil.c', 'kin_cartesian.c', 'kin_corexy.c', 'kin_delta.c',
    'kin_polar.c', 'kin_rotary_delta.c', 'kin_winch.c', 'kin_extruder.c',
    'planner_trapezoidal.c', 'planner_jerk.c'
]
DEST_LIB = "c_helper.so"
OTHER_FILES = [
    'list.h', 'serialqueue.h', 'stepcompress.h', 'itersolve.h', 'pyhelper.h',
    'trapq.h', 'mathutil.h'
]

defs_stepcompress = """
    struct stepcompress *stepcompress_alloc(uint32_t oid);
    void stepcompress_fill(struct stepcompress *sc, uint32_t max_error
        , uint32_t invert_sdir, uint32_t queue_step_msgid
        , uint32_t set_next_step_dir_msgid);
    void stepcompress_free(struct stepcompress *sc);
    int stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock);
    int stepcompress_queue_msg(struct stepcompress *sc
        , uint32_t *data, int len);

    struct steppersync *steppersync_alloc(struct serialqueue *sq
        , struct stepcompress **sc_list, int sc_num, int move_num);
    void steppersync_free(struct steppersync *ss);
    void steppersync_set_time(struct steppersync *ss
        , double time_offset, double mcu_freq);
    int steppersync_flush(struct steppersync *ss, uint64_t move_clock);
"""

defs_itersolve = """
    int32_t itersolve_generate_steps(struct stepper_kinematics *sk
        , double flush_time);
    double itersolve_check_active(struct stepper_kinematics *sk
        , double flush_time);
    int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);
    void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
    void itersolve_set_stepcompress(struct stepper_kinematics *sk
        , struct stepcompress *sc, double step_dist);
    double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
        , double x, double y, double z);
    void itersolve_set_position(struct stepper_kinematics *sk
        , double x, double y, double z);
    double itersolve_get_commanded_pos(struct stepper_kinematics *sk);
"""

defs_trapq = """
    void trapq_append(struct trapq *tq, double print_time
        , double accel_t, double cruise_t, double decel_t
        , double start_pos_x, double start_pos_y, double start_pos_z
        , double axes_r_x, double axes_r_y, double axes_r_z
        , double start_v, double cruise_v, double accel);
    struct trapq *trapq_alloc(void);
    void trapq_free(struct trapq *tq);
    void trapq_free_moves(struct trapq *tq, double print_time);
"""

defs_move = """
    struct move {
        double start_pos[4];
        double end_pos[4];
        double axes_d[4];
        double axes_r[4];
        double move_d;
        bool is_kinematic_move;
        double start_v;
        double cruise_v;
        double end_v;
        double start_a;
        double accel_t;
        double cruise_t;
        double decel_t;
        double jerk_t[7];
        double max_junction_v2;
        double max_start_v2;
        double max_smoothed_v2;
        double accel;
        double jerk;
        double max_cruise_v2;
        double delta_v2;
        double smooth_delta_v2;
        double min_move_t;
    };

    struct move_queue {
        unsigned int allocated_size;
        unsigned int first;
        unsigned int size;
    };

    struct move_queue* move_queue_alloc(unsigned int num_moves);
    void move_queue_free(struct move_queue *queue);
    struct move* move_reserve(
        double *start_pos,
        double *end_pos,
        double speed,
        double accel,
        double accel_to_decel,
        double jerk,
        struct move_queue* q);
    void move_commit(struct move_queue *queue);
    void move_queue_flush(struct move_queue *queue, unsigned int count);
    void limit_speed(struct move *m, double speed, double accel,
        double max_accel_to_decel);
    void calc_junction(struct move *m, struct move *prev_move,
        double junction_deviation, double extruder_instant_v);
    void set_trapezoidal_times(struct move *m, double distance, double start_v2,
        double cruise_v2, double end_v2, double accel);
    void calculate_trapezoidal(struct move* m, double start_v, double end_v);
    void calculate_jerk(struct move* m, double start_v, double end_v);
    double get_max_allowed_jerk_end_speed(double distance, double start_v,
        double end_v, double max_a, double jerk);
    bool can_accelerate_fully(double distance, double start_v, double end_v,
        double accel, double jerk);
"""

defs_kin_cartesian = """
    struct stepper_kinematics *cartesian_stepper_alloc(char axis);
"""

defs_kin_corexy = """
    struct stepper_kinematics *corexy_stepper_alloc(char type);
"""

defs_kin_delta = """
    struct stepper_kinematics *delta_stepper_alloc(double arm2
        , double tower_x, double tower_y);
"""

defs_kin_polar = """
    struct stepper_kinematics *polar_stepper_alloc(char type);
"""

defs_kin_rotary_delta = """
    struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height
        , double angle, double upper_arm, double lower_arm);
"""

defs_kin_winch = """
    struct stepper_kinematics *winch_stepper_alloc(double anchor_x
        , double anchor_y, double anchor_z);
"""

defs_kin_extruder = """
    struct stepper_kinematics *extruder_stepper_alloc(void);
    void extruder_set_smooth_time(struct stepper_kinematics *sk
        , double smooth_time);
"""

defs_serialqueue = """
    #define MESSAGE_MAX 64
    struct pull_queue_message {
        uint8_t msg[MESSAGE_MAX];
        int len;
        double sent_time, receive_time;
        uint64_t notify_id;
    };

    struct serialqueue *serialqueue_alloc(int serial_fd, int write_only);
    void serialqueue_exit(struct serialqueue *sq);
    void serialqueue_free(struct serialqueue *sq);
    struct command_queue *serialqueue_alloc_commandqueue(void);
    void serialqueue_free_commandqueue(struct command_queue *cq);
    void serialqueue_send(struct serialqueue *sq, struct command_queue *cq
        , uint8_t *msg, int len, uint64_t min_clock, uint64_t req_clock
        , uint64_t notify_id);
    void serialqueue_pull(struct serialqueue *sq
        , struct pull_queue_message *pqm);
    void serialqueue_set_baud_adjust(struct serialqueue *sq
        , double baud_adjust);
    void serialqueue_set_receive_window(struct serialqueue *sq
        , int receive_window);
    void serialqueue_set_clock_est(struct serialqueue *sq, double est_freq
        , double last_clock_time, uint64_t last_clock);
    void serialqueue_get_stats(struct serialqueue *sq, char *buf, int len);
    int serialqueue_extract_old(struct serialqueue *sq, int sentq
        , struct pull_queue_message *q, int max);
"""

defs_pyhelper = """
    void set_python_logging_callback(void (*func)(const char *));
    double get_monotonic(void);
"""

defs_std = """
    void free(void*);
"""

defs_planner_trapezoidal = """
    struct trapezoidal_planner*
    trapezoidal_planner_alloc(struct move_queue *queue);
    void trapezoidal_planner_free(struct trapezoidal_planner *planner);
    // Returns the number of moves flushed
    unsigned int trapezoidal_planner_flush(struct trapezoidal_planner *planner,
        bool lazy);
"""

defs_planner_jerk = """
    struct jerk_planner* jerk_planner_alloc(struct move_queue *queue);
    void jerk_planner_free(struct jerk_planner *planner);
    // Returns the number of moves flushed
    unsigned int jerk_planner_flush(struct jerk_planner *planner, bool lazy);
"""

defs_all = [
    defs_pyhelper, defs_serialqueue, defs_std,
    defs_stepcompress, defs_itersolve, defs_trapq, defs_move,
    defs_kin_cartesian, defs_kin_corexy, defs_kin_delta, defs_kin_polar,
    defs_kin_rotary_delta, defs_kin_winch, defs_kin_extruder,
    defs_planner_trapezoidal, defs_planner_jerk
]

# Return the list of file modification times
def get_mtimes(srcdir, filelist):
    out = []
    for filename in filelist:
        pathname = os.path.join(srcdir, filename)
        try:
            t = os.path.getmtime(pathname)
        except os.error:
            continue
        out.append(t)
    return out

# Check if the code needs to be compiled
def check_build_code(srcdir, target, sources, cmd, other_files=[]):
    src_times = get_mtimes(srcdir, sources + other_files)
    obj_times = get_mtimes(srcdir, [target])
    if not obj_times or max(src_times) > min(obj_times):
        logging.info("Building C code module %s", target)
        srcfiles = [os.path.join(srcdir, fname) for fname in sources]
        destlib = os.path.join(srcdir, target)
        # Make sure we don't use an out of date library
        if os.path.isfile(destlib):
            os.remove(destlib)
        res = os.system(cmd % (destlib, ' '.join(srcfiles)))
        if res:
            msg = "Unable to build C code module (error=%s)" % (res,)
            logging.error(msg)
            raise Exception(msg)

FFI_main = None
FFI_lib = None
pyhelper_logging_callback = None

# Return the Foreign Function Interface api to the caller
def get_ffi():
    global FFI_main, FFI_lib, pyhelper_logging_callback
    if FFI_lib is None:
        srcdir = os.path.dirname(os.path.realpath(__file__))
        check_build_code(srcdir, DEST_LIB, SOURCE_FILES, COMPILE_CMD
                         , OTHER_FILES)
        FFI_main = cffi.FFI()
        for d in defs_all:
            FFI_main.cdef(d)
        FFI_lib = FFI_main.dlopen(os.path.join(srcdir, DEST_LIB))
        # Setup error logging
        def logging_callback(msg):
            logging.error(FFI_main.string(msg))
        pyhelper_logging_callback = FFI_main.callback(
            "void(const char *)", logging_callback)
        FFI_lib.set_python_logging_callback(pyhelper_logging_callback)
    return FFI_main, FFI_lib


######################################################################
# hub-ctrl hub power controller
######################################################################

HC_COMPILE_CMD = "gcc -Wall -g -O2 -o %s %s -lusb"
HC_SOURCE_FILES = ['hub-ctrl.c']
HC_SOURCE_DIR = '../../lib/hub-ctrl'
HC_TARGET = "hub-ctrl"
HC_CMD = "sudo %s/hub-ctrl -h 0 -P 2 -p %d"

def run_hub_ctrl(enable_power):
    srcdir = os.path.dirname(os.path.realpath(__file__))
    hubdir = os.path.join(srcdir, HC_SOURCE_DIR)
    check_build_code(hubdir, HC_TARGET, HC_SOURCE_FILES, HC_COMPILE_CMD)
    os.system(HC_CMD % (hubdir, enable_power))


if __name__ == '__main__':
    get_ffi()
