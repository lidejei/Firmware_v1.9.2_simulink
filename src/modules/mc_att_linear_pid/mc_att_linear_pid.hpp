#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <mathlib/mathlib.h>

// add the file from Utility
#include "Utility/PID.h"
#include "Utility/PID_private.h"
#include "Utility/PID.h"

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

extern "C" __EXPORT int mc_att_linear_pid_main(int argc, char *argv[]);
#define MAX_GYRO_COUNT 3

class MulticopterAttitudeAdvanceControl : public ModuleBase<MulticopterAttitudeAdvanceControl>
{
public:
	MulticopterAttitudeAdvanceControl();

	~MulticopterAttitudeAdvanceControl() {};

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterAttitudeAdvanceControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:

	/*
	 *Check and update the topic
	 */
	void		vehicle_attitude_poll();
	void 		vehicle_control_mode_poll();
	void		vehicle_attitude_setpoint_poll();
	void		sensor_correction_poll();
	void		vehicle_manual_poll();
	void		att_and_rates_conrtol();
	void		actuator_controls_publish();
	matrix::Vector3f 	get_rates_pid();
	void		att_rates_input_control(vehicle_attitude_s vehicle_attitude, vehicle_attitude_setpoint_s att_sp);

	int		_v_att_sub{-1};			/**< vehicle attitude subscription */
	int		_v_att_sp_sub{-1};		/**< vehicle attitude setpoint subscription */
	int		_sensor_correction_sub{-1};	/**< sensor thermal correction subscription */
	int		_manual_control_sp_sub{-1};	/**< manual control setpoint subscription */
	int		_v_control_mode_sub {-1};
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	orb_advert_t	_actuator_controls_pub{nullptr};		/**< rate setpoint publication */

	unsigned _gyro_count{1};
	int _selected_gyro{0};
	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */
	float _thrust_sp;				/**< thrust setpoint */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct sensor_correction_s		_sensor_correction {};	/**< sensor thermal corrections */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct sensor_gyro_s			_sensor_gyro {};	/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct actuator_controls_s		_actuators {};		/**< actuator controls */
	struct vehicle_control_mode_s	_v_control_mode {};
	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */
	perf_counter_t	_loop_perf;			/**< loop performance counter */

	PIDModelClass pid_control {};
};
