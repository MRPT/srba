/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/* See srba-slam_main.cpp for docs */

#include "srba-run-generic-impl.h"

#include "CDatasetParser_Monocular.h"

template <>
struct InitializerSensorParams<srba::observations::MonocularCamera>
{
	template <class RBA>
	static void init(RBA &rba, RBASLAM_Params &config)
	{
		// Load params from file:

		if (!config.arg_sensor_params.isSet())
			throw std::runtime_error("Error: --sensor-params-cfg-file is mandatory for this type of observations.");

		const std::string sCfgFile = config.arg_sensor_params.getValue();
		rba.parameters.sensor.camera_calib.loadFromConfigFile("CAMERA",mrpt::utils::CConfigFile(sCfgFile) );
	}
};

// Specializations:
template <>
struct problem_settings_traits_t<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::MonocularCamera>  : public srba::RBA_SETTINGS_DEFAULT
{
	typedef kf2kf_poses::SE3                  kf2kf_pose_t;
	typedef landmarks::Euclidean3D            landmark_t;
	typedef observations::MonocularCamera     obs_t;

	// Camera sensors have a different coordinate system wrt the robot (rotated yaw=-90, pitch=0, roll=-90)
	typedef options::sensor_pose_on_robot_se3     sensor_pose_on_robot_t;
	typedef options::solver_LM_schur_dense_cholesky      solver_t;
};

// Explicit instantiation:
template struct RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::MonocularCamera>;


// Register this RBA problem:
RBA_Run_BasePtr my_creator_se3_lm3d_monocular(RBASLAM_Params &config)
{
	if (config.arg_se3.isSet() && config.arg_lm3d.isSet() && config.arg_obs.getValue()=="MonocularCamera")
		return RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::MonocularCamera>::create();

	return RBA_Run_BasePtr();
}

struct TMyRegister_se3_lm3d_monocular
{
	TMyRegister_se3_lm3d_monocular()
	{
		RBA_implemented_registry & reg = RBA_implemented_registry::getInstance();
		reg.doRegister( &my_creator_se3_lm3d_monocular, "--se3 --lm-3d --obs MonocularCamera" );
	}
};

static TMyRegister_se3_lm3d_monocular my_initializer_se3_lm3d_monocular;



