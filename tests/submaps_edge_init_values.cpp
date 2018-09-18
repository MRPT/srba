/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <srba.h>
#include <mrpt/random.h>

#include <gtest/gtest.h>

using namespace srba;
using namespace std;
using mrpt::DEG2RAD;

struct RBA_OPTIONS : public RBA_OPTIONS_DEFAULT
{
//	typedef ecps::local_areas_fixed_size            edge_creation_policy_t;  //!< One of the most important choices: how to construct the relative coordinates graph problem
//	typedef options::sensor_pose_on_robot_none      sensor_pose_on_robot_t;  //!< The sensor pose coincides with the robot pose
	typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D>   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to some given matrix
//	typedef options::solver_LM_schur_dense_cholesky solver_t;                //!< Solver algorithm (Default: Lev-Marq, with Schur, with dense Cholesky)
};

typedef RbaEngine<
	kf2kf_poses::SE2,               // Parameterization  of KF-to-KF poses
	landmarks::RelativePoses2D,     // Parameterization of landmark positions
	observations::RelativePoses_2D, // Type of observations
	RBA_OPTIONS
	>  my_srba_t;

// -------------------------------------------------------------------------------------
// A test dataset: manually designed to challenge the way in which submaps are connected:
//  With the following pattern and a max. depth = 3, the observation of KF#1 from KF#11 
//  is a loop closure, which should raise a new kf2kf between 10<=>0. 
//  How to bootstrap the initial value of this edge, without any direct shared observations
//  between KFs #0 & #10?? This is what is tested here!
//
//   Center KF # 0
//      Center KF # 1
//   ...
//   Center KF # 5
//   ...
//   Center KF # 10
//      Center KF # 11  ==> Observes KF #1 (loop closure)
//   ...
//
// ------------------------------------------------------------------------------------
const double STD_NOISE_XY = 0.001;
const double STD_NOISE_YAW = DEG2RAD(0.05);

struct basic_graph_slam_dataset_entry_t
{
	unsigned int current_kf;
	unsigned int observed_kf;
	double x,y,yaw; // Relative pose of "observed_kf" as seen from "current_kf"
};
basic_graph_slam_dataset_entry_t dataset[] = {
 {     1,      0,     -1.0,      0.0,      0.0 },
 {     2,      1,     -1.0,      0.0,      0.0 },
 {     3,      2,     -1.0,      0.0,      0.0 },
 {     4,      3,     -1.0,      0.0,      0.0 },
 {     5,      4,     -1.0,      0.0,      0.0 },
 {     6,      5,     -1.0,      0.0,      0.0 },
 {     7,      6,     -1.0,      0.0,      0.0 },
 {     8,      7,     -1.0,      0.0,      0.0 },
 {     9,      8,     -1.0,      0.0,      0.0 },
 {    10,      9,     -1.0,      0.0,      0.0 },
 {    11,     10,     -1.0,      0.0,      0.0 },
 {    11,      1,   -10.05,      0.0,      0.0 },
 {    12,     11,     -1.0,      0.0,      0.0 },
 {    13,     12,     -1.0,      0.0,      0.0 },
 {    14,     13,     -1.0,      0.0,      0.0 },
 {    15,     14,     -1.0,      0.0,      0.0 },
 {    16,     15,     -1.0,      0.0,      0.0 },
};

TEST(MiniProblems,SubmapsEdgesInitValues)
{
	my_srba_t rba;     //  Create an empty RBA problem

	rba.get_time_profiler().disable();

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 0 );   // 0: None; 1:Important only; 2:Verbose

	// Information matrix for relative pose observations:
	{
		Eigen::Matrix3d ObsL;
		ObsL.setZero();
		ObsL(0,0) = 1/square(STD_NOISE_XY); // x
		ObsL(1,1) = 1/square(STD_NOISE_XY); // y
		ObsL(2,2) = 1/square(STD_NOISE_YAW); // phi

		// Set:
		rba.parameters.obs_noise.lambda = ObsL;
	}

	// =========== Topology parameters ===========
	rba.parameters.srba.max_tree_depth       = 
	rba.parameters.srba.max_optimize_depth   = 3;
	rba.parameters.ecp.submap_size          = 5;
	rba.parameters.ecp.min_obs_to_loop_closure = 1;
	// ===========================================

	// --------------------------------------------------------------------------------
	// Process the dataset:
	// --------------------------------------------------------------------------------
	const size_t nObs = sizeof(dataset)/sizeof(dataset[0]);
	size_t cur_kf = 0; // Start at keyframe #0 in the dataset

	for (size_t obsIdx = 0; obsIdx<nObs;  cur_kf++ /* move to next KF */  )
	{
		// Create list of observations for keyframe: "cur_kf"
		my_srba_t::new_kf_observations_t  list_obs;

		// To emulate graph-SLAM, each keyframe MUST have exactly ONE fixed "fake landmark", representing its pose:
		// ------------------------------------------------------------------------------------------------------------
		{
			my_srba_t::new_kf_observation_t obs_field;
			obs_field.is_fixed = true;
			obs_field.obs.feat_id = cur_kf; // Feature ID == keyframe ID
			obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
			obs_field.obs.obs_data.y = 0;
			obs_field.obs.obs_data.yaw = 0;
			list_obs.push_back( obs_field );
		}

		// The rest "observations" are real observations of relative poses:
		// -----------------------------------------------------------------
		while ( dataset[obsIdx].current_kf == cur_kf && obsIdx<nObs )
		{
			my_srba_t::new_kf_observation_t obs_field;
			obs_field.is_fixed = false;   // "Landmarks" (relative poses) have unknown relative positions (i.e. treat them as unknowns to be estimated)
			obs_field.is_unknown_with_init_val = false; // Ignored, since all observed "fake landmarks" already have an initialized value.

			obs_field.obs.feat_id      = dataset[obsIdx].observed_kf;
			obs_field.obs.obs_data.x   = dataset[obsIdx].x + mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.y   = dataset[obsIdx].y + mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.yaw = dataset[obsIdx].yaw  + mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_YAW);

			list_obs.push_back( obs_field );
			obsIdx++; // Next dataset entry
		}

		//  Here happens the main stuff: create Key-frames, build structures, run optimization, etc.
		//  ============================================================================================
		my_srba_t::TNewKeyFrameInfo new_kf_info;
		rba.define_new_keyframe(
			list_obs,      // Input observations for the new KF
			new_kf_info,   // Output info
			true           // Also run local optimization?
			);

	
		// Test condition on loop closure:
		if ( new_kf_info.created_edge_ids.size()==2 )
		{
			EXPECT_TRUE( new_kf_info.created_edge_ids[0].loopclosure_base_kf != SRBA_INVALID_KEYFRAMEID || new_kf_info.created_edge_ids[1].loopclosure_base_kf != SRBA_INVALID_KEYFRAMEID );
			EXPECT_TRUE( new_kf_info.created_edge_ids[0].loopclosure_observer_kf != SRBA_INVALID_KEYFRAMEID || new_kf_info.created_edge_ids[1].loopclosure_observer_kf != SRBA_INVALID_KEYFRAMEID );

			EXPECT_GT(new_kf_info.optimize_results.num_observations,1);
			EXPECT_LT(new_kf_info.optimize_results.obs_rmse, 1e-6);
		}

	} // end-for each dataset entry

}
