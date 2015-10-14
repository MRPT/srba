/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/utils/CConfigFileBase.h> // MRPT_LOAD_CONFIG_VAR

namespace srba {
namespace ecps {

/** Edge creation policy: A linear graph where each KF is always connected to the precedding one. 
    * Occasional loop closure edges are also inserted depending on thresholding parameters related to 
	* the number of observations whose base KF is too distant.
    *
    * \ingroup mrpt_srba_ecps
	*/
struct classic_linear_rba
{
	struct parameters_t
	{
		size_t              min_obs_to_loop_closure; //!< Default:4, reduce to 1 for relative graph-slam
		size_t              min_dist_for_loop_closure; //!< Default: 4. Ideally, set to SRBA param `max_optimize_depth`+1

		/** Ctor for default values */
		parameters_t() :
			min_obs_to_loop_closure ( 4 ),
			min_dist_for_loop_closure ( 4 )
		{ }

		/** See docs of mrpt::utils::CLoadableOptions */
		void loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section)
		{
			MRPT_LOAD_CONFIG_VAR(min_obs_to_loop_closure,uint64_t,source,section)
			MRPT_LOAD_CONFIG_VAR(min_dist_for_loop_closure,uint64_t,source,section)
		}

		/** See docs of mrpt::utils::CLoadableOptions */
		void saveToConfigFile(mrpt::utils::CConfigFileBase & out,const std::string & section) const
		{
			out.write(section,"min_obs_to_loop_closure",static_cast<uint64_t>(min_obs_to_loop_closure), /* text width */ 30, 30, "Min. num. of covisible observations to add a loop closure edge");
			out.write(section,"min_dist_for_loop_closure",static_cast<uint64_t>(min_dist_for_loop_closure), /* text width */ 30, 30, "Min. topological distance to observed features base to create LC edges.Ideally = max_optimize_depth+1");
		}
	};
	
	/** Implements the edge-creation policy. 
	 * \tparam traits_t Use rba_joint_parameterization_traits_t<kf2kf_pose_t,landmark_t,obs_t>
	 */
	template <class traits_t,class rba_engine_t>
	void eval(
		const TKeyFrameID               new_kf_id,
		const typename traits_t::new_kf_observations_t   & obs,
		std::vector<TNewEdgeInfo> &new_k2k_edge_ids,
		rba_engine_t       & rba_engine,
		const parameters_t &params)
	{
		using namespace std;
		ASSERT_(new_kf_id>=1)

		// The simplest policy: Always add a single edge (n-1) => (n)
		const typename traits_t::original_kf2kf_pose_t::pose_t   init_inv_pose;

		TNewEdgeInfo nei;
		nei.has_approx_init_val = true; // In a linear graph it's a reasonable approx. to make each KF start at the last KF pose, which is what means a null pose init val.

		nei.id = rba_engine.create_kf2kf_edge(new_kf_id, TPairKeyFrameID( new_kf_id-1, new_kf_id), obs, init_inv_pose);

		new_k2k_edge_ids.push_back(nei);

	} // end of eval()

private:
	std::set<std::pair<TKeyFrameID,TKeyFrameID> > central2central_connected_areas; // 1st the lowest id, to avoid duplicates
	std::set<size_t> last_timestep_touched_kfs;

};  // end of struct

} } // End of namespaces
