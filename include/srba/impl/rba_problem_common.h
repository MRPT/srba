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
#include <mrpt/math/ops_containers.h> // meanAndStd()

namespace srba {

/** Default constructor */
template <class RBA_SETTINGS_T>
RbaEngine<RBA_SETTINGS_T>::RbaEngine() :
	rba_state(),
	m_profiler(true)
{
	clear();
}

/** Reset the entire problem to an empty state (automatically called at construction) */
template <class RBA_SETTINGS_T>
void RbaEngine<RBA_SETTINGS_T>::clear()
{
	this->rba_state.clear();
}

template <class RBA_SETTINGS_T>
RbaEngine<RBA_SETTINGS_T>::TSRBAParameters::TSRBAParameters() :
	// -------------------------------
	max_tree_depth       ( 4 ),
	max_optimize_depth   ( 4 ),
	// -------------------------------
	optimize_new_edges_alone (true),
	use_robust_kernel    ( false ),
	use_robust_kernel_stage1 ( false ),
	kernel_param         ( 3. ),
	max_iters            ( 20 ),
	max_error_per_obs_to_stop    ( 1e-6 ),
	max_rho              ( 10.0 ),
	max_lambda           ( 1e20 ),
	min_error_reduction_ratio_to_relinearize ( 0.01 ),
	numeric_jacobians    ( false ),
	feedback_user_iteration(NULL),
	compute_condition_number(false),
	cov_recovery         ( crpLandmarksApprox )
{
}

/** See docs of mrpt::utils::CLoadableOptions */
template <class RBA_SETTINGS_T>
void RbaEngine<RBA_SETTINGS_T>::TSRBAParameters::loadFromConfigFile(const mrpt::utils::CConfigFileBase & source,const std::string & section)
{
	MRPT_LOAD_CONFIG_VAR(max_tree_depth,uint64_t,source,section)
	MRPT_LOAD_CONFIG_VAR(max_optimize_depth,uint64_t,source,section)

	MRPT_LOAD_CONFIG_VAR(optimize_new_edges_alone,bool,source,section)
	MRPT_LOAD_CONFIG_VAR(use_robust_kernel,bool,source,section)
	MRPT_LOAD_CONFIG_VAR(use_robust_kernel_stage1,bool,source,section)
	MRPT_LOAD_CONFIG_VAR(max_rho,double,source,section)
	MRPT_LOAD_CONFIG_VAR(max_lambda,double,source,section)
	MRPT_LOAD_CONFIG_VAR(kernel_param,double,source,section)
	MRPT_LOAD_CONFIG_VAR(max_iters,uint64_t,source,section)
	MRPT_LOAD_CONFIG_VAR(max_error_per_obs_to_stop,double,source,section)

	cov_recovery = source.read_enum(section, "cov_recovery", cov_recovery);
}

/** See docs of mrpt::utils::CLoadableOptions */
template <class RBA_SETTINGS_T>
void RbaEngine<RBA_SETTINGS_T>::TSRBAParameters::saveToConfigFile(mrpt::utils::CConfigFileBase & out,const std::string & section) const
{
	out.write(section,"max_tree_depth",max_tree_depth,  /* text width */ 30, 30, "Maximum depth of all spanning trees");
	out.write(section,"max_optimize_depth",max_optimize_depth, /* text width */ 30, 30, "Max. local optimization distance");

	out.write(section,"optimize_new_edges_alone",optimize_new_edges_alone,  /* text width */ 30, 30, "Optimize new edges alone before optimizing the entire local area?");
	out.write(section,"use_robust_kernel",use_robust_kernel,  /* text width */ 30, 30, "Use pseudo-Huber kernel?");
	out.write(section,"use_robust_kernel_stage1",use_robust_kernel_stage1,  /* text width */ 30, 30, "Use pseudo-Huber kernel at stage1?");
	out.write(section,"kernel_param",kernel_param,  /* text width */ 30, 30, "robust kernel parameter");
	out.write(section,"max_rho",max_rho,  /* text width */ 30, 30, "Lev-Marq optimization: maximum rho value to stop");
	out.write(section,"max_lambda",max_lambda,  /* text width */ 30, 30, "Lev-Marq optimization: maximum lambda to stop");
	out.write(section,"max_iters",static_cast<uint64_t>(max_iters),  /* text width */ 30, 30, "Max. iterations for optimization");
	out.write(section,"max_error_per_obs_to_stop",max_error_per_obs_to_stop,  /* text width */ 30, 30, "Another criterion for stopping optimization");
	out.write(section,"cov_recovery", mrpt::utils::TEnumType<TCovarianceRecoveryPolicy>::value2name(cov_recovery) ,  /* text width */ 30, 30, "Covariance recovery policy");
}


/** Computes stats on the degree (# of adjacent nodes) of all the nodes in the graph. Runs in O(N) with N=# of keyframes */
template <class RBA_SETTINGS_T>
void TRBA_Problem_state<RBA_SETTINGS_T>::compute_all_node_degrees(
	double &out_mean_degree,
	double &out_std_degree,
	double &out_max_degree) const
{
	out_mean_degree = 0;
	out_std_degree  = 0;
	out_max_degree  = 0;

	const size_t nKFs = keyframes.size();

	std::vector<size_t> degs;
	degs.reserve(nKFs);

	for(size_t i=0;i<nKFs;i++)
		degs.push_back( keyframes[i].adjacent_k2k_edges.size() );

	mrpt::math::meanAndStd(degs,out_mean_degree,out_std_degree);
	out_max_degree = mrpt::math::maximum(degs);

}

/** Returns true if the pair of KFs are connected thru a kf2kf edge, no matter the direction of the edge.
  * Runs in worst-case O(D) with D the degree of the KF graph (that is, the maximum number of edges adjacent to one KF) */
template <class RBA_SETTINGS_T>
bool TRBA_Problem_state<RBA_SETTINGS_T>::are_keyframes_connected(const TKeyFrameID id1, const TKeyFrameID id2) const
{
	ASSERT_BELOW_(id1, keyframes.size())
	ASSERT_BELOW_(id2, keyframes.size())

	const std::deque<k2k_edge_t*> & id1_adj = keyframes[id1].adjacent_k2k_edges;

	for (size_t i=0;i<id1_adj.size();i++)
		if ( id2== getTheOtherFromPair2(id1, *id1_adj[i]) )
			return true;

	return false;
}

} // end NS
