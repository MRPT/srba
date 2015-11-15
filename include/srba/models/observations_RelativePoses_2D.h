/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/tfest.h>  // for use in landmark_matcher<>

namespace srba {
namespace observations {
	/** \addtogroup mrpt_srba_observations
		* @{ */

	/** Observation = Relative SE(2) poses (x,y,yaw) */
	struct RelativePoses_2D
	{
		static const size_t  OBS_DIMS = 3; //!< Each observation is a triplet (x,y,yaw) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double x,y;   //!< Displacement (in meters)
			double yaw;   //!< Angle around +Z (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0]=x; obs[1]=y; obs[2]=yaw;
			}
		};

		/** The type "TObservationParams" must be declared in each "observations::TYPE" to 
		  *  hold sensor-specific parameters, etc. needed in the sensor model. */
		struct TObservationParams
		{
			// This type of observations has no further parameters.
		};
	};

	/** Landmark matcher overloaded function. Used to provide a first initial guess for the relative pose in loop closures. \tparam POSE can be mrpt::poses::CPose2D or mrpt::poses::CPose3D */
	template <> struct landmark_matcher<RelativePoses_2D>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_containers<RelativePoses_2D::obs_data_t>::vector_t & new_kf_obs,
			const mrpt::aligned_containers<RelativePoses_2D::obs_data_t>::vector_t & old_kf_obs,
			const RelativePoses_2D::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf)
		{
			ASSERT_(new_kf_obs.size()==old_kf_obs.size())
			if (new_kf_obs.size()<2) return false;
			MRPT_TODO("Implement!")
			return true;
		}
	};
	/** @} */
}
} // end NS
