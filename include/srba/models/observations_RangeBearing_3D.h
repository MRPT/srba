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

	/** Observation = Range+Bearing (yaw & pitch) of landmarks relative to the sensor */
	struct RangeBearing_3D
	{
		static const size_t  OBS_DIMS = 3; //!< Each observation is a triplet of coordinates (range,yaw,pitch) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double range; //!< Distance (in meters)
			double yaw;   //!< Angle around +Z (in radians)
			double pitch; //!< Angle around +Y (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0] = range; obs[1] = yaw; obs[2] = pitch; 
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
	template <> struct landmark_matcher<RangeBearing_3D>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_containers<RangeBearing_3D::obs_data_t>::vector_t & new_kf_obs,
			const mrpt::aligned_containers<RangeBearing_3D::obs_data_t>::vector_t & old_kf_obs,
			const RangeBearing_3D::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf)
		{
			ASSERT_(new_kf_obs.size()==old_kf_obs.size())
			if (POSE::rotation_dimensions==3 && new_kf_obs.size()<3) return false; // Minimum number of points for SE(3): 3
			if (POSE::rotation_dimensions==2 && new_kf_obs.size()<2) return false; // Minimum number of points for SE(2): 2
			MRPT_TODO("Implement!")
			return true;
		}
	};
	/** @} */
}
} // end NS
