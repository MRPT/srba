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

	/** Observation = Relative SE(3) poses (x,y,z,yaw,pitch,roll) */
	struct RelativePoses_3D
	{
		static const size_t  OBS_DIMS = 6; //!< Each observation is a vector (x,y,z,yaw,pitch,roll) 
		
		/** The observation-specific data structure */
		struct obs_data_t
		{
			double x,y,z;   //!< Displacement (in meters)
			double yaw,pitch,roll;   //!< Angles (in radians)

			/** Converts this observation into a plain array of its parameters */
			template <class ARRAY>
			inline void getAsArray(ARRAY &obs) const {
				obs[0]=x; obs[1]=y; obs[2]=z; 
				obs[3]=yaw; obs[4]=pitch; obs[5]=roll;
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
	template <> struct landmark_matcher<RelativePoses_3D>
	{
		template <class POSE>
		static bool find_relative_pose(
			const mrpt::aligned_containers<RelativePoses_3D::obs_data_t>::vector_t & new_kf_obs,
			const mrpt::aligned_containers<RelativePoses_3D::obs_data_t>::vector_t & old_kf_obs,
			const RelativePoses_3D::TObservationParams &params,
			POSE &pose_new_kf_wrt_old_kf)
		{
			ASSERT_(new_kf_obs.size()==old_kf_obs.size())
			// Find the observation related to one of the two KFs connected by this new edge: it should have an exact (0,...,0) in its relative pose:
			for (size_t i=0;i<new_kf_obs.size();i++)
			{
				const RelativePoses_3D::obs_data_t & kf0 = new_kf_obs[i], &kf1 = old_kf_obs[i];
				if ( (kf0.x!=0 || kf0.y!=0 || kf0.z!=0 || kf0.yaw!=0 || kf0.pitch!=0 || kf0.roll!=0) && 
				     (kf1.x!=0 || kf1.y!=0 || kf1.z!=0 || kf1.yaw!=0 || kf1.pitch!=0 || kf1.roll!=0) )
					 continue; // skip.
				const mrpt::poses::CPose3D new_obs(kf0.x,kf0.y,kf0.z,kf0.yaw,kf0.pitch,kf0.roll);
				const mrpt::poses::CPose3D old_obs(kf1.x,kf1.y,kf1.z,kf1.yaw,kf1.pitch,kf1.roll);
				pose_new_kf_wrt_old_kf = POSE(old_obs-new_obs);
				return true;
			}
			return false; // None found (should not happen?)
		}
	};

	/** @} */
}
} // end NS
