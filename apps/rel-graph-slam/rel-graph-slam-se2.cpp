/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <srba.h>
#include <mrpt/graphs.h>
#include <mrpt/random.h>
#include <mrpt/gui.h>  // For rendering results as a 3D scene
#include <mrpt/graphs/TNodeID.h>

#include <set>

using namespace srba;
using namespace mrpt::poses;
using namespace std;

// --------------------------------------------------------------------------------
// Declare a typedef "my_srba_t" for easily referring to my RBA problem type:
// --------------------------------------------------------------------------------
struct RBA_OPTIONS : public RBA_OPTIONS_DEFAULT
{
	typedef ecps::local_areas_fixed_size            edge_creation_policy_t;  //!< One of the most important choices: how to construct the relative coordinates graph problem
	typedef options::sensor_pose_on_robot_none           sensor_pose_on_robot_t;  // sensor pose == robot pose
	typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D>   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to some given matrix
	typedef options::solver_LM_no_schur_sparse_cholesky  solver_t;
};

typedef RbaEngine<
	kf2kf_poses::SE2,                // Parameterization  KF-to-KF poses
	landmarks::RelativePoses2D,      // Parameterization of landmark positions
	observations::RelativePoses_2D,  // Type of observations
	RBA_OPTIONS                  // Other parameters
	>
	my_srba_t;

const bool SRBA_SHOW_GLOBAL_MAP = getenv("SRBA_SHOW_GLOBAL_MAP")!=NULL;

const double STD_NOISE_XY = 0.001;
const double STD_NOISE_YAW = mrpt::DEG2RAD(0.05);

int main(int argc, char**argv)
{
	if (argc!=2)
	{
		cerr << "Usage: " << argv[0] << " <INPUT_DATASET.txt>\n";
		return 1;
	}
	const string sFileDataset = string(argv[1]);

	// Parse input dataset file:
	// --------------------------------
	mrpt::graphs::CNetworkOfPoses2D graph_dataset;
	cout << "Loading "<< sFileDataset << " ...\n";
	graph_dataset.loadFromTextFile(sFileDataset);

	cout << "Done. "<< graph_dataset.countDifferentNodesInEdges() << " nodes, " << graph_dataset.edgeCount() << " edges.\n";
	
	cout << "Collapsing...";
	graph_dataset.collapseDuplicatedEdges();
	cout << "Remanining edges: " << graph_dataset.edgeCount() << endl;

	// Get neighbors so we can easily iterate over nodes:
	mrpt::containers::map_as_vector<mrpt::graphs::TNodeID,std::set<mrpt::graphs::TNodeID> > nodeNeighbors;
	graph_dataset.getAdjacencyMatrix(nodeNeighbors);


	// SRBA:
	// -----------------------------------------------------
	my_srba_t rba;     //  Create an empty RBA problem

	// --------------------------------------------------------------------------------
	// Set parameters
	// --------------------------------------------------------------------------------
	rba.setVerbosityLevel( 1 );   // 0: None; 1:Important only; 2:Verbose

	rba.parameters.srba.use_robust_kernel = false;
	//rba.parameters.srba.optimize_new_edges_alone  = false;  // skip optimizing new edges one by one? Relative graph-slam without landmarks should be robust enough, but just to make sure we can leave this to "true" (default)

	// Information matrix for relative pose observations:
	{
		Eigen::Matrix3d ObsL;
		ObsL.setZero();
		ObsL(0,0) = 1/mrpt::square(STD_NOISE_XY); // x
		ObsL(1,1) = 1/mrpt::square(STD_NOISE_XY); // y
		ObsL(2,2) = 1/mrpt::square(STD_NOISE_YAW); // phi

		// Set:
		rba.parameters.obs_noise.lambda = ObsL;
	}

	// =========== Topology parameters ===========
	rba.parameters.srba.max_tree_depth       =
	rba.parameters.srba.max_optimize_depth   = 3;
	rba.parameters.ecp.submap_size           = 40;
	rba.parameters.ecp.min_obs_to_loop_closure = 1;
	// ===========================================

	// --------------------------------------------------------------------------------
	// Dump parameters to console (for checking/debugging only)
	// --------------------------------------------------------------------------------
	cout << "RBA parameters:\n-----------------\n";
	rba.parameters.srba.dumpToConsole();

#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D win("RBA results",640,480);
#endif

	// --------------------------------------------------------------------------------
	// Process the dataset:
	// --------------------------------------------------------------------------------
	const size_t nKFs = nodeNeighbors.size();

	for (size_t cur_kf = 0; cur_kf<nKFs; ++cur_kf)
	{
		cout << " ============ DATASET TIMESTEP: " << cur_kf << " / " << nKFs-1 << " ==========\n";
		if (mrpt::system::os::kbhit())
		{
			char ch = mrpt::system::os::getch();
			if (ch==27) 
				break;
		}

		// Create list of observations for keyframe: "cur_kf"
		my_srba_t::new_kf_observations_t  list_obs;

		// To emulate graph-SLAM, each keyframe MUST have exactly ONE fixed "fake landmark", representing its pose:
		// --------------------------------------------------------------------------------------------------------
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
		const std::set<mrpt::graphs::TNodeID> &nn = nodeNeighbors[cur_kf];

		for (std::set<mrpt::graphs::TNodeID>::const_iterator it_nn = nn.begin();it_nn!=nn.end();++it_nn)
		{
			const mrpt::graphs::TNodeID other_id = *it_nn;

			// Online SLAM: we cannot add an edge to a FUTURE node: 
			if (other_id>cur_kf) 
				continue;

			// Get the observation (and invert it if the edge was otherway around):
			CPose2D observed_pose;
			if (graph_dataset.edgeExists(cur_kf,other_id))
			     observed_pose =  graph_dataset.getEdge(cur_kf,other_id);
			else observed_pose = -graph_dataset.getEdge(other_id,cur_kf);
			
			
			my_srba_t::new_kf_observation_t obs_field;
			obs_field.is_fixed = false;   // "Landmarks" (relative poses) have unknown relative positions (i.e. treat them as unknowns to be estimated)
			obs_field.is_unknown_with_init_val = false; // Ignored, since all observed "fake landmarks" already have an initialized value.

			obs_field.obs.feat_id      = other_id;  // The observed KF ID
			obs_field.obs.obs_data.x   = observed_pose.x() ; // mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.y   = observed_pose.y() ; // mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_XY);
			obs_field.obs.obs_data.yaw = observed_pose.phi() ; // mrpt::random::getRandomGenerator().drawGaussian1D(0,STD_NOISE_YAW);

			list_obs.push_back( obs_field );
		}

		ASSERT_(cur_kf==0 || list_obs.size()>1);

		//  Here happens the main stuff: create Key-frames, build structures, run optimization, etc.
		//  ============================================================================================
		my_srba_t::TNewKeyFrameInfo new_kf_info;
		rba.define_new_keyframe(
			list_obs,      // Input observations for the new KF
			new_kf_info,   // Output info
			true           // Also run local optimization?
			);

		cout << "Created KF #" << new_kf_info.kf_id
			<< " | # kf-to-kf edges created:" <<  new_kf_info.created_edge_ids.size()  << endl
			<< "Optimization error: " << new_kf_info.optimize_results.total_sqr_error_init << " -> " << new_kf_info.optimize_results.total_sqr_error_final << endl;

	// Display:
#if MRPT_HAS_WXWIDGETS
		if (win.isOpen())
		{
			// --------------------------------------------------------------------------------
			// Show 3D view of the resulting map:
			// --------------------------------------------------------------------------------
			my_srba_t::TOpenGLRepresentationOptions  opengl_options;
			opengl_options.draw_kf_hierarchical = true;
			if (!SRBA_SHOW_GLOBAL_MAP) 
				opengl_options.span_tree_max_depth = rba.parameters.srba.max_tree_depth;

			mrpt::opengl::CSetOfObjects::Ptr rba_3d = mrpt::opengl::CSetOfObjects::Create();

			rba.build_opengl_representation(
				new_kf_info.kf_id ,  // Root KF: the current (latest) KF
				opengl_options, // Rendering options
				rba_3d  // Output scene
				);

			{
				mrpt::opengl::COpenGLScene::Ptr &scene = win.get3DSceneAndLock();
				scene->clear();
				scene->insert(rba_3d);
				win.unlockAccess3DScene();
			}
			win.repaint();

			//cout << "Press any key to continue.\n";
			//win.waitForKey();
		}
#endif

	} // end-for each dataset entry

	// --------------------------------------------------------------------------------
	// Saving RBA graph as a DOT file:
	// --------------------------------------------------------------------------------
	const string sFil = "graph.dot";
	cout << "Saving final graph of KFs and LMs to: " << sFil << endl;
	rba.save_graph_as_dot(sFil, true /* LMs=save */);
	cout << "Done.\n";

	// Show final "global" map (spanning tree).
	{
		my_srba_t::TOpenGLRepresentationOptions  opengl_options;
		opengl_options.draw_kf_hierarchical = true;
		mrpt::opengl::CSetOfObjects::Ptr rba_3d = mrpt::opengl::CSetOfObjects::Create();

		rba.build_opengl_representation(
			0,  // Root KF
			opengl_options, // Rendering options
			rba_3d  // Output scene
			);

		{
			mrpt::opengl::COpenGLScene scene;
			scene.insert(rba_3d);
			scene.saveToFile("final_global_map.3Dscene");
		}

	#if MRPT_HAS_WXWIDGETS
		mrpt::gui::CDisplayWindow3D win2("RBA final map",640,480);
		{
			mrpt::opengl::COpenGLScene::Ptr &scene = win2.get3DSceneAndLock();
			scene->clear();
			scene->insert(rba_3d);
			win2.unlockAccess3DScene();
		}
		win2.repaint();
		win2.waitForKey();
	#endif
	}


	return 0; // All ok
}
