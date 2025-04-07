#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <Eigen/Eigen>
#include "grid_path_searcher/backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

enum HeuType {Manhattan, Euclidean, Diagonal, None};

class AstarPathFinder
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		GridNodePtr terminatePtr;
		GridNodePtr startPtr;
		GridNodePtr endPtr;
		std::multimap<double, GridNodePtr> openSet;

		HeuType heu_type_{HeuType::Euclidean};
		bool use_tie_breaker_{false};

		double getHeu(GridNodePtr node1, GridNodePtr node2);
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);

		void setHeuType(HeuType type) { heu_type_ = type; };
		void setTieBreaker(bool enable) { use_tie_breaker_ = enable; };
		
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
		double getCost();
};

#endif