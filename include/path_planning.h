#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

// modify the below map configuration if needed
#define tiles_h 3
#define tiles_v 3
#define grid_per_unit 5
#define map_height tiles_v*grid_per_unit
#define map_width tiles_h*grid_per_unit

#include "vector"

struct node 
{
  bool is_obstacle = false;
  bool visited = false;
  double global_dis;
  double local_dis;
  double x;
  double y;
  std::vector<node*> neighbors;
  node * parent;

  // operator overload for sorting
  bool operator < (const node &nodeObj) const
  {
    return global_dis < nodeObj.global_dis;
  }
};

std::vector<std::vector<double>> find_path_A_star (const int start_idx, const int end_idx);
void destroy_map (node* map);
void temp_init(void);

const extern double ft_per_grid;

extern int obstacle_map [map_height][map_width];
extern bool no_solution;

#endif