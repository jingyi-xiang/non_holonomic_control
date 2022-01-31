#include "vex.h"
#include "iostream"
#include "vector"
#include "list"
#include "math.h"
#include "path_planning.h"
#include "screen_print.h"

const double ft_per_grid = 2.0 / grid_per_unit;

// helper functions
double pt2ptDis (node* pt1, node* pt2)
{
  return sqrt (pow(pt1->y - pt2->y, 2) + pow(pt1->x - pt2->x, 2));
}

// obstacle matrix initialization
// [0] is t/f, [1] is detection count
// not using the third dimension in this portion of the project
int obstacle_map [map_height][map_width];
bool no_solution = false;

std::vector<std::vector<double>> find_path_A_star (const int start_idx, const int end_idx)
{
  // allocate memory space for map
  node* map = new node [map_height * map_width];

  // initialize all nodes in map
  for (int i = 0; i < map_height; i ++)
  {
    for (int j = 0; j < map_width; j ++)
    {
      map [i*map_width + j].x = 2.0/grid_per_unit*0.5 + j*2.0/grid_per_unit;
      map [i*map_width + j].y = 2.0/grid_per_unit*0.5 + i*2.0/grid_per_unit;
      map [i*map_width + j].is_obstacle = false;
      map [i*map_width + j].visited = false;
      map [i*map_width + j].parent = nullptr;
      map [i*map_width + j].local_dis = INFINITY;
      map [i*map_width + j].global_dis = INFINITY;
    }
  }

  // update map against obstacle map
  for (int i = 0; i < map_height; i ++)
  {
    for (int j = 0; j < map_width; j ++)
    {
      if (obstacle_map[i][j])
      {
        map [i*map_width + j].is_obstacle = true;
      }
    }
  }

  for (int i = 0; i < map_height; i ++)
  {
    for (int j = 0; j < map_width; j ++)
    {
      // initialize all neighbors
      if (i > 0)
      {
        // underneath
        map [i*map_width + j].neighbors.push_back(&map [(i-1)*map_width + j]);
      }
      if (j > 0)
      {
        // left
        map [i*map_width + j].neighbors.push_back(&map [i*map_width + j-1]);
      }
      if (i < map_height-1)
      {
        // above
        map [i*map_width + j].neighbors.push_back(&map [(i+1)*map_width + j]);
      }
      if (j < map_width-1)
      {
        // right
        map [i*map_width + j].neighbors.push_back(&map [i*map_width + j+1]);
      }
      
      // diagonal elements
      if (i > 0 && j > 0)
      {
        // lower left
        map [i*map_width + j].neighbors.push_back(&map [(i-1)*map_width + j-1]);
      }
      if (i > 0 && j < map_width-1)
      {
        // lower right
        map [i*map_width + j].neighbors.push_back(&map [(i+1)*map_width + j+1]);
      }
      if (i < map_height-1 && j > 0)
      {
        // upper left
        map [i*map_width + j].neighbors.push_back(&map [(i+1)*map_width + j-1]);
      }
      if (i < map_height-1 && j < map_width-1)
      {
        // upper right
        map [i*map_width + j].neighbors.push_back(&map [(i+1)*map_width + j+1]);
      }
    }
  }

  node* start = &map[start_idx];
  node* end = &map[end_idx];
  
  // create the path to be returned
  std::vector<std::vector<double>> path;

  // initialize conditions
  node* current_node = start;
  // the global dis is essentially a heuristic that equals to 
  // current local dis + euclidean dis between current node and end node
  start->global_dis = pt2ptDis(start, end);
  start->local_dis = 0.0;

  // create an array of unvisited nodes
  std::list<node*> not_checked;
  // push back the first node
  not_checked.push_back (start);

  // run the algorithm
  while ((!not_checked.empty()) && (current_node != end))
  {
    // first sort the array of unvisited nodes
    not_checked.sort();

    while (!not_checked.empty() && (not_checked.front()->visited))
    {
      not_checked.pop_front();
    }
    // in case the poped object is the only object in list
    // which probably means no solution
    if (not_checked.empty())
    {
      no_solution = true;
      Brain.Screen.printAt (10, 20, "no solution");
      return path;
    }
    
    // since the not_checked list is sorted, check the first one
    current_node = not_checked.front();
    current_node->visited = true;

    // now visit its neighbors
    for (node* neighbor_node : current_node->neighbors)
    {
      // add this neighbor to the not checked list iff not obstacle and not visited
      if (!(neighbor_node->visited) && !(neighbor_node->is_obstacle))
      {
        not_checked.push_back (neighbor_node);
      }
      
      // now check if this neighbor can produce a shorter path
      double new_local_dis = current_node->local_dis + pt2ptDis(current_node, neighbor_node);
      // update path if this neighbor produce a shorter path
      if (new_local_dis < neighbor_node->local_dis)
      {
        // create links
        neighbor_node->parent = current_node;
        // update neighbor node's info
        neighbor_node->local_dis = new_local_dis;
        // update heuristic value
        neighbor_node->global_dis = new_local_dis + pt2ptDis(neighbor_node, end);
      }
    }
  }

  if (!no_solution)
  {
    no_solution = false;

    // prepare the path output
    std::vector<double> temp = {end->x, end->y};
    path.push_back (temp);

    // set cursor to traverse the computed path
    node* ntemp = end->parent;
    
    while (ntemp != nullptr)
    {
      // weird c++ stuff
      std::vector<std::vector<double>>::iterator it;
      it = path.begin();

      temp = {ntemp->x, ntemp->y};
      path.insert(it, temp);
      
      // update cursor
      ntemp = ntemp->parent;
    }
  }

  // to prevent memory leak
  delete []map;

  return path;
}