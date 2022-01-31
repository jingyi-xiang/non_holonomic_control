#include "vex.h"
#include "vector"
#include "iostream"
#include "path_planning.h"

double right_offset = 60;
double grids = map_height;
double pixel_per_grid = 230.0 / grids;

void draw_obstacles (void)
{
  for (int i = 0; i < map_height; i++)
  {
    for (int j = 0; j < map_width; j++)
    {
      if (obstacle_map[i][j])
      {
        double origin_x = 240 - 115 + right_offset;
        double origin_y = 230;
        double print_x_1 = origin_x + j*ft_per_grid/6.0*230.0;
        double print_y_2 = origin_y - (i+1)*ft_per_grid/6.0*230.0;

        Brain.Screen.drawRectangle (print_x_1, print_y_2, ft_per_grid/6.0*230.0, ft_per_grid/6.0*230.0, red);
      }
    }
  }
}

void draw_background (void)
{
  // graph the boarder
  Brain.Screen.drawLine(240.0 - 230.0/3.0*1.5 + right_offset, 0, 240.0 - 230.0/3.0*1.5 + right_offset, 230);
  Brain.Screen.drawLine(240.0 + 230.0/3.0*1.5 + right_offset, 0, 240.0 + 230.0/3.0*1.5 + right_offset, 230);
  Brain.Screen.drawLine(240.0 - 230.0/3.0*1.5 + right_offset, 0, 240.0 + 230.0/3.0*1.5 + right_offset, 0);
  Brain.Screen.drawLine(240.0 - 230.0/3.0*1.5 + right_offset, 230, 240.0 + 230.0/3.0*1.5 + right_offset, 230);

  // graph the grid lines
  // for (int i = 0; i < grids; i ++)
  // {
  //   // graph vertical
  //   Brain.Screen.drawLine(240.0 - 230.0/3.0*1.5 + right_offset + i*230.0/grids, 0, 240.0 - 230.0/3.0*1.5 + right_offset + i*230.0/grids, 230);

  //   // graph horizontal
  //   Brain.Screen.drawLine(240.0 - 230.0/3.0*1.5 + right_offset, i*230.0/grids, 240.0 + 230.0/3.0*1.5 + right_offset, i*230.0/grids);
  // }
}

void draw_point (double x, double y, double r, color pcolor)
{
  double origin_x = 240 - 115 + right_offset;
  double origin_y = 230;
  double print_x = x/6*230 + origin_x;
  double print_y = origin_y - y/6*230;

  Brain.Screen.drawCircle(print_x, print_y, 3, pcolor);
}

void draw_pixel (double x, double y)
{
  double origin_x = 240 - 115 + right_offset;
  double origin_y = 230;
  double print_x = x/6*230 + origin_x;
  double print_y = origin_y - y/6*230;
  
  Brain.Screen.drawPixel(print_x, print_y);
}

void draw_path (std::vector<std::vector<double>> path)
{
  int start_idx_i = path[0][1] / ft_per_grid;
  int start_idx_j = path[0][0] / ft_per_grid;
  int end_idx_i = path[path.size()-1][1] / ft_per_grid;
  int end_idx_j =  path[path.size()-1][0] / ft_per_grid;

  // start and end point
  double origin_x = 240 - 115 + right_offset;
  double origin_y = 230;
  double print_x = origin_x + start_idx_j*ft_per_grid/6.0*230.0;
  double print_y = origin_y - (start_idx_i + 1)*ft_per_grid/6.0*230.0;
  Brain.Screen.drawRectangle (print_x, print_y, ft_per_grid/6.0*230.0, ft_per_grid/6.0*230.0, blue);
  // end point
  print_x = origin_x + (end_idx_j)*ft_per_grid/6.0*230.0;
  print_y = origin_y - (end_idx_i + 1)*ft_per_grid/6.0*230.0;
  Brain.Screen.drawRectangle (print_x, print_y, ft_per_grid/6.0*230.0, ft_per_grid/6.0*230.0, green);

  for (int i = 1; i < path.size()-1; i ++)
  {
    draw_point (path[i][0], path[i][1], 3, yellow);
  }
}

bool updated = false;

void release (void)
{
  updated = false;
}

void obstacle_update (void)
{
  if (Brain.Screen.pressing())
  {
    if (Brain.Screen.xPosition() < (355+right_offset) && Brain.Screen.xPosition() > (125+right_offset) 
        && Brain.Screen.yPosition() > 0 && Brain.Screen.yPosition() < 230 && !updated)
    {
      updated = true;
      int i = (230 - Brain.Screen.yPosition()) / pixel_per_grid;
      int j = (Brain.Screen.xPosition() - (240 - 115 + right_offset))/ pixel_per_grid;

      std::cout << i << " " << j << std::endl;

      if (obstacle_map[i][j])
      {
        obstacle_map[i][j] = 0;
      }
      else
      {
        obstacle_map[i][j] = 1;
      }
    }
  }

  // use a boolean to prevent multiple trigger
  Brain.Screen.released(release);
}