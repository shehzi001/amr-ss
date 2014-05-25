#include <vector>

#include <ros/console.h>

#include "sonar_map.h"

SonarMap::SonarMap(double resolution, double m_size_x, double m_size_y)
: resolution_(resolution)
, c_size_x_(lround(m_size_x / resolution) + 2)
, c_size_y_(lround(m_size_y / resolution) + 2)
, m_size_x_(resolution_ * c_size_x_)
, m_size_y_(resolution_ * c_size_y_)
, m_min_x_(-m_size_x_ / 2.0)
, m_min_y_(-m_size_y_ / 2.0)
, map_(c_size_x_, c_size_y_)
, map_free_(c_size_x_, c_size_y_)
, map_occupied_(c_size_x_, c_size_y_)
, map_tmp_occupied_(c_size_x_, c_size_y_)
{
}

void SonarMap::addScan(double sonar_x, double sonar_y, double sonar_theta, double fov, double max_range, double distance, double uncertainty)
{
  // Variables initialization
  int cell_x=0, cell_y=0;
  double map_x=0, map_y=0;
  double distance_to_cell = 0;
  double theta_to_cell = 0;
  double erfree = 0, erocc = 0, ea = 0;
  double empty_old = 0, empty_new = 0;
  double occupied_old= 0, occupied_new = 0, occupied_sum = 0;

  // Calculates possibilities, the occupied sum and updates empty map
  mapstore::MapStoreCone cone = mapstore::MapStoreCone(sonar_x / resolution_, sonar_y / resolution_, sonar_theta, fov, max_range / resolution_);
  while (cone.nextCell(cell_x, cell_y))
  {
    if (convertToMap(cell_x, cell_y, map_x, map_y))
    {
      // Pre calculation for the possibilities
      distance_to_cell = computeEuclideanDistance(map_x, map_y, sonar_x, sonar_y);
      theta_to_cell = computeAngularDistance(atan2(map_y - sonar_y, map_x - sonar_x), sonar_theta);

      // Calculates the possibility per cell
      erfree = ErFree(distance, distance_to_cell, uncertainty);
      erocc = ErOcc(distance, distance_to_cell, uncertainty);
      ea = Ea(fov, theta_to_cell);

      // If distance equals max_range, the sensor is not detecting an obstacle and the occupied possibility should be 0
      if (distance >= max_range) { erocc = 0.0; }

      // Updates the empty map
      empty_new = erfree * ea;
      clamp(empty_new, 0.0, 1.0);
      empty_old = map_free_.get(cell_x, cell_y);
      empty_new = empty_old + empty_new - (empty_old * empty_new);
      map_free_.set(cell_x, cell_y, empty_new);

      // Creates occupied error sum
      occupied_new = erocc * ea;
      clamp(occupied_new, 0.0, 1.0);
      occupied_new = occupied_new * (1.0 - empty_new);
      occupied_sum = occupied_sum + occupied_new;

      // Stores occ_new in temporary occupied map
      map_tmp_occupied_.set(cell_x, cell_y, occupied_new);
    }
  }

  // Normalizes occupied possibilities and updates occupied and combined map
  mapstore::MapStoreCone cone2 = mapstore::MapStoreCone(sonar_x / resolution_, sonar_y / resolution_, sonar_theta, fov, max_range / resolution_);
  while (cone2.nextCell(cell_x, cell_y))
  {
    if (convertToMap(cell_x, cell_y, map_x, map_y))
    {
      occupied_new = map_tmp_occupied_.get(cell_x,cell_y);
      occupied_old = map_occupied_.get(cell_x,cell_y);

      // Checks if there was an obstacle in the measurement and if yes, the occupied map gets updated
      if (occupied_sum > 0)
      {
        // Normalization for occupied map
        occupied_new = occupied_new / occupied_sum;

        // Updates the occupied map
        occupied_new = occupied_old + occupied_new - (occupied_old * occupied_new);
        clamp(occupied_new, 0.0, 1.0);
        map_occupied_.set(cell_x, cell_y, occupied_new);
      }
      else { occupied_new = occupied_old; }

      // Updates the combined map
      empty_new = map_free_.get(cell_x, cell_y);
      if (occupied_new >= empty_new) { map_.set(cell_x, cell_y, occupied_new); }
      else { map_.set(cell_x, cell_y, -empty_new); }
    }
  }
}

double SonarMap::ErFree(double sensed_distance, double delta, double uncertainty) const
{
  if (delta >= 0.0 && delta <= sensed_distance - uncertainty)
  {
    return 1 - pow((delta / (sensed_distance - uncertainty)), 2);
  }
  return 0.0;
}

double SonarMap::ErOcc(double sensed_distance, double delta, double uncertainty) const
{
  if (delta >= sensed_distance - uncertainty && delta <= sensed_distance + uncertainty)
  {
    return 1 - pow(((delta - sensed_distance) / uncertainty), 2);
  }
  return 0.0;
}

double SonarMap::Ea(double sonar_fov, double theta) const
{
  return 1 - pow(((2 * theta) / sonar_fov), 2);
}

bool SonarMap::convertToCell(const double m_x, const double m_y, int &c_x, int &c_y) const
{
  c_x = lround(m_x / resolution_);
  c_y = lround(m_y / resolution_);
  return (map_.isInX(c_x) && map_.isInY(c_y));
}

bool SonarMap::convertToMap(const int c_x, const int c_y, double &m_x, double &m_y) const
{
  m_x = c_x * resolution_;
  m_y = c_y * resolution_;
  return (map_.isInX(c_x) && map_.isInY(c_y));
}

