#define _USE_MATH_DEFINES
#include <cmath>
#include <random>

#include <ros/console.h>

#include "particle_filter.h"

ParticleFilter::ParticleFilter(double map_min_x, double map_max_x, double map_min_y, double map_max_y, ComputeParticleWeightCallback callback)
: callback_(callback)
, motion_model_(0.02, 0.01)
, random_particle_generator_(map_min_x, map_max_x, map_min_y, map_max_y)
, particle_set_size_(100)
, motion_guesses_(1)
, is_initialized(false)
{
}

void ParticleFilter::update(double x, double y, double yaw)// desired motion
{
  Particle particle;
  Pose best_pose;
  double best_weight = 0.0;
  double weight_sum = 0.0;
  ParticleVector particles_new;
  ParticleVector::iterator particles_new_iterator;
  particles_new_iterator = particles_new.begin();

  double resample_pos = 0.0;
  double resample_step = 0.0;
  double weight_pos = 0.0;
  int particle_index = 0;
  ParticleVector particles_resampled;
  particles_resampled.resize(particle_set_size_);

  // Create initial particle set
  if (!is_initialized)
  {
    is_initialized = true;
    particles_.resize(particle_set_size_);
    for (int i = 0; i < particle_set_size_; i++)
    {
      particles_.at(i) = random_particle_generator_.generateParticle();
    }
  }

  // Create new particles and calculates their weight
  for (int index_particle = 0; index_particle < particle_set_size_; index_particle++)
  {
    for (int index_motion = 0; index_motion < motion_guesses_; index_motion++)
    {
      // Create new particles based on motion and save them
      particle.pose = motion_model_.sample(particles_.at(index_particle).pose);
      particle.weight = callback_(particle);
      particles_new_iterator = particles_new.insert(particles_new_iterator, particle);

      // Save the weight sum for resampling
      weight_sum = weight_sum + particle.weight;

      // Set best pose based on the weight
      if (particle.weight > best_weight)
      {
        best_weight = particle.weight;
        best_pose = particle.pose;
      }
    }
  }

  // Resampling
  resample_step = weight_sum / particle_set_size_;
  for (int i = 0; i < particle_set_size_; i++)
  {
    while (resample_pos > (weight_pos + particles_new.at(particle_index).weight))
    {
      weight_pos = weight_pos + particles_new.at(particle_index).weight;
      particle_index++;
    }
    particles_resampled.at(i) = particles_new.at(particle_index);
    resample_pos = resample_pos + resample_step;
  }

  // Filtering bad samples out and create new ones
  for (int i = 0; i < particle_set_size_; i++)
  {
    if (particles_resampled.at(i).weight < 0.1)
    {
      particles_resampled.at(i) = random_particle_generator_.generateParticle();
    }
  }

  pose_estimate_ = best_pose;
  particles_ = particles_resampled;
}



