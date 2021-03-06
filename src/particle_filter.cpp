/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles = 100;

  // Create normal distributions around initial location estimate
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  std::random_device rd;
  unsigned int seed = rd();
  std::default_random_engine gen(seed);

  // Initialize particles
  for (int i = 0; i < num_particles; i++){
    Particle new_particle;
    new_particle.x = dist_x(gen);
    new_particle.y = dist_y(gen);
    new_particle.theta = dist_theta(gen);
    new_particle.weight = 1.0;
    particles.push_back(new_particle);
  }

  // Initialize vector of weights
  vector<double> init_weights(num_particles, 1.0);
  weights = init_weights;
  
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  for (int i = 0; i < num_particles; i++){

    double x_pred = 0.0;
    double y_pred = 0.0;
    double theta_pred = particles[i].theta + yaw_rate * delta_t;

    // Calculate new position, using linear model if yaw rate is zero
    if (yaw_rate == 0){
      x_pred = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      y_pred = particles[i].y + velocity * delta_t * sin(particles[i].theta);
    }
    else{
      x_pred = particles[i].x + (velocity / yaw_rate) * (sin(theta_pred) - sin(particles[i].theta));
      y_pred = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(theta_pred));
    }
    
    // Take into account the random gaussian noise of the control data
    std::normal_distribution<double> dist_x(x_pred, std_pos[0]);
    std::normal_distribution<double> dist_y(y_pred, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

    std::random_device rd;
    unsigned int seed = rd();
    std::default_random_engine gen(seed);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /*
   * Finds the predicted measurement that is closest to each observed measurement and
   * assigns the observed measurement to that particular landmark.
   */

  for (int i = 0; i < observations.size(); i++){   // for each observation
  
    double min_dist = __DBL_MAX__;

    for (auto landmark_pred : predicted){          // check all nearby landmarks, find the closest one
      double curr_dist = dist(observations[i].x, observations[i].y, landmark_pred.x, landmark_pred.y);

      if (curr_dist < min_dist){
        min_dist = curr_dist;
        observations[i].id = landmark_pred.id;     // assign a landmark ID to each measurement
      }
      
    }
    
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /*
   * Transforms measurements to map coordinates
   * Creates measurement - landmark associations using nearest neighbor method
   * Updates the weights of each particle using a mult-variate Gaussian distribution. 
   */

  for (int i = 0; i < num_particles; i++){  // for each particle, collect landmarks in range

    vector<LandmarkObs> landmarks_in_range;

    for (auto landmark : map_landmarks.landmark_list){

      if (dist(landmark.x_f, landmark.y_f, particles[i].x, particles[i].y) < sensor_range){

        LandmarkObs new_landmark;
        new_landmark.x = landmark.x_f;
        new_landmark.y = landmark.y_f;
        new_landmark.id = landmark.id_i;
        landmarks_in_range.push_back(new_landmark);
      }
    }

    particles[i].sense_x.clear();
    particles[i].sense_y.clear();

    vector<LandmarkObs> measurements;

    for (int j = 0; j < observations.size(); j++){      // transform observations to map coordinates
      LandmarkObs transformed_obs;
      transformed_obs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
      transformed_obs.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
      
      // data association between transformed obs and map landmarks
      particles[i].sense_x.push_back(transformed_obs.x);
      particles[i].sense_y.push_back(transformed_obs.y);
      measurements.push_back(transformed_obs);
    }
    
    // data association between transformed obs and map landmarks
    dataAssociation(landmarks_in_range, measurements);
    
    vector<int> associations;
    for (int j = 0; j < measurements.size(); j++){
      associations.push_back(measurements[j].id);
    }
    particles[i].associations = associations;

    // calculate weights
    double new_weight = 1.0;

    for (auto measurement : measurements){

      double denom = (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
      double x = measurement.x;
      double y = measurement.y;
      double mu_x = map_landmarks.landmark_list[measurement.id - 1].x_f;
      double mu_y = map_landmarks.landmark_list[measurement.id - 1].y_f;
      double stddev_x = pow(std_landmark[0], 2);
      double stddev_y = pow(std_landmark[1], 2);
      double exp_part = exp(-( pow((x-mu_x), 2)/(2*stddev_x) + pow((y-mu_y), 2)/(2*stddev_y)));
      double prob = exp_part / denom;
      new_weight *= prob;
    }

    weights[i] = new_weight;
    particles[i].weight = new_weight;
  }

}

void ParticleFilter::resample() {
  /*
   * Resamples particles with replacement with probability proportional to their weight. 
   */

  std::random_device rd;
  unsigned int seed = rd();
  std::default_random_engine gen(seed);
  std::discrete_distribution<> d(weights.begin(), weights.end());

  vector<Particle> resampled_particles;

  for (int i = 0; i < num_particles; i++){
    resampled_particles.push_back(particles[d(gen)]);
  }

  particles = resampled_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}