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
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  std::cout << "Initializing particle filter..." << std::endl;

  num_particles = 20;  // TODO: Set the number of particles

  // Create normal distributions from initial location estimate
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  std::default_random_engine gen;

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

  std::cout << "Particle filter initialized!" << std::endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::cout << "Starting prediction step..." << std::endl;

  for (int i = 0; i < num_particles; i++){

    double theta_pred = particles[i].theta + yaw_rate * delta_t;
    double x_pred = particles[i].x + (velocity / yaw_rate) * (sin(theta_pred) - sin(particles[i].theta));
    double y_pred = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta - cos(theta_pred)));

    std::normal_distribution<double> dist_x(x_pred, std_pos[0]);
    std::normal_distribution<double> dist_y(y_pred, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

    std::default_random_engine gen;

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }

  std::cout << "Prediction step done!" << std::endl;

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  std::cout << "Starting data association..." << std::endl;
  std::cout << "Predicted measurements: " << predicted.size() << std::endl;
  std::cout << "Observations: " << observations.size() << std::endl;

  for (int i = 0; i < observations.size(); i++){   // for each observation
    double min_dist = __DBL_MAX__;
    std::cout << "OBS" << i << ": " << observations[i].x << "," << observations[i].y << std::endl;
    for (auto landmark_pred : predicted){   // check all nearby landmarks, find the closest one
      double curr_dist = dist(observations[i].x, observations[i].y, landmark_pred.x, landmark_pred.y);
      std::cout << "Current distance: " << curr_dist << std::endl;
      if (curr_dist < min_dist){
        min_dist = curr_dist;
        observations[i].id = landmark_pred.id; // assign a landmark ID to each measurement
      }
      std::cout << "Minimum distance: " << min_dist << std::endl;
      std::cout << "Nearest neighbor for OBS " << i << ": " << observations[i].id << std::endl;

    }
    
  }

  std::cout << "Data association finished!" << std::endl;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  std::cout << "Updating weights..." << std::endl;

  for (int i = 0; i < num_particles; i++){  // for each particle
    std::cout << "PARTICLE #" << i << " at " << particles[i].x << ", " << particles[i].y << std::endl;
    // collect landmarks in range
    vector<LandmarkObs> landmarks_in_range;
    for (auto landmark : map_landmarks.landmark_list){
      if (dist(landmark.x_f, landmark.y_f, particles[i].x, particles[i].y) < sensor_range){
        LandmarkObs new_landmark;
        new_landmark.x = landmark.x_f;
        new_landmark.y = landmark.y_f;
        new_landmark.id = landmark.id_i;
        landmarks_in_range.push_back(new_landmark);
        std::cout << "Found nearby landmark: " << new_landmark.id << std::endl;
      }
    }
    // transform observations to map coordinates
    vector<LandmarkObs> measurements;
    for (int j = 0; j < observations.size(); j++){
      LandmarkObs transformed_obs;
      transformed_obs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
      transformed_obs.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
      // data association between transformed obs and map landmarks (?)
      measurements.push_back(transformed_obs);
    }
    for (int i = 0; i < measurements.size(); i++){
      std::cout << "Measurement ID: " << measurements[i].id << std::endl;
    }

    dataAssociation(landmarks_in_range, measurements); // measurements will now be assigned to landmarks
    
    for (int i = 0; i < measurements.size(); i++){
      std::cout << "Measurement ID: " << measurements[i].id << std::endl;
    }

    // calculate weights
    double new_weight = 1.0;

    for (auto measurement : measurements){

      std::cout << "Calculating prob for " << measurement.id << std::endl;

      double denom = (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
      double x = measurement.x;
      double y = measurement.y;
      double mu_x = map_landmarks.landmark_list[measurement.id - 1].x_f;  // cheating a bit because landmarks are in a row
      double mu_y = map_landmarks.landmark_list[measurement.id - 1].y_f;  // this is messed up
      double stddev_x = pow(std_landmark[0], 2);
      double stddev_y = pow(std_landmark[1], 2);
      long double exp_part = exp(-( pow((x-mu_x), 2)/(2*stddev_x) + pow((y-mu_y), 2)/(2*stddev_y)));
      double prob = exp_part / denom;
      new_weight *= prob;

      std::cout << "Prob = " << prob << std::endl;
    }

    std::cout << "Updating weights: " << i << " of " << weights.size() << std::endl;
    weights[i] = new_weight;
  }
  
  std::cout << "Weights updated!" << std::endl;

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::cout << "Resampling!" << std::endl;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());

  vector<Particle> resampled_particles;

  for (int i = 0; i < num_particles; i++){
    resampled_particles.push_back(particles[d(gen)]);
  }

  particles = resampled_particles;

  std::cout << "Resampling finished!" << std::endl;

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
  particle.associations= associations;
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