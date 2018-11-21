/*
 * particle_filter.cpp
 *
 *  Created on: Nov 17, 2018
 *      Author: Sunsheng Gu
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// declare a random engine
// random_device rd;
// static default_random_engine gen(rd());
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // Set number of particles
  num_particles = 100;
    
  // Resize the particles vector
  // particles.resize(num_particles);
  
  // Resize the weights vector
  weights.resize(num_particles);
  
  normal_distribution<double> x_uncertainty(0, std[0]);
  normal_distribution<double> y_uncertainty(0, std[1]);
  normal_distribution<double> theta_uncertainty(0, std[2]);
  
  for (int i = 0; i < num_particles; i++){
    Particle p;
    p.id = i;
    p.x = x + x_uncertainty(gen);
    p.y = y + y_uncertainty(gen);
    p.theta = theta + theta_uncertainty(gen);
    p.weight = 1.0;
    weights[i] = p.weight;
    particles.push_back(p);
  }
  
  // Indicates that initializaiton is done
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  // Set noise for sensors
  normal_distribution<double> x_Normal(0, std_pos[0]);
  normal_distribution<double> y_Normal(0, std_pos[1]);
  normal_distribution<double> theta_Normal(0, std_pos[2]);
  
  for (int i = 0; i < num_particles; i++){
    // Update based on measurements
    // Use the constant yaw rate formula for small yaw rate to save time
	  
    // Source for the following if statement: https://github.com/jeremy-shannon/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp
    if (fabs(yaw_rate) < 0.0001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate* delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Add noise
    particles[i].x += x_Normal(gen);
    particles[i].y += y_Normal(gen);
    particles[i].theta += theta_Normal(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  // Loop through the predictions
  for (unsigned int i = 0; i < observations.size(); i++){
    
    // Get the current observation
    LandmarkObs obs = observations[i];

    // Initialize minimum distance to a very large number
    double min_dist = numeric_limits<double>::max();
    
    // Initialize the landmark id
    int lm_id = -1;
    
    for (unsigned int j = 0; j < predicted.size(); j++) {
      // Get current prediction
      LandmarkObs pred = predicted[j];
      
      // Calculate distance between observed and predicted landmarks
      double distance = dist(obs.x, obs.y, pred.x, pred.y);

      // Set the observation's id to the closest predicted landmark
      if (distance < min_dist) {
        min_dist = distance;
        lm_id = pred.id;
      }
    }
    observations[i].id = lm_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  // I studied code from the following source, but the updateWeights function itself is entirely my own work:
  // https://github.com/jeremy-shannon/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp
  // Loop through each particle
  for (unsigned int i = 0; i < num_particles; i++){
    // Get the x, y, and orientation for each particle
    double px, py, ptheta;
    px = particles[i].x;
    py = particles[i].y;
    ptheta = particles[i].theta;
    
    // Create a vector to hold valid map landmarks
    vector<LandmarkObs> valid_lm;
    
    // Loop through each landmark
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
      // Load landmark information
      int lm_id = map_landmarks.landmark_list[j].id_i;
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      // Find distance between landmark and particle
      double range = dist(lm_x, lm_y, px, py);
      
      // If within valid range, store that landmark
      if (range < sensor_range){
        valid_lm.push_back(LandmarkObs{lm_id, lm_x, lm_y});
      }
    }
    
    // Create a vector to store transformed observations
    vector<LandmarkObs> t_obs;
    
    // Transform each observation
    for (unsigned int k = 0; k < observations.size(); k++){
      double xc, yc, xt, yt;
      xc = observations[k].x;
      yc = observations[k].y;
      xt = xc*cos(ptheta) - yc*sin(ptheta) + px;
      yt = xc*sin(ptheta) + yc*cos(ptheta) + py;
      t_obs.push_back(LandmarkObs{observations[k].id, xt, yt});
    }
    
    // Perform data association
    dataAssociation(valid_lm, t_obs);
    
    // Initialize particle weight and the temporary weight variable
    particles[i].weight = 1.0;
    //double weight = 1.0;
    
    // Loop through each observation to update particle weight
    for (unsigned int m = 0; m < t_obs.size(); m++){
      double sig_x, sig_y, obs_x, obs_y, mu_x, mu_y, gauss_norm, exponent, weight;
      sig_x = std_landmark[0];
      sig_y = std_landmark[1];
      obs_x = t_obs[m].x;
      obs_y = t_obs[m].y;
      gauss_norm = (1.0/(2.0 * M_PI * sig_x * sig_y));
      for (unsigned int n = 0; n < valid_lm.size(); n++){
        if (t_obs[m].id == valid_lm[n].id){
          mu_x = valid_lm[n].x;
          mu_y = valid_lm[n].y;
        }
      }
      exponent = pow((obs_x - mu_x), 2.0)/(2.0 * pow(sig_x, 2.0)) + pow((obs_y - mu_y), 2.0)/(2.0 * pow(sig_y, 2.0));
      particles[i].weight *= gauss_norm * exp(-exponent);
      //particles[i].weight = weight;
    }
    weights[i] = particles[i].weight;
    //cout << weights[i] << endl;
  }
  cout << weights[2] << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
  // I used code from the following source to understand how to use a discrete distribution in C++, but I made some changes:
  // https://github.com/mvirgo/Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp
  // Initialize a new particle list
  vector<Particle> p2;
  
  // Define generator and distribution
  // random_device rd;
  default_random_engine generator;
  discrete_distribution<int> distribution(weights.begin(), weights.end());
  
  // Perform resampling
  for (unsigned int j = 0; j < num_particles; j++){
    p2.push_back(particles[distribution(generator)]);
  }
  
  // Update particle list
  particles = p2;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
