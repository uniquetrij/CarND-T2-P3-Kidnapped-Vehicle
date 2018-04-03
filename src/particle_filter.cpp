/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    initParticles(particles, x, y, theta, std[0], std[1], std[2], num_particles);
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    predictLocation(particles, delta_t, std_pos[0], std_pos[1], std_pos[2], velocity, yaw_rate);

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


    transformCoordinate(particles, observations);
    associate(particles, map_landmarks);
    weigh(particles, std_landmark[0], std_landmark[1]);
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    roulette(particles);
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
        const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

void initParticles(vector<Particle>& particles, double gps_x, double gps_y, double gps_theta, double sd_x, double sd_y, double sd_theta, int count) {

    normal_distribution<double> dist_x(gps_x, sd_x);
    normal_distribution<double> dist_y(gps_y, sd_y);
    normal_distribution<double> dist_theta(gps_theta, sd_theta);

    particles.clear();
    for (int i = 0; i < count; ++i) {
        Particle p;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;
        particles.push_back(p);
    }
}

void predictLocation(std::vector<Particle>& particles, double delta_t, double sd_x, double sd_y, double sd_theta, double velocity, double yaw_rate) {

    for (int i = 0; i < particles.size(); ++i) {
        Particle& p = particles.at(i);

        if (fabs(yaw_rate) < 0.0001) { 
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        } else {
            p.x = p.x + velocity / yaw_rate * (sin(p.theta + delta_t * yaw_rate) - sin(p.theta));
            p.y = p.y + velocity / yaw_rate * (cos(p.theta) - cos(p.theta + delta_t * yaw_rate));
            p.theta = p.theta + delta_t * yaw_rate;
        }

        normal_distribution<double> dist_x(p.x, sd_x);
        normal_distribution<double> dist_y(p.y, sd_y);
        normal_distribution<double> dist_theta(p.theta, sd_theta);

        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
    }

}

void transformCoordinate(std::vector<Particle>& particles, const std::vector<LandmarkObs>& observations) {
    for (Particle& p : particles) {
        p.sense_x.clear();
        p.sense_y.clear();
        p.associations.clear();
        p.mu_x.clear();
        p.mu_y.clear();
        for (LandmarkObs obs : observations) {
            LandmarkObs observation;
            p.sense_x.push_back(p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y));
            p.sense_y.push_back(p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y));
            p.associations.push_back(-1);
            p.mu_x.push_back(0);
            p.mu_y.push_back(0);
        }
    }
}

void associate(std::vector<Particle>& particles, const Map& map_landmarks) {
    for (Particle& p : particles) {
        for (int i = 0; i < p.associations.size(); i++) {
            double min = std::numeric_limits<double>::infinity();
            for (Map::single_landmark_s landmark : map_landmarks.landmark_list) {
                double distance = dist(p.sense_x.at(i), p.sense_y.at(i), landmark.x_f, landmark.y_f);
                if (distance < min) {
                    min = distance;
                    p.associations.at(i) = landmark.id_i;
                    p.mu_x.at(i) = landmark.x_f;
                    p.mu_y.at(i) = landmark.y_f;
                }
            }
        }
    }
}

void weigh(std::vector<Particle>& particles, double sd_x, double sd_y) {
    for (Particle& p : particles) {
        p.weight = 1;
        for (int i = 0; i < p.associations.size(); i++) {
            double x_obs = p.sense_x.at(i);
            double y_obs = p.sense_y.at(i);
            double mu_x = p.mu_x.at(i);
            double mu_y = p.mu_y.at(i);

            double gauss_norm = (1 / (2 * M_PI * sd_x * sd_y));
            double exponent = pow((x_obs - mu_x), 2) / (2 * pow(sd_x, 2))
                    + pow((y_obs - mu_y), 2) / (2 * pow(sd_y, 2));
            p.weight *= gauss_norm * exp(-exponent);
        }
    }
}

void roulette(std::vector<Particle>& particles) {
    vector<double> weights;
    double max = 0;
    for (int i = 0; i < particles.size(); i++) {
        weights.push_back(particles.at(i).weight);
        if (particles.at(i).weight > max) {
            max = particles.at(i).weight;
        }
    }

    uniform_int_distribution<int> dist_i(0, particles.size() - 1);
    int index = dist_i(gen);

    uniform_real_distribution<double> dist_d(0, max);
    double beta = 0;

    vector<Particle> resampled;
    for (int i = 0; i < particles.size(); i++) {
        beta += dist_d(gen) * 2.0;
        while (beta > weights.at(index)) {
            beta -= weights.at(index);
            index = (index + 1) % particles.size();
        }
        resampled.push_back(particles.at(index));
    }

    particles.clear();
    particles.assign(resampled.begin(), resampled.end());
}
