/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle {
    int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    std::vector<double> mu_x;
    std::vector<double> mu_y;
};

class ParticleFilter {
    // Number of particles to draw
    int num_particles;




    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    //    std::vector<double> weights;

public:

    // Set of current particles
    std::vector<Particle> particles;

    // Constructor
    // @param num_particles Number of particles

    ParticleFilter() : num_particles(99), is_initialized(false) {
    }

    // Destructor

    ~ParticleFilter() {
    }

    /**
     * init Initializes particle filter by initializing particles to Gaussian
     *   distribution around first position and all the weights to 1.
     * @param x Initial x position [m] (simulated estimate from GPS)
     * @param y Initial y position [m]
     * @param theta Initial orientation [rad]
     * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
     *   standard deviation of yaw [rad]]
     */
    void init(double x, double y, double theta, double std[]);

    /**
     * prediction Predicts the state for the next time step
     *   using the process model.
     * @param delta_t Time between time step t and t+1 in measurements [s]
     * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
     *   standard deviation of yaw [rad]]
     * @param velocity Velocity of car from t to t+1 [m/s]
     * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
     */
    void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);

    /**
     * updateWeights Updates the weights for each particle based on the likelihood of the 
     *   observed measurements. 
     * @param sensor_range Range [m] of sensor
     * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
     * @param observations Vector of landmark observations
     * @param map Map class containing map landmarks
     */
    void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs> &observations,
            const Map &map_landmarks);

    /**
     * resample Resamples from the updated set of particles to form
     *   the new set of particles.
     */
    void resample();

    /*
     * Set a particles list of associations, along with the associations calculated world x,y coordinates
     * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
     */
    Particle SetAssociations(Particle& particle, const std::vector<int>& associations,
            const std::vector<double>& sense_x, const std::vector<double>& sense_y);


    std::string getAssociations(Particle best);
    std::string getSenseX(Particle best);
    std::string getSenseY(Particle best);

    /**
     * initialized Returns whether particle filter is initialized yet or not.
     */
    const bool initialized() const {
        return is_initialized;
    }
};
/**
 * initializes the particles set.
 * @param particles Vector of particles in the filter
 * @param gps_x Initial x position [m] (simulated estimate from GPS)
 * @param gps_y Initial y position [m]
 * @param gps_theta Initial orientation [rad]
 * @param sd_x standard deviation in x
 * @param sd_y standard deviation in y
 * @param sd_theta standard deviation in orientation
 * @param count number of particles to be initialized
 */
void initParticles(std::vector<Particle>& particles, double gps_x, double gps_y, double gps_theta, double sd_x, double sd_y, double sd_theta, int count);

/**
 * predicts the location at the next time step.
 * @param particles Vector of particles in the filter
 * @param delta_t Time between time step t and t+1 in measurements [s]
 * @param sd_x standard deviation in x
 * @param sd_y standard deviation in y
 * @param sd_theta standard deviation in orientation
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 */
void predictLocation(std::vector<Particle>& particles, double delta_t, double sd_x, double sd_y, double sd_theta, double velocity, double yaw_rate);

/**
 * transforms car coordinates of observations to world coordinates for each of the particles.
 * @param particles Vector of particles in the filter
 * @param observations Vector containing landmark observations
 */
void transformCoordinate(std::vector<Particle>& particles, const std::vector<LandmarkObs>& observations);

/**
 * finds which observations correspond to which landmarks for each of the particles.
 * @param particles Vector of particles in the filter
 * @param map_landmarks Map class containing map landmarks
 */
void associate(std::vector<Particle>& particles, const Map& map_landmarks);

/**
 * assigns weights to each of the particles.
 * @param particles Vector of particles in the filter
 * @param sd_x standard deviation in x
 * @param sd_y standard deviation in y
 */
void weigh(std::vector<Particle>& particles, double sd_x, double sd_y);

/**
 * resamples the particles based on their weights.
 * @param particles Vector of particles in the filter
 */
void roulette(std::vector<Particle>& particles);
#endif /* PARTICLE_FILTER_H_ */
