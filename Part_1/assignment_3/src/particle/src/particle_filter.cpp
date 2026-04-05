#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle/particle_filter.h"
using namespace std;

static default_random_engine gen;

/*
* This function initialize randomly the particles
* Input:
*  std - noise that might be added to the position
*  nParticles - number of particles
*/
void ParticleFilter::init_random(double std[], int nParticles) {
    num_particles = nParticles;

    uniform_real_distribution<double> dist_x(map_x_boundaries.first, map_x_boundaries.second);
    uniform_real_distribution<double> dist_y(map_y_boundaries.first, map_y_boundaries.second);
    uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
    }
    is_initialized = true;
}

/*
* This function initialize the particles using an initial guess
* Input:
*  x,y,theta - position and orientation
*  std - noise that might be added to the position
*  nParticles - number of particles
*/ 
void ParticleFilter::init(double x, double y, double theta, double std[], int nParticles) {
    num_particles = nParticles;
    normal_distribution<double> dist_x(-std[0], std[0]);
    normal_distribution<double> dist_y(-std[1], std[1]);
    normal_distribution<double> dist_theta(-std[2], std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = x + dist_x(gen);
        p.y = y + dist_y(gen);
        p.theta = theta + dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
    }
    is_initialized = true;
}
/*
* The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep
* Input:
*  delta_t  - time elapsed beetween measurements
*  std_pos  - noise that might be added to the position
*  velocity - velocity of the vehicle
*  yaw_rate - current orientation
* Output:
*  Updated x,y,theta position
*/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    for (int i = 0; i < num_particles; i++) {
        double x     = particles[i].x;
        double y     = particles[i].y;
        double theta = particles[i].theta;

        if (fabs(yaw_rate) < 0.00001) {
            // Straight motion
            x += velocity * delta_t * cos(theta);
            y += velocity * delta_t * sin(theta);
        } else {
            // Curved motion
            x += (velocity / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
            y += (velocity / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
            theta += yaw_rate * delta_t;
        }

        particles[i].x = x + dist_x(gen);
        particles[i].y = y + dist_y(gen);
        particles[i].theta = theta + dist_theta(gen);
    }
}
/*
* This function associates the landmarks from the MAP to the landmarks from the OBSERVATIONS
* Input:
*  mapLandmark   - landmarks of the map
*  observations  - observations of the car
* Output:
*  Associated observations to mapLandmarks (perform the association using the ids)
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations) {
    for (int i = 0; i < observations.size(); i++) {
        double min_dist = numeric_limits<double>::max();
        for (int j = 0; j < mapLandmark.size(); j++) {
            double d = dist(observations[i].x, observations[i].y,
                            mapLandmark[j].x,  mapLandmark[j].y);
            if (d < min_dist) {
                min_dist = d;
                observations[i].id = mapLandmark[j].id;
            }
        }
    }
}

/*
* This function associates observations to landmarks using Mahalanobis distance
* with chi-squared gating (99% confidence, 2 DOF: threshold = 9.210).
* Observations for which no landmark falls within the gate are marked id = -1.
*/
void ParticleFilter::dataAssociationMahalanobis(std::vector<LandmarkObs> mapLandmark, std::vector<LandmarkObs>& observations, double sig_x, double sig_y) {

    // chi-squared 99% confidence threshold for 2 DOF
    const double chi2_gate = 9.210;

    for (int i = 0; i < (int)observations.size(); i++) {
        double min_d2 = numeric_limits<double>::max();
        observations[i].id = -1;  // default: no valid association

        for (int j = 0; j < (int)mapLandmark.size(); j++) {
            double dx = observations[i].x - mapLandmark[j].x;
            double dy = observations[i].y - mapLandmark[j].y;
            double d2 = (dx * dx) / (sig_x * sig_x) + (dy * dy) / (sig_y * sig_y);

            if (d2 < min_d2) {
                min_d2 = d2;
                if (d2 < chi2_gate)
                    observations[i].id = mapLandmark[j].id;
            }
        }
    }
}

/*
* This function transform a local (vehicle) observation into a global (map) coordinates
* Input:
*  observation   - A single landmark observation
*  p             - A single particle
* Output:
*  local         - transformation of the observation from local coordinates to global
*/
LandmarkObs transformation(LandmarkObs observation, Particle p) {
    LandmarkObs global;
    global.id = observation.id;
    global.x  = p.x + observation.x * cos(p.theta) - observation.y * sin(p.theta);
    global.y  = p.y + observation.x * sin(p.theta) + observation.y * cos(p.theta);
    return global;
}

/*
* This function updates the weights of each particle
* Input:
*  std_landmark   - Sensor noise
*  observations   - Sensor measurements
*  map_landmarks  - Map with the landmarks
* Output:
*  Updated particle's weight (particles[i].weight *= w)*/
void ParticleFilter::updateWeights(double std_landmark[],
        std::vector<LandmarkObs> observations, Map map_landmarks) {

    //Creates a vector that stores tha map (this part can be improved)
    std::vector<LandmarkObs> mapLandmark;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        mapLandmark.push_back(LandmarkObs{
            map_landmarks.landmark_list[j].id_i,
            map_landmarks.landmark_list[j].x_f,
            map_landmarks.landmark_list[j].y_f
        });
    }

    for (int i = 0; i < particles.size(); i++) {
        // Before applying the association we have to transform the observations in the global coordinates
        std::vector<LandmarkObs> transformed_observations;
        // for each observation transform it (transformation function)
        for (int j = 0; j < (int)observations.size(); j++)
            transformed_observations.push_back(transformation(observations[j], particles[i]));

        // Associate each transformed observation to a map landmark.
        // During warm-up always use plain NN regardless of use_mahalanobis.
        bool mahal_active = use_mahalanobis && (frames_processed >= mahalanobis_warmup);
        if (mahal_active)
            dataAssociationMahalanobis(mapLandmark, transformed_observations,
                                       std_landmark[0], std_landmark[1]);
        else
            dataAssociation(mapLandmark, transformed_observations);

        particles[i].weight = 1.0;

        // Minimum weight assigned to gated-out observations: gaussian evaluated at the gate boundary
        const double w_gate_boundary =
            exp(-9.210 / 2.0) / (2 * M_PI * std_landmark[0] * std_landmark[1]);

        // Compute the probability
        // The particle’s final weight is the product of each measurement’s Multivariate-Gaussian probability density.

        for (int k = 0; k < (int)transformed_observations.size(); k++) {
            // No landmark within the gate: assign boundary-level weight
            if (transformed_observations[k].id == -1) {
                particles[i].weight *= w_gate_boundary;
                continue;
            }

            double obs_x = transformed_observations[k].x;
            double obs_y = transformed_observations[k].y;
            double l_x = 0, l_y = 0;
            for (int p = 0; p < (int)mapLandmark.size(); p++) {
                if (transformed_observations[k].id == mapLandmark[p].id) {
                    l_x = mapLandmark[p].x;
                    l_y = mapLandmark[p].y;
                }
            }
            double w = exp(-(pow(l_x - obs_x, 2) / (2 * pow(std_landmark[0], 2))
                        + pow(l_y - obs_y, 2) / (2 * pow(std_landmark[1], 2))))
                        / (2 * M_PI * std_landmark[0] * std_landmark[1]);
            particles[i].weight *= w;
        }
    }
    frames_processed++;
}

/*
* This function resamples the set of particles by repopulating the particles using the weight as metric
*/
void ParticleFilter::resample() {
    uniform_int_distribution<int> dist_distribution(0, num_particles - 1);
    double beta = 0.0;
    vector<double> weights;
    int index = dist_distribution(gen);
    vector<Particle> new_particles;

    for (int i = 0; i < num_particles; i++)
        weights.push_back(particles[i].weight);

    double max_w = *max_element(weights.begin(), weights.end());

    // Range 2*max_w as per Thrun 2002 (same as in lab)

    uniform_real_distribution<double> uni_dist(0.0, 2.0 * max_w);

    // Write here the resampling technique

    for (int i = 0; i < num_particles; i++) {
        beta += uni_dist(gen);
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}