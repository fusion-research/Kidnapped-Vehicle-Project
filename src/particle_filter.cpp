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
#include <functional>

#include "helper_functions.h"

using namespace std;
using std::string;
using std::vector;
using std::normal_distribution;
using std::max_element;

#define EPS 0.00001
#define INIT_WEIGHT 1.0

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    num_particles = 100;  // the number of particles
    // This line creates a normal (Gaussian) distribution for x
    normal_distribution<double> dist_x(x, std[0]);
    // Create normal distributions for y
    normal_distribution<double> dist_y(y, std[1]);
    // Create normal distributions for theta
    normal_distribution<double> dist_theta(theta, std[2]);
    
    cout << "Number of particles = " << num_particles << endl;

    /* ***  init particles  *** */
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(INIT_WEIGHT);

        // init particle
        Particle p = Particle();
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.weight = weights[i];
        p.theta = dist_theta(gen);
        p.id = i;
        particles.push_back(p);
    }

    // set that particles are initialized to <true>
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    // normal distributions 0-centered
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // New state calculation
    for (int i = 0; i < num_particles; ++i)
    {
        Particle *p = &(particles[i]);
        double theta = p->theta;

        if (fabs(yaw_rate) < EPS)
        {
            p->x += velocity * delta_t * cos(theta);
            p->y += velocity * delta_t * sin(theta);
            // yaw stays the same
        }
        else
        {
            double d_theta = yaw_rate * delta_t;
            double angle = theta + d_theta;
            p->x += velocity / yaw_rate * (sin(angle) - sin(theta));
            p->y += velocity / yaw_rate * (cos(theta) - cos(angle));
            p->theta += d_theta;
        }

        // Add noise
        p->x += dist_x(gen);
        p->y += dist_y(gen);
        p->theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
    for (unsigned int i = 0; i < observations.size(); i++) {
        // take current observation
        LandmarkObs o = observations[i];
        // init minimum distance to maximum possible
        double min_dist = numeric_limits<double>::max();
        // init id of landmark from map placeholder to be associated with the observation
        int map_id = -1;
        for (unsigned int j = 0; j < predicted.size(); j++) {
            // take current prediction
            LandmarkObs p = predicted[j];
            // get distance between current/predicted landmarks
            double cur_dist = dist(o.x, o.y, p.x, p.y);
            // find the predicted landmark nearest the current observed landmark
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                map_id = p.id;
            }
        }
        // set the observation's id to the nearest predicted landmark's id
        observations[i].id = map_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    // find landmarks in range
    // for each particle...
    for (int i = 0; i < num_particles; i++) {
        // get the particle x, y coordinates
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;

        // create a vector to hold the map landmark locations predicted to be within sensor range of the particle
        vector<LandmarkObs> predictions;

        // for each map landmark...
        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            // get id and x,y coordinates
            float landmark_x = map_landmarks.landmark_list[j].x_f;
            float landmark_y = map_landmarks.landmark_list[j].y_f;
            int landmark_id = map_landmarks.landmark_list[j].id_i;

            // only consider landmarks within sensor range of the particle
            if (fabs(landmark_x - p_x) <= sensor_range && fabs(landmark_y - p_y) <= sensor_range) {
                // add prediction to vector
                predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
            }
        }

        // create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates
        vector<LandmarkObs> transformed_os;
        for (unsigned int j = 0; j < observations.size(); j++) {
            double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
            double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
            transformed_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
        }

        // perform dataAssociation for the predictions and transformed observations on current particle
        dataAssociation(predictions, transformed_os);
        // reinit weight
        particles[i].weight = INIT_WEIGHT;

        for (unsigned int j = 0; j < transformed_os.size(); j++) {
            // placeholders for observation and associated prediction coordinates
            double o_x, o_y, pr_x, pr_y;
            o_x = transformed_os[j].x;
            o_y = transformed_os[j].y;

            int associated_prediction = transformed_os[j].id;

            // get the x,y coordinates of the prediction associated with the current observation
            for (unsigned int k = 0; k < predictions.size(); k++) {
                if (predictions[k].id == associated_prediction) {
                  pr_x = predictions[k].x;
                  pr_y = predictions[k].y;
                }
            }
            // calculate weight for this observation with multivariate Gaussian
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );
            // product of this obersvation weight with total observations weight
            particles[i].weight *= obs_w;
        }
    }
}

void ParticleFilter::resample() {
    vector<Particle> new_particles;
    // get all of the current weights
    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }
    // generate random starting index for resampling wheel
    uniform_int_distribution<int> uniintdist(0, num_particles-1);
    auto index = uniintdist(gen);
    // get max weight
    double max_weight = *max_element(weights.begin(), weights.end());
    // uniform random distribution [0.0, max_weight)
    uniform_real_distribution<double> unirealdist(0.0, max_weight);
    double beta = 0.0;
    // spin the resample wheel!
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}

double ParticleFilter::keepWeights() {
    // keep previous weights
    weights.clear();
    double max_weight = 0.0;
    for (int i = 0; i < num_particles; i++) {
        auto current_weight = particles.at(i).weight;
        if (current_weight > max_weight) {
            max_weight = current_weight;
        }
        weights.push_back(current_weight);
    }
    return max_weight;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {
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
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
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
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}