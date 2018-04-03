/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: uniquetrij
 *
 * Created on 2 April, 2018, 11:01 PM
 */

#include <cstdlib>
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

void testInitParticle() {
    vector<Particle> particles;
    initParticles(particles, 4983, 5029, 1.201, 2, 2, 0.05, 3);
    cout << particles.at(0).x << " " << particles.at(0).y << " " << particles.at(0).theta;

}

void testEstimateLocation() {
    Particle particle;
    particle.x = 102;
    particle.y = 65;
    particle.theta = 5 * M_PI / 8;
    vector<Particle> particles;
    particles.push_back(particle);
    predictLocation(particles, 0.1, 0, 0, 0, 110, M_PI / 8);
    cout << particles.at(0).x << " " << particles.at(0).y << " " << particles.at(0).theta;
}

void testTrandformCoordinates() {
    Particle particle;
    particle.x = 4;
    particle.y = 5;
    particle.theta = -M_PI / 2;
    vector<Particle> particles;
    particles.push_back(particle);

    vector<LandmarkObs> obss;
    LandmarkObs obs;
    obs.x = 2;
    obs.y = 2;
    obss.push_back(obs);
    obs.x = 3;
    obs.y = -2;
    obss.push_back(obs);
    obs.x = 0;
    obs.y = -4;
    obss.push_back(obs);
    transformCoordinate(particles, obss);
    cout << particles.at(0).sense_x.at(0) << " " << particles.at(0).sense_y.at(0) << " " << particles.at(0).theta << endl;
    cout << particles.at(0).sense_x.at(1) << " " << particles.at(0).sense_y.at(1) << " " << particles.at(0).theta << endl;
    cout << particles.at(0).sense_x.at(2) << " " << particles.at(0).sense_y.at(2) << " " << particles.at(0).theta;


}

void testAssociate() {
    Particle particle;
    particle.x = 4;
    particle.y = 5;
    particle.theta = -M_PI / 2;
    vector<Particle> particles;
    particles.push_back(particle);

    vector<LandmarkObs> obss;
    LandmarkObs obs;
    obs.x = 2;
    obs.y = 2;
    obss.push_back(obs);
    obs.x = 3;
    obs.y = -2;
    obss.push_back(obs);
    obs.x = 0;
    obs.y = -4;
    obss.push_back(obs);
    transformCoordinate(particles, obss);

    Map map;
    Map::single_landmark_s lm;
    lm.id_i = 1;
    lm.x_f = 5;
    lm.y_f = 3;
    map.landmark_list.push_back(lm);

    lm.id_i = 2;
    lm.x_f = 2;
    lm.y_f = 1;
    map.landmark_list.push_back(lm);

    lm.id_i = 3;
    lm.x_f = 6;
    lm.y_f = 1;
    map.landmark_list.push_back(lm);

    lm.id_i = 4;
    lm.x_f = 7;
    lm.y_f = 4;
    map.landmark_list.push_back(lm);

    lm.id_i = 5;
    lm.x_f = 4;
    lm.y_f = 7;
    map.landmark_list.push_back(lm);

    associate(particles, map);

    cout << particles.at(0).sense_x.at(0) << " " << particles.at(0).sense_y.at(0) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(0) << " " << particles.at(0).mu_x.at(0) << " " << particles.at(0).mu_y.at(0) << endl;
    cout << particles.at(0).sense_x.at(1) << " " << particles.at(0).sense_y.at(1) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(1) << " " << particles.at(0).mu_x.at(1) << " " << particles.at(0).mu_y.at(1) << endl;
    cout << particles.at(0).sense_x.at(2) << " " << particles.at(0).sense_y.at(2) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(2) << " " << particles.at(0).mu_x.at(2) << " " << particles.at(0).mu_y.at(2) << endl;



}

void testWeigh() {
    Particle particle;
    particle.x = 4;
    particle.y = 5;
    particle.theta = -M_PI / 2;
    vector<Particle> particles;
    particles.push_back(particle);

    vector<LandmarkObs> obss;
    LandmarkObs obs;
    obs.x = 2;
    obs.y = 2;
    obss.push_back(obs);
    obs.x = 3;
    obs.y = -2;
    obss.push_back(obs);
    obs.x = 0;
    obs.y = -4;
    obss.push_back(obs);
    transformCoordinate(particles, obss);

    Map map;
    Map::single_landmark_s lm;
    lm.id_i = 1;
    lm.x_f = 5;
    lm.y_f = 3;
    map.landmark_list.push_back(lm);

    lm.id_i = 2;
    lm.x_f = 2;
    lm.y_f = 1;
    map.landmark_list.push_back(lm);

    lm.id_i = 3;
    lm.x_f = 6;
    lm.y_f = 1;
    map.landmark_list.push_back(lm);

    lm.id_i = 4;
    lm.x_f = 7;
    lm.y_f = 4;
    map.landmark_list.push_back(lm);

    lm.id_i = 5;
    lm.x_f = 4;
    lm.y_f = 7;
    map.landmark_list.push_back(lm);

    associate(particles, map);

    weigh(particles, 0.3, 0.3);

    cout << particles.at(0).sense_x.at(0) << " " << particles.at(0).sense_y.at(0) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(0) << " " << particles.at(0).mu_x.at(0) << " " << particles.at(0).mu_y.at(0) << endl;
    cout << particles.at(0).sense_x.at(1) << " " << particles.at(0).sense_y.at(1) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(1) << " " << particles.at(0).mu_x.at(1) << " " << particles.at(0).mu_y.at(1) << endl;
    cout << particles.at(0).sense_x.at(2) << " " << particles.at(0).sense_y.at(2) << " " << particles.at(0).theta << " " << particles.at(0).associations.at(2) << " " << particles.at(0).mu_x.at(2) << " " << particles.at(0).mu_y.at(2) << endl;
    cout << particles.at(0).weight << endl;



}

/*
 * 
 */
int main(int argc, char** argv) {

    //    testInitParticle();
    //    testEstimateLocation();
    //    testTrandformCoordinates();
    //    testAssociate();
    testWeigh();


}



