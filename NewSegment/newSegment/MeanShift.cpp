#include "MeanShift.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

using namespace std;

double kernel(double dist, double bandwidth) {
	if(dist > bandwidth) return 0;
	return 1;
}

double distance(vector<double> a, vector<double> b) {
	assert(a.size()==b.size());

    double dist = 0;
	for(int i=0; i<a.size(); ++i)
		dist += (a[i]-b[i])*(a[i]-b[i]);
	return sqrt(dist);
}

void shift(vector<double> oldMean, vector<vector<double> > data, double bandwidth, vector<double>& newMean) {
	int dim = oldMean.size();
    double scale = 0;
    for(vector<vector<double> >::iterator it=data.begin(); it!=data.end(); ++it) {
        vector<double> pt = *it;
        double weight = kernel(distance(oldMean, pt), bandwidth);
		for(int i=0; i<dim; ++i)
			newMean[i] += pt[i]*weight;
		scale += weight;
	}
	for(int i=0; i<dim; ++i) {
		if(scale != 0)
			newMean[i] /= scale;
		else
			newMean[i] = oldMean[i];
	}
}

void cluster(vector<vector<double> > data, double bandwidth, vector<vector<double> >& clusters, vector<int>& group) {
	for(int i=0; i<data.size(); ++i) {
        vector<double> oldMean = data[i];
        vector<double> newMean(oldMean.size(), 0);
		while(1) {
			newMean.assign(oldMean.size(), 0);
			shift(oldMean, data, bandwidth, newMean);
			if(distance(oldMean, newMean) < 1e-3*bandwidth)
				break;
			oldMean = newMean;
		}

		int j;
		for(j=0; j<clusters.size(); ++j)
			if(distance(newMean, clusters[j]) < bandwidth/2)
				break;

		if(j==clusters.size())
			clusters.push_back(newMean);
		
		group[i] = j;
	}
}
