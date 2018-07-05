#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_

#include <vector>

using namespace std;

void cluster(vector<vector<double> > data, double bandwidth, vector<vector<double> >& clusters, vector<int>& group);

#endif // _MEANSHIFT_H_
