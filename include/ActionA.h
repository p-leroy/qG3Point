// Example of a plugin action

#include "ccPointCloud.h"

#include "Eigen/Dense"

#pragma once

class ccMainAppInterface;

namespace G3Point
{
void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
int segment_labels(bool useParallelStrategy=true);
void get_neighbors_distances_slopes(unsigned index);
bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
void performActionA( ccMainAppInterface *appInterface );
}
