#include "Eigen/Dense"

#include <vector>

#pragma once

class ccMainAppInterface;
class ccPointCloud;

namespace G3Point
{
void add_to_stack(int index, const Eigen::ArrayXi& n_donors, const Eigen::ArrayXXi& donors, std::vector<int>& stack);
int segment_labels(bool useParallelStrategy=true);
int segment_labels_steepest_slope(bool useParallelStrategy=true);
void add_to_stack_braun_willett(int index, const Eigen::ArrayXi& delta, const Eigen::ArrayXi &Di, std::vector<int>& stack, int local_maximum);
int segment_labels_braun_willett(bool useParallelStrategy=true);
void get_neighbors_distances_slopes(unsigned index);
bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
void performActionA( ccMainAppInterface *appInterface );
}
