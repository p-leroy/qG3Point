// Example of a plugin action

#include "ccPointCloud.h"

#pragma once

class ccMainAppInterface;

namespace G3Point
{
int getBestOctreeLevel();
void get_neighbors(unsigned index);
bool query_neighbors(ccPointCloud* cloud, ccMainAppInterface* appInterface, bool useParallelStrategy=true);
void performActionA( ccMainAppInterface *appInterface );
}
