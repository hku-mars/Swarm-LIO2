#pragma once
#pragma warning(disable:4996)
//#ifndef PCL_SEGEMENT_FEC_H
//#define PCL_SEGEMENT_FEC_H
//#endif

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <omp.h>


/**
* Store index and label information for each point
*/

namespace FEC{
    using namespace std;
    struct PointIndex_NumberTag
    {
        float nPointIndex;
        float nNumberTag;
    };

    static bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1)
    {
        return p0.nNumberTag < p1.nNumberTag;
    }

    static std::vector<pcl::PointIndices> FastEuclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolorance, int max_n) {

        unsigned long i, j;
        if (cloud->size() < min_component_size)
        {
            PCL_ERROR("Could not find any cluster");
        }

        pcl::KdTreeFLANN<pcl::PointXYZ>cloud_kdtreeflann;
        cloud_kdtreeflann.setInputCloud(cloud);

        int cloud_size = cloud->size();
        std::vector<int> marked_indices;
        marked_indices.resize(cloud_size);


        memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
        std::vector<int> pointIdx;
        std::vector<float> pointquaredDistance;

        int tag_num = 1, temp_tag_num = -1;

        for (i = 0; i < cloud_size; i++)
        {
            // Clustering process
            if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated
            {
                pointIdx.clear();
                pointquaredDistance.clear();
                cloud_kdtreeflann.radiusSearch(cloud->points[i], tolorance, pointIdx, pointquaredDistance, max_n);
                /**
                * All neighbors closest to a specified point with a query within a given radius
                * para.tolorance is the radius of the sphere that surrounds all neighbors
                * pointIdx is the resulting index of neighboring points
                * pointquaredDistance is the final square distance to adjacent points
                * pointIdx.size() is the maximum number of neighbors returned by limit
                */
                int min_tag_num = tag_num;
                for (j = 0; j < pointIdx.size(); j++)
                {
                    /**
                     * find the minimum label value contained in the field points, and tag it to this cluster label.
                     */
                    if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                    {
                        min_tag_num = marked_indices[pointIdx[j]];
                    }
                }
                for (j = 0; j < pointIdx.size(); j++)
                {
                    temp_tag_num = marked_indices[pointIdx[j]];

                    /*
                     * Each domain point, as well as all points in the same cluster, is uniformly assigned this label
                     */
                    if (temp_tag_num > min_tag_num)
                    {
                        for (int k = 0; k < cloud_size; k++)
                        {
                            if (marked_indices[k] == temp_tag_num)
                            {
                                marked_indices[k] = min_tag_num;
                            }
                        }
                    }
                    marked_indices[pointIdx[j]] = min_tag_num;
                }
                tag_num++;
            }
        }

        std::vector<PointIndex_NumberTag> indices_tags;
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        indices_tags.resize(cloud_size);

        PointIndex_NumberTag temp_index_tag;


        for (i = 0; i < cloud_size; i++)
        {
            /**
            * Put each point index and the corresponding tag value into the indices_tags
            */
            temp_index_tag.nPointIndex = i;
            temp_index_tag.nNumberTag = marked_indices[i];

            indices_tags[i] = temp_index_tag;
        }

        sort(indices_tags.begin(), indices_tags.end(), NumberTag);

        unsigned long begin_index = 0;
        for (i = 0; i < indices_tags.size(); i++)
        {
            // Relabel each cluster
            if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
            {
                if ((i - begin_index) >= min_component_size)
                {
                    unsigned long m = 0;
                    inliers->indices.resize(i - begin_index);
                    for (j = begin_index; j < i; j++)
                        inliers->indices[m++] = indices_tags[j].nPointIndex;
                    cluster_indices.push_back(*inliers);
                }
                begin_index = i;
            }
        }

        if ((i - begin_index) >= min_component_size)
        {
            for (j = begin_index; j < i; j++)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                {
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                }
                cluster_indices.push_back(*inliers);
            }
        }
        return cluster_indices;

    }
}
