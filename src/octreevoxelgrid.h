#pragma once

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

template<class PointT>
class OctreeCentroidContainer : public pcl::octree::OctreeContainerBase<int>
{
public:
	/** \brief Class initialization. */
	OctreeCentroidContainer() : point_counter_(0)
	{
		this->reset();
	}

	/** \brief Empty class deconstructor. */
	virtual ~OctreeCentroidContainer()
	{
	}

	/** \brief deep copy function */
	virtual OctreeCentroidContainer *
	deepCopy() const
	{
		return (new OctreeCentroidContainer(*this));
	}

	/** \brief Add new point to voxel.
	 * \param[in] new_point the new point to add
	 */
	void addPoint(const PointT& new_point)
	{
		++point_counter_;

		pt_sum_.x += new_point.x;
		pt_sum_.y += new_point.y;
		pt_sum_.z += new_point.z;
	}

	/** \brief Calculate centroid of voxel.
	 * \param[out] centroid_arg the resultant centroid of the voxel
	 */
	void getCentroid(PointT& centroid_arg) const
	{
		if(point_counter_) {
			float fc = static_cast<float>(point_counter_);
			centroid_arg.x = pt_sum_.x / fc;
			centroid_arg.y = pt_sum_.y / fc;
			centroid_arg.z = pt_sum_.z / fc;
		}
	}

	/** \brief Reset leaf container. */
	void reset()
	{
		point_counter_ = 0;
		pt_sum_.x = pt_sum_.y = pt_sum_.z = 0;
	}

private:
	unsigned int point_counter_;
	PointT pt_sum_;
};

// Specializations for pcl::PointNormal where the normal is averaged, too
template<>
void OctreeCentroidContainer<pcl::PointNormal>::addPoint(const pcl::PointNormal& new_point)
{
	++point_counter_;

	pt_sum_.x += new_point.x;
	pt_sum_.y += new_point.y;
	pt_sum_.z += new_point.z;

	pt_sum_.normal_x += new_point.normal_x;
	pt_sum_.normal_y += new_point.normal_y;
	pt_sum_.normal_z += new_point.normal_z;
}

template<>
void OctreeCentroidContainer<pcl::PointNormal>::getCentroid(pcl::PointNormal& centroid_arg) const
{
	if(point_counter_) {
		float fc = static_cast<float>(point_counter_);
		centroid_arg.x = pt_sum_.x / fc;
		centroid_arg.y = pt_sum_.y / fc;
		centroid_arg.z = pt_sum_.z / fc;

		centroid_arg.normal_x = pt_sum_.normal_x / fc;
		centroid_arg.normal_y = pt_sum_.normal_y / fc;
		centroid_arg.normal_z = pt_sum_.normal_z / fc;
	}
}

template<>
void OctreeCentroidContainer<pcl::PointNormal>::reset()
{
	point_counter_ = 0;
	pt_sum_.x = pt_sum_.y = pt_sum_.z = 0;
	pt_sum_.normal_x = pt_sum_.normal_y = pt_sum_.normal_z = 0;
}


template<class PointT>
class OctreeVoxelGrid : public pcl::octree::OctreePointCloudVoxelCentroid< PointT, OctreeCentroidContainer<PointT> >
{
public:
	OctreeVoxelGrid(double resolution_arg)
		: pcl::octree::OctreePointCloudVoxelCentroid< PointT, OctreeCentroidContainer<PointT> >(resolution_arg)
	{

	}

	void filter(pcl::PointCloud<PointT>& cloud)
	{
		this->defineBoundingBox();
		this->addPointsFromInputCloud();

		size_t count = getVoxelCentroids(cloud.points);
		cloud.width = count;
		cloud.height = 1;
		cloud.is_dense = false;
	}
};
