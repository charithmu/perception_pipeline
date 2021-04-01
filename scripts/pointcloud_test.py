def create_random_pointlcloud():
    source = [0, 0, 0]
    deviationFromPoint = 10
    num_points = 100

    points_array = []
    labels = np.random.randint(1, 5, num_points)

    for _ in range(num_points):
        newPoint = [source[i] + random.random() * deviationFromPoint for i in range(3)]
        points_array.append(newPoint)

    points = np.array(points_array)

    print(points.shape)
    print(labels.shape)
    return points, labels


# def ros2open3d(ros_cloud):
#     field_names = [field.name for field in ros_cloud.fields]
#     cloud_data = list(point_cloud2.read_points(ros_cloud, skip_nans=False, field_names=field_names))

#     open3d_cloud = open3d.geometry.PointCloud()

#     xyz = [(x, y, z) for x, y, z, i in cloud_data]

#     open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
#     open3d.visualization.draw_geometries([open3d_cloud])

#     return open3d_cloud
