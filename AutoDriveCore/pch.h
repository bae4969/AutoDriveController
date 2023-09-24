#ifndef PCH_H
#define PCH_H

#include <format>
#include <future>
#include <thread>

#include <Windows.h>
#include <debugapi.h>

#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <zmq_utils.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>

#include <vtkRenderWindow.h>

#endif
