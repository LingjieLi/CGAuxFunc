#pragma once
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/ArrayKernel.hh>

#include<Eigen/Eigen>
#include<Eigen/SVD>
#include<Eigen/Sparse>
#include<Eigen/SparseLU>
#include<Eigen/SparseQR>
#include<Eigen/SparseCholesky>

#include<pcl/io/ply_io.h>
#include<pcl/io/obj_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>


#include <glm/glm.hpp>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;//三角面片网格
typedef OpenMesh::PolyMesh_ArrayKernelT<> PolyMesh;//四边形网格

typedef Eigen::Triplet<double> T;
typedef Eigen::SparseMatrix<double> SpMat;

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointNormal> NormalCloud;

struct Vertex
{
    glm::vec3 Position=glm::vec3(0.0f);
    glm::vec3 Color=glm::vec3(0.0f);
    glm::vec3 Normal=glm::vec3(1.0f);
};

enum MeshElement
{
    VERTEX,
    FACE,
    EDGE
};

enum DrawMode
{
    POINTS,
    WIREFRAME,
    FLATLINES,
    FLAT
};
