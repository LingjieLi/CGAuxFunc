#ifndef AUXILIARY_H
#define AUXILIARY_H

//opengl
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
//pcl
#include <pcl/io/ply_io.h>

//std
#include <cmath>

//local
#include "CGAuxFunc/globaldefinition.h"

class Auxiliary {
public:
    static Cloud ToCloud(TriMesh mesh)
    {
        Cloud cloud;
        for (TriMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++) {
            TriMesh::Point p = mesh.point(*v_it);
            cloud.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
        }
        return cloud;
    }

    static glm::quat quaternion(glm::vec3 rotateAxis, float rotateAngle)
    {
        glm::quat MyQuat;
        MyQuat.w = glm::cos(glm::degrees(rotateAngle) / 2.0);
        MyQuat.x = rotateAxis.x * glm::sin(glm::degrees(rotateAngle) / 2.0);
        MyQuat.y = rotateAxis.y * glm::sin(glm::degrees(rotateAngle) / 2.0);
        MyQuat.z = rotateAxis.z * glm::sin(glm::degrees(rotateAngle) / 2.0);

        return MyQuat;
    }
    static glm::quat rotQuat(glm::vec3 rotateAxis, float rotateAngle) //根据旋转轴和旋转角计算四元数
    {
        //std::cout<<"angle: "<<rotateAngle<<", degress angle: "<<glm::degrees(rotateAngle)<<", axis: "<<rotateAxis.x<<","<<rotateAxis.y<<","<<rotateAxis.z;
        rotateAxis = glm::normalize(rotateAxis);
        glm::quat r;
        r.w = glm::cos(glm::degrees(rotateAngle / 2.0));
        r.x = glm::sin(glm::degrees(rotateAngle / 2.0)) * rotateAxis.x;
        r.y = glm::sin(glm::degrees(rotateAngle / 2.0)) * rotateAxis.y;
        r.z = glm::sin(glm::degrees(rotateAngle / 2.0)) * rotateAxis.z;

        return r;
    }
    static glm::mat3 rotQuat(glm::quat quat) //计算四元数的同态变换
    {
        float w = quat.w;
        float x = quat.x;
        float y = quat.y;
        float z = quat.z;

        glm::mat3 Rq = glm::mat3(1.0);
        Rq[0][0] = 1 - 2 * y * y - 2 * z * z;
        Rq[0][1] = 2 * x * y - 2 * w * z;
        Rq[0][2] = 2 * x * z + 2 * w * y;

        Rq[1][0] = 2 * x * y + 2 * w * z;
        Rq[1][1] = 1 - 2 * x * x - 2 * z * z;
        Rq[1][2] = 2 * y * z - 2 * w * x;

        Rq[2][0] = 2 * x * z - 2 * w * y;
        Rq[2][1] = 2 * y * z + 2 * w * x;
        Rq[2][2] = 1 - 2 * x * x - 2 * y * y;

        return Rq;
    }

    static glm::mat3 rotAngleAndAxis(glm::vec3 axis, float angle)
    {
        float c = glm::cos(glm::degrees(angle));
        float s = glm::sin(glm::degrees(angle));
        float t = 1 - c;
        float x = axis.x;
        float y = axis.y;
        float z = axis.z;

        glm::mat3 Rm = glm::mat3(1.0f);

        Rm[0][0] = t * x * x + c;
        Rm[0][1] = t * x * y - s * z;
        Rm[0][2] = t * x * z + s * y;

        Rm[1][0] = t * x * y + s * z;
        Rm[1][1] = t * y * y + c;
        Rm[1][2] = t * y * z - s * x;

        Rm[2][0] = t * x * z - s * y;
        Rm[2][1] = t * y * z + s * x;
        Rm[2][2] = t * z * z + c;

        return Rm;
    }
};

#endif // AUXILIARY_H
