/**
 * @file geometry_conv.cpp
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2023 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "tesseract_robotraconteur/conv/geometry_conv.h"
#include "tesseract_robotraconteur/conv/common_conv.h"
#include "RobotRaconteurCompanion/Converters/EigenConverters.h"

#include <boost/range/algorithm.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <tesseract/common/resource_locator.h>

namespace RR = RobotRaconteur;
namespace RRC_Eigen = RobotRaconteur::Companion::Converters::Eigen;
namespace rr_color = com::robotraconteur::color;
namespace rr_image = com::robotraconteur::image;

#define RR_SHAPES_PREFIX "com.robotraconteur.geometry.shapes"

namespace tesseract_robotraconteur
{
namespace conv
{
rr_shapes::BoxPtr BoxToRR(const tesseract::geometry::Box& box)
{
    rr_shapes::BoxPtr ret(new rr_shapes::Box());
    ret->x = box.getX();
    ret->y = box.getY();
    ret->z = box.getZ();
    return ret;
}

tesseract::geometry::Box::Ptr BoxFromRR(const rr_shapes::BoxPtr& box)
{
    RR_NULL_CHECK(box);
    return std::make_shared<tesseract::geometry::Box>(box->x, box->y, box->z);
}

rr_shapes::CapsulePtr CapsuleToRR(const tesseract::geometry::Capsule& capsule)
{
    rr_shapes::CapsulePtr ret(new rr_shapes::Capsule());
    ret->radius = capsule.getRadius();
    ret->height = capsule.getLength();
    return ret;
}

tesseract::geometry::Capsule::Ptr CapsuleFromRR(const rr_shapes::CapsulePtr& capsule)
{
    RR_NULL_CHECK(capsule);
    return std::make_shared<tesseract::geometry::Capsule>(capsule->radius, capsule->height);
}

rr_shapes::ConePtr ConeToRR(const tesseract::geometry::Cone& cone)
{
    rr_shapes::ConePtr ret(new rr_shapes::Cone());
    ret->radius = cone.getRadius();
    ret->height = cone.getLength();
    return ret;
}

tesseract::geometry::Cone::Ptr ConeFromRR(const rr_shapes::ConePtr& cone)
{
    RR_NULL_CHECK(cone);
    return std::make_shared<tesseract::geometry::Cone>(cone->radius, cone->height);
}

std::tuple<rr_shapes::MeshPtr, rr_shapes::MaterialPtr> MeshToRR(const tesseract::geometry::PolygonMesh& mesh)
{
    rr_shapes::MeshPtr ret(new rr_shapes::Mesh());
    rr_shapes::MaterialPtr ret_material;

    // Tesseract faces are stored as [n, v_0, ..., v_n-1, n, ...]. Faces with more than three vertices (for
    // example convex hulls) are fan triangulated.
    auto& faces = *mesh.getFaces();
    const Eigen::Index faces_size = faces.size();
    size_t triangle_count = 0;
    for (Eigen::Index i = 0; i < faces_size; i += faces[i] + 1)
    {
        if (faces[i] < 3 || i + faces[i] >= faces_size)
        {
            throw RR::InvalidArgumentException("Invalid mesh face data");
        }
        triangle_count += static_cast<size_t>(faces[i] - 2);
    }

    ret->triangles = RR::AllocateEmptyRRNamedArray<rr_shapes::MeshTriangle>(triangle_count);
    size_t triangle_index = 0;
    for (Eigen::Index i = 0; i < faces_size; i += faces[i] + 1)
    {
        for (int j = 2; j < faces[i]; ++j)
        {
            rr_shapes::MeshTriangle& triangle = ret->triangles->at(triangle_index++);
            triangle.s.v1 = faces[i + 1];
            triangle.s.v2 = faces[i + j];
            triangle.s.v3 = faces[i + j + 1];
        }
    }

    ret->vertices = RR::AllocateEmptyRRNamedArray<rr_geom::Point>(mesh.getVertexCount());
    auto& vertices = *mesh.getVertices();
    auto& scale = mesh.getScale();
    for (size_t i = 0; i < mesh.getVertexCount(); ++i)
    {
        rr_geom::Point& point = ret->vertices->at(i);
        auto& vertex = vertices[i];
        point.s.x = vertex.x() * scale.x();
        point.s.y = vertex.y() * scale.y();
        point.s.z = vertex.z() * scale.z();
    }

    auto& normals = mesh.getNormals();
    if (normals)
    {
        ret->normals = RR::AllocateEmptyRRNamedArray<rr_geom::Vector3>(normals->size());
        for (size_t i = 0; i < normals->size(); ++i)
        {
            rr_geom::Vector3& normal = ret->normals->at(i);
            auto& n = normals->at(i);
            normal.s.x = n.x();
            normal.s.y = n.y();
            normal.s.z = n.z();
        }
    }
    else
    {
        ret->normals = RR::AllocateEmptyRRNamedArray<rr_geom::Vector3>(0);
    }

    auto& vertex_colors = mesh.getVertexColors();
    if (vertex_colors)
    {
        ret->colors = RR::AllocateEmptyRRNamedArray<rr_color::ColorRGB>(vertex_colors->size());
        for (size_t i = 0; i < vertex_colors->size(); ++i)
        {
            rr_color::ColorRGB& color = ret->colors->at(i);
            auto& c = vertex_colors->at(i);
            color.s.r = c(0);
            color.s.g = c(1);
            color.s.b = c(2);
        }
    }
    else
    {
        ret->colors = RR::AllocateEmptyRRNamedArray<rr_color::ColorRGB>(0);
    }

    auto& mesh_textures = mesh.getTextures();
    if (mesh_textures)
    {
        ret->textures = RR::AllocateEmptyRRList<rr_shapes::MeshTexture>();
        for (size_t i = 0; i < mesh_textures->size(); ++i)
        {
            ret->textures->push_back(MeshTextureToRR(*mesh_textures->at(i)));
        }
    }

    switch (mesh.getType())
    {
    case tesseract::geometry::GeometryType::MESH:
    case tesseract::geometry::GeometryType::POLYGON_MESH:
        ret->mesh_type = rr_shapes::MeshType::mesh;
        break;
    case tesseract::geometry::GeometryType::SDF_MESH:
        ret->mesh_type = rr_shapes::MeshType::sdf_mesh;
        break;
    case tesseract::geometry::GeometryType::CONVEX_MESH:
        ret->mesh_type = rr_shapes::MeshType::convex_mesh;
        break;
    default:
        throw RR::InvalidArgumentException("Unknown mesh type");
    }

    return std::make_tuple(ret, ret_material);
}

tesseract::geometry::PolygonMesh::Ptr MeshFromRR(const rr_shapes::MeshPtr& mesh)
{
    RR_NULL_CHECK(mesh);
    tesseract::geometry::PolygonMesh::Ptr ret;
    auto faces = std::make_shared<Eigen::VectorXi>(mesh->triangles->size() * 4);
    auto& faces_ref = *faces;
    for (size_t i = 0; i < mesh->triangles->size(); ++i)
    {
        const rr_shapes::MeshTriangle& triangle = mesh->triangles->at(i);
        faces_ref[i * 4 + 0] = 3;
        faces_ref[i * 4 + 1] = triangle.s.v1;
        faces_ref[i * 4 + 2] = triangle.s.v2;
        faces_ref[i * 4 + 3] = triangle.s.v3;
    }

    int face_count = boost::numeric_cast<int>(mesh->triangles->size());

    auto vertices = std::make_shared<tesseract::common::VectorVector3d>(mesh->vertices->size());
    auto& vertices_ref = *vertices;
    for (size_t i = 0; i < mesh->vertices->size(); ++i)
    {
        const rr_geom::Point& point = mesh->vertices->at(i);
        vertices_ref[i] = Eigen::Vector3d(point.s.x, point.s.y, point.s.z);
    }

    Eigen::Vector3d scale(1, 1, 1);

    std::shared_ptr<tesseract::common::VectorVector3d> normals;
    if (mesh->normals && mesh->normals->size() > 0)
    {
        normals = std::make_shared<tesseract::common::VectorVector3d>(mesh->normals->size());
        auto& normals_ref = *normals;
        for (size_t i = 0; i < mesh->normals->size(); ++i)
        {
            const rr_geom::Vector3& normal = mesh->normals->at(i);
            normals_ref[i] = Eigen::Vector3d(normal.s.x, normal.s.y, normal.s.z);
        }
    }

    std::shared_ptr<tesseract::common::VectorVector4d> vertex_colors;
    if (mesh->colors && mesh->colors->size() > 0)
    {
        vertex_colors = std::make_shared<tesseract::common::VectorVector4d>(mesh->colors->size());
        auto& vertex_colors_ref = *vertex_colors;
        for (size_t i = 0; i < mesh->colors->size(); ++i)
        {
            const rr_color::ColorRGB& color = mesh->colors->at(i);
            vertex_colors_ref[i] = Eigen::Vector4d(color.s.r, color.s.g, color.s.b, 1.0);
        }
    }

    std::shared_ptr<std::vector<tesseract::geometry::MeshTexture::Ptr>> mesh_textures;
    if (mesh->textures)
    {
        mesh_textures = std::make_shared<std::vector<tesseract::geometry::MeshTexture::Ptr>>();
        mesh_textures->reserve(mesh->textures->size());
        boost::range::transform(*mesh->textures, std::back_inserter(*mesh_textures), MeshTextureFromRR);
    }

    switch (mesh->mesh_type)
    {
    case rr_shapes::MeshType::mesh:
        ret = std::make_shared<tesseract::geometry::PolygonMesh>(vertices, faces, face_count, nullptr, scale, normals,
                                                                 vertex_colors, nullptr, mesh_textures);
        break;
    case rr_shapes::MeshType::sdf_mesh:
        ret = std::make_shared<tesseract::geometry::SDFMesh>(vertices, faces, face_count, nullptr, scale, normals,
                                                             vertex_colors, nullptr, mesh_textures);
        break;
    case rr_shapes::MeshType::convex_mesh:
        ret = std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, face_count, nullptr, scale, normals,
                                                                vertex_colors, nullptr, mesh_textures);
        break;
    default:
        throw RR::InvalidArgumentException("Unknown mesh type");
    }

    return ret;
}

rr_shapes::CylinderPtr CylinderToRR(const tesseract::geometry::Cylinder& cylinder)
{
    rr_shapes::CylinderPtr ret(new rr_shapes::Cylinder());
    ret->radius = cylinder.getRadius();
    ret->height = cylinder.getLength();
    return ret;
}

tesseract::geometry::Cylinder::Ptr CylinderFromRR(const rr_shapes::CylinderPtr& cylinder)
{
    RR_NULL_CHECK(cylinder);
    return std::make_shared<tesseract::geometry::Cylinder>(cylinder->radius, cylinder->height);
}

// TODO: octree

rr_shapes::PlanePtr PlaneToRR(const tesseract::geometry::Plane& plane)
{
    rr_shapes::PlanePtr ret(new rr_shapes::Plane());
    ret->a = plane.getA();
    ret->b = plane.getB();
    ret->c = plane.getC();
    ret->d = plane.getD();
    return ret;
}

tesseract::geometry::Plane::Ptr PlaneFromRR(const rr_shapes::PlanePtr& plane)
{
    RR_NULL_CHECK(plane);
    return std::make_shared<tesseract::geometry::Plane>(plane->a, plane->b, plane->c, plane->d);
}

rr_shapes::SpherePtr SphereToRR(const tesseract::geometry::Sphere& sphere)
{
    rr_shapes::SpherePtr ret(new rr_shapes::Sphere());
    ret->radius = sphere.getRadius();
    return ret;
}

tesseract::geometry::Sphere::Ptr SphereFromRR(const rr_shapes::SpherePtr& sphere)
{
    RR_NULL_CHECK(sphere);
    return std::make_shared<tesseract::geometry::Sphere>(sphere->radius);
}

tesseract::geometry::Geometry::Ptr GeometryFromRR(const RobotRaconteur::RRValuePtr& geom)
{
    RR_NULL_CHECK(geom);
    std::string rr_type = geom->RRType();
    if (rr_type == RR_SHAPES_PREFIX ".Box")
    {
        return BoxFromRR(RR::rr_cast<rr_shapes::Box>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Capsule")
    {
        return CapsuleFromRR(RR::rr_cast<rr_shapes::Capsule>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Cone")
    {
        return ConeFromRR(RR::rr_cast<rr_shapes::Cone>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Cylinder")
    {
        return CylinderFromRR(RR::rr_cast<rr_shapes::Cylinder>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Plane")
    {
        return PlaneFromRR(RR::rr_cast<rr_shapes::Plane>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Sphere")
    {
        return SphereFromRR(RR::rr_cast<rr_shapes::Sphere>(geom));
    }
    else if (rr_type == RR_SHAPES_PREFIX ".Mesh")
    {
        return MeshFromRR(RR::rr_cast<rr_shapes::Mesh>(geom));
    }
    else
    {
        throw RR::InvalidArgumentException("Unknown geometry type");
    }
}

RobotRaconteur::RRValuePtr GeometryToRR(const tesseract::geometry::Geometry& geom)
{
    switch (geom.getType())
    {
    case tesseract::geometry::GeometryType::BOX:
        return BoxToRR(static_cast<const tesseract::geometry::Box&>(geom));
    case tesseract::geometry::GeometryType::CAPSULE:
        return CapsuleToRR(static_cast<const tesseract::geometry::Capsule&>(geom));
    case tesseract::geometry::GeometryType::CONE:
        return ConeToRR(static_cast<const tesseract::geometry::Cone&>(geom));
    case tesseract::geometry::GeometryType::CYLINDER:
        return CylinderToRR(static_cast<const tesseract::geometry::Cylinder&>(geom));
    case tesseract::geometry::GeometryType::PLANE:
        return PlaneToRR(static_cast<const tesseract::geometry::Plane&>(geom));
    case tesseract::geometry::GeometryType::SPHERE:
        return SphereToRR(static_cast<const tesseract::geometry::Sphere&>(geom));
    case tesseract::geometry::GeometryType::MESH:
    case tesseract::geometry::GeometryType::POLYGON_MESH:
    case tesseract::geometry::GeometryType::SDF_MESH:
    case tesseract::geometry::GeometryType::CONVEX_MESH:
        return std::get<0>(MeshToRR(static_cast<const tesseract::geometry::PolygonMesh&>(geom)));
    default:
        throw RR::InvalidArgumentException("Unknown geometry type");
    }
}

rr_shapes::MaterialPtr MeshMaterialToRR(const tesseract::geometry::MeshMaterial& mesh_material)
{
    throw std::runtime_error("Not implemented");
}

tesseract::geometry::MeshMaterial::Ptr MeshMaterialFromRR(const rr_shapes::MaterialPtr& mesh_material)
{
    throw std::runtime_error("Not implemented");
}

rr_shapes::MeshTexturePtr MeshTextureToRR(tesseract::geometry::MeshTexture& mesh_texture)
{
    rr_shapes::MeshTexturePtr ret(new rr_shapes::MeshTexture());
    auto& uvs = mesh_texture.getUVs();
    auto& uvs_ref = *uvs;
    ret->uvs = RR::AllocateEmptyRRNamedArray<rr_geom::Vector2>(uvs->size());
    for (size_t i = 0; i < uvs->size(); ++i)
    {
        rr_geom::Vector2& point = ret->uvs->at(i);
        auto& uv = uvs_ref[i];
        point.s.x = uv.x();
        point.s.y = uv.y();
    }

    rr_image::CompressedImagePtr image(new rr_image::CompressedImage());
    auto img_res = mesh_texture.getTextureImage();
    if (!img_res)
    {
        throw RR::InvalidArgumentException("Texture image resource is null");
    }
    auto img_data = img_res->getResourceContents();
    image->data = RR::AttachRRArrayCopy(img_data.data(), img_data.size());

    rr_image::ImageInfoPtr image_info(new rr_image::ImageInfo());
    image_info->width = 0;
    image_info->height = 0;
    image_info->step = 0;
    image_info->encoding = rr_image::ImageEncoding::compressed;

    image->image_info = image_info;

    ret->image = image;

    return ret;
}

tesseract::geometry::MeshTexture::Ptr MeshTextureFromRR(const rr_shapes::MeshTexturePtr& mesh_texture)
{
    RR_NULL_CHECK(mesh_texture);
    RR_NULL_CHECK(mesh_texture->image);
    RR_NULL_CHECK(mesh_texture->uvs);
    RR_NULL_CHECK(mesh_texture->image->data);

    auto uvs = std::make_shared<tesseract::common::VectorVector2d>(mesh_texture->uvs->size());
    auto& uvs_ref = *uvs;
    for (size_t i = 0; i < mesh_texture->uvs->size(); ++i)
    {
        const rr_geom::Vector2& point = mesh_texture->uvs->at(i);
        uvs_ref[i] = Eigen::Vector2d(point.s.x, point.s.y);
    }

    auto& rr_data = mesh_texture->image->data;
    auto data = std::vector<uint8_t>(rr_data->data(), rr_data->data() + rr_data->size());

    auto resource = std::make_shared<tesseract::common::BytesResource>("", std::move(data));

    return std::make_shared<tesseract::geometry::MeshTexture>(resource, uvs);
}

} // namespace conv
} // namespace tesseract_robotraconteur
