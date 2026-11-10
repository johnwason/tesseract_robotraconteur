/**
 * @file geometry_conv.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H
#define TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/mesh_material.h>

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;

namespace tesseract_robotraconteur
{
namespace conv
{
rr_shapes::BoxPtr BoxToRR(const tesseract::geometry::Box& box);

tesseract::geometry::Box::Ptr BoxFromRR(const rr_shapes::BoxPtr& box);

rr_shapes::CapsulePtr CapsuleToRR(const tesseract::geometry::Capsule& capsule);

tesseract::geometry::Capsule::Ptr CapsuleFromRR(const rr_shapes::CapsulePtr& capsule);

rr_shapes::ConePtr ConeToRR(const tesseract::geometry::Cone& cone);

tesseract::geometry::Cone::Ptr ConeFromRR(const rr_shapes::ConePtr& cone);

std::tuple<rr_shapes::MeshPtr, rr_shapes::MaterialPtr> MeshToRR(const tesseract::geometry::PolygonMesh& mesh);

tesseract::geometry::PolygonMesh::Ptr MeshFromRR(const rr_shapes::MeshPtr& mesh);

rr_shapes::CylinderPtr CylinderToRR(const tesseract::geometry::Cylinder& cylinder);

tesseract::geometry::Cylinder::Ptr CylinderFromRR(const rr_shapes::CylinderPtr& cylinder);

// TODO: octree

rr_shapes::PlanePtr PlaneToRR(const tesseract::geometry::Plane& plane);

tesseract::geometry::Plane::Ptr PlaneFromRR(const rr_shapes::PlanePtr& plane);

rr_shapes::SpherePtr SphereToRR(const tesseract::geometry::Sphere& sphere);

tesseract::geometry::Sphere::Ptr SphereFromRR(const rr_shapes::SpherePtr& sphere);

tesseract::geometry::Geometry::Ptr GeometryFromRR(const RobotRaconteur::RRValuePtr& geom);

RobotRaconteur::RRValuePtr GeometryToRR(const tesseract::geometry::Geometry& geom);

rr_shapes::MaterialPtr MeshMaterialToRR(const tesseract::geometry::MeshMaterial& mesh_material);

tesseract::geometry::MeshMaterial::Ptr MeshMaterialFromRR(const rr_shapes::MaterialPtr& mesh_material);

rr_shapes::MeshTexturePtr MeshTextureToRR(tesseract::geometry::MeshTexture& mesh_texture);

tesseract::geometry::MeshTexture::Ptr MeshTextureFromRR(const rr_shapes::MeshTexturePtr& mesh_texture);

} // namespace conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H
