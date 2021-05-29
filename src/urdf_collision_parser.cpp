/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Dawid Seredynski */

#include "collision_convex_model/collision_convex_model.h"
#include "narrowphase.h"

namespace self_collision
{

fcl_2::GJKSolver_indep CollisionModel::gjk_solver;

Geometry::Geometry(int type) :
    type_(type),
    broadphase_radius_(100000.0)
{
}

int Geometry::getType() const {
    return type_;
}

static const std::string geom_type_str[] = {"UNDEFINED", "CAPSULE", "CONVEX", "SPHERE", "TRIANGLE", "OCTOMAP"};

const std::string& Geometry::getTypeStr() const {
    return geom_type_str[type_];
}

double Geometry::getBroadphaseRadius() const {
    return broadphase_radius_;
}

void Geometry::setColor(double cr, double cg, double cb, double ca) {
    cr_ = cr;
    cg_ = cg;
    cb_ = cb;
    ca_ = ca;
}

void Geometry::getColor(double &cr, double &cg, double &cb, double &ca) const {
    cr = cr_;
    cg = cg_;
    cb = cb_;
    ca = ca_;
}

Octomap::Octomap() :
    Geometry(OCTOMAP)
{
}

Octomap::Octomap(const boost::shared_ptr<octomap::OcTree > &om) :
    Geometry(OCTOMAP),
    om_(om)
{
}

void Octomap::setOctomap(const boost::shared_ptr<octomap::OcTree > &om) {
    om_ = om;
}

const boost::shared_ptr<octomap::OcTree >& Octomap::getOctomap() const {
    return om_;
}

void Octomap::clear() {
    std::cout << "Octomap::clear() is not implemented" << std::endl;
}

void Octomap::addMarkers(visualization_msgs::MarkerArray &marker_array) {
}

void Octomap::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr){
}

Capsule::Capsule() :
    Geometry(CAPSULE)
{
}

Capsule::Capsule(double radius, double length) :
    Geometry(CAPSULE)
{
    setSize(radius, length);
}

void Capsule::setSize(double radius, double length) {
    this->radius = radius;
    this->length = length;
    broadphase_radius_ = this->length/2.0 + this->radius;
    shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(radius, length)) );
}

double Capsule::getRadius() const {
    return radius;
}

double Capsule::getLength() const {
    return length;
}

void Capsule::clear()
{
    radius = 0.0;
    length = 0.0;
}

void Capsule::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
    const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(shape.get());

    marker_id_ = 0;
    if (marker_array.markers.size() > 0)
    {
        marker_id_ = marker_array.markers.back().id + 1;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "default";
    marker.id = marker_id_;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = ob->radius * 2.0;
    marker.scale.y = ob->radius * 2.0;
    marker.scale.z = ob->radius * 2.0;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "world";
    marker2.ns = "default";
    marker2.id = marker_id_ + 1;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.scale.x = ob->radius * 2.0;
    marker2.scale.y = ob->radius * 2.0;
    marker2.scale.z = ob->radius * 2.0;
    marker2.color.a = 0.5;
    marker2.color.r = 0.0;
    marker2.color.g = 0.0;
    marker2.color.b = 1.0;
    marker_array.markers.push_back(marker2);

    visualization_msgs::Marker marker3;
    marker3.header.frame_id = "world";
    marker3.ns = "default";
    marker3.id = marker_id_+2;
    marker3.type = visualization_msgs::Marker::CYLINDER;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.scale.x = ob->radius * 2.0;
    marker3.scale.y = ob->radius * 2.0;
    marker3.scale.z = ob->lz;
    marker3.color.a = 0.5;
    marker3.color.r = 0.0;
    marker3.color.g = 0.0;
    marker3.color.b = 1.0;
    marker_array.markers.push_back(marker3);

/*    visualization_msgs::Marker marker4;
    marker4.header.frame_id = "world";
    marker4.ns = "default";
    marker4.id = marker_id_+3;
    marker4.type = visualization_msgs::Marker::SPHERE;
    marker4.action = visualization_msgs::Marker::ADD;
    marker4.scale.x = 0.01;
    marker4.scale.y = 0.01;
    marker4.scale.z = 0.01;
    marker4.color.a = 1.0;
    marker4.color.r = 0.0;
    marker4.color.g = 1.0;
    marker4.color.b = 0.0;
    marker_array.markers.push_back(marker4);
*/
}

void Capsule::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
    const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(shape.get());

    double qx, qy, qz, qw;
    fr.M.GetQuaternion(qx, qy, qz, qw);
    KDL::Vector v(0,0,ob->lz);
    KDL::Vector v2 = (fr * v) - (fr * KDL::Vector());

    if (marker_id_+3 >= marker_array.markers.size())
    {
        std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
        *((int*)NULL) = 1;
    }

    if (marker_array.markers[marker_id_].id != marker_id_)
    {
        std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
        *((int*)NULL) = 1;
    }
    marker_array.markers[marker_id_].header.stamp = ros::Time();
    marker_array.markers[marker_id_].pose.position.x = fr.p.x() - v2.x()/2.0;
    marker_array.markers[marker_id_].pose.position.y = fr.p.y() - v2.y()/2.0;
    marker_array.markers[marker_id_].pose.position.z = fr.p.z() - v2.z()/2.0;
    marker_array.markers[marker_id_].pose.orientation.x = qx;
    marker_array.markers[marker_id_].pose.orientation.y = qy;
    marker_array.markers[marker_id_].pose.orientation.z = qz;
    marker_array.markers[marker_id_].pose.orientation.w = qw;

    if (marker_array.markers[marker_id_+1].id != marker_id_+1)
    {
        std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
        *((int*)NULL) = 1;
    }
    if (marker_array.markers[marker_id_+2].id != marker_id_+2)
    {
        std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
    }
    if (marker_array.markers[marker_id_+3].id != marker_id_+3)
    {
        std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
        *((int*)NULL) = 1;
    }
    marker_array.markers[marker_id_+1].header.stamp = ros::Time();
    marker_array.markers[marker_id_+1].pose.position.x = fr.p.x() + v2.x()/2.0;
    marker_array.markers[marker_id_+1].pose.position.y = fr.p.y() + v2.y()/2.0;
    marker_array.markers[marker_id_+1].pose.position.z = fr.p.z() + v2.z()/2.0;
    marker_array.markers[marker_id_+1].pose.orientation.x = qx;
    marker_array.markers[marker_id_+1].pose.orientation.y = qy;
    marker_array.markers[marker_id_+1].pose.orientation.z = qz;
    marker_array.markers[marker_id_+1].pose.orientation.w = qw;

    marker_array.markers[marker_id_+2].header.stamp = ros::Time();
    marker_array.markers[marker_id_+2].pose.position.x = fr.p.x();
    marker_array.markers[marker_id_+2].pose.position.y = fr.p.y();
    marker_array.markers[marker_id_+2].pose.position.z = fr.p.z();
    marker_array.markers[marker_id_+2].pose.orientation.x = qx;
    marker_array.markers[marker_id_+2].pose.orientation.y = qy;
    marker_array.markers[marker_id_+2].pose.orientation.z = qz;
    marker_array.markers[marker_id_+2].pose.orientation.w = qw;

/*    marker_array.markers[marker_id_+3].header.stamp = ros::Time();
    marker_array.markers[marker_id_+3].pose.position.x = fr.p.x();
    marker_array.markers[marker_id_+3].pose.position.y = fr.p.y();
    marker_array.markers[marker_id_+3].pose.position.z = fr.p.z();
*/
}

Sphere::Sphere() :
    Geometry(SPHERE)
{
}

Sphere::Sphere(double radius) :
    Geometry(SPHERE)
{
    setSize(radius);
}

void Sphere::setSize(double radius) {
    this->radius = radius;
    broadphase_radius_ = this->radius;
    shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Sphere(radius)) );
}

double Sphere::getRadius() const {
    return radius;
}

void Sphere::clear()
{
    radius = 0.0;
}

void Sphere::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
}

void Sphere::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
}

Triangle::Triangle() :
    Geometry(TRIANGLE)
{
}

Triangle::Triangle(const KDL::Vector &v1, const KDL::Vector &v2, const KDL::Vector &v3) :
    Geometry(TRIANGLE)
{
    setPoints(v1, v2, v3);
}

void Triangle::setPoints(const KDL::Vector &v1, const KDL::Vector &v2, const KDL::Vector &v3) {
    v1_ = v1;
    v2_ = v2;
    v3_ = v3;
    double radius = 0.0;
    if (v1_.Norm() > radius) {
        radius = v1_.Norm();
    }
    if (v2_.Norm() > radius) {
        radius = v2_.Norm();
    }
    if (v3_.Norm() > radius) {
        radius = v3_.Norm();
    }
    broadphase_radius_ = radius;

    KDL::Vector v12 = v2_-v1_;
    KDL::Vector v13 = v3_-v1_;
    KDL::Vector nx = v12;
    KDL::Vector nz = nx * v13;
    KDL::Vector normal = nz;
    KDL::Vector ny = nz * nx;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();

    T_O_L_ = KDL::Frame(KDL::Rotation(nx, ny, nz), KDL::Vector( (v1_ + v2_ + v3_)/3.0 ));
    T_L_O_ = T_O_L_.Inverse();

    v1_L_ = T_L_O_ * v1_;
    v2_L_ = T_L_O_ * v2_;
    v3_L_ = T_L_O_ * v3_;

    c1_.reset( new fcl_2::Capsule(0.0, v1_.Norm()) );
    c2_.reset( new fcl_2::Capsule(0.0, v2_.Norm()) );
    c3_.reset( new fcl_2::Capsule(0.0, v3_.Norm()) );

    nz = v2_ - v1_;
    nx = normal;
    ny = nz * nx;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();
    T_O_C1_ = KDL::Frame(KDL::Rotation(nx, ny, nz), KDL::Vector( (v1_ + v2_) / 2.0 ));

    nz = v3_ - v2_;
    nx = normal;
    ny = nz * nx;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();
    T_O_C2_ = KDL::Frame(KDL::Rotation(nx, ny, nz), KDL::Vector( (v2_ + v3_) / 2.0 ));

    nz = v1_ - v3_;
    nx = normal;
    ny = nz * nx;
    nx.Normalize();
    ny.Normalize();
    nz.Normalize();
    T_O_C3_ = KDL::Frame(KDL::Rotation(nx, ny, nz), KDL::Vector( (v3_ + v1_) / 2.0 ));
}

void Triangle::getPoints(KDL::Vector &v1, KDL::Vector &v2, KDL::Vector &v3) const {
    v1 = v1_;
    v2 = v2_;
    v3 = v3_;
}

void Triangle::clear()
{
}

void Triangle::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
}

void Triangle::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
}

Convex::Convex() :
    Geometry(CONVEX),
    max_points_vec_size_(5000),
    max_polygons_vec_size_(20000)
{
    // allocate a lot of memory and initialize very simple convex hull mesh (4 points and 4 polygons)
    int num_points = 4;
    fcl_2::Vec3f* points = new fcl_2::Vec3f[max_points_vec_size_];
    int num_planes = 4;
    int *polygons = new int[max_polygons_vec_size_];
    points[0] = fcl_2::Vec3f(0.0, 0.0, 0.0);
    points[1] = fcl_2::Vec3f(0.1, 0.0, 0.0);
    points[2] = fcl_2::Vec3f(0.0, 0.1, 0.0);
    points[3] = fcl_2::Vec3f(0.0, 0.0, 0.1);
    polygons[0] = 3;    polygons[1] = 2;    polygons[2] = 1;    polygons[3] = 0;
    polygons[4] = 3;    polygons[5] = 3;    polygons[6] = 1;    polygons[7] = 0;
    polygons[8] = 3;    polygons[9] = 3;    polygons[10] = 2;    polygons[11] = 0;
    polygons[12] = 3;    polygons[13] = 3;    polygons[14] = 2;    polygons[15] = 1;
    shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)) );
    calculateRadius();
    updateInternalData();
}

const std::vector<KDL::Vector >& Convex::getPoints() const {
    return points_;
}

const std::vector<int >& Convex::getPolygons() const {
    return polygons_;
}

void Convex::calculateRadius() {
    fcl_2::Convex *conv = static_cast<fcl_2::Convex* >(shape.get());
    broadphase_radius_ = 0.0;
    for (int i=0; i<conv->num_points; i++)
    {
        double dist = sqrt(conv->points[i][0]*conv->points[i][0] + conv->points[i][1]*conv->points[i][1] + conv->points[i][2]*conv->points[i][2]);
        if (dist > broadphase_radius_) {
            broadphase_radius_ = dist;
        }
    }
}

void Convex::updateInternalData() {
    fcl_2::Convex *conv = static_cast<fcl_2::Convex* >(shape.get());
    points_.clear();
    polygons_.clear();
    for (int i=0; i<conv->num_points; i++) {
        points_.push_back( KDL::Vector(conv->points[i][0], conv->points[i][1], conv->points[i][2]) );
    }

    int polygons_idx = 0;
    for (int f_idx=0; f_idx<conv->num_planes; f_idx++)
    {
        polygons_idx += conv->polygons[polygons_idx] + 1;
    }

    for (int i=0; i<polygons_idx; i++) {
        polygons_.push_back(conv->polygons[i]);
    }

/*    int next_poly_idx = 0;
    int poly_idx = 0;
    for (int i=0; ; i++) {
//        std::cout << i << " " << poly_idx << " " << conv->polygons[i] << std::endl;
//        getchar();
        if (next_poly_idx == i) {
            next_poly_idx += conv->polygons[i] + 1;
            poly_idx++;
            if (poly_idx >= conv->num_planes) {
                break;
            }
        }
        polygons_.push_back(conv->polygons[i]);
    }
*/
}

Convex::~Convex()
{
    fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
    delete[] conv->polygons;
    conv->polygons = NULL;
    delete[] conv->points;
    conv->points = NULL;
}

bool Convex::updateConvex(int num_points, const std::vector<KDL::Vector> &points, int num_planes, const std::vector<int> &polygons) {
    if (num_points > max_points_vec_size_) {
        return false;
    }

    fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
    int polygons_idx = 0;
    for (int f_idx=0; f_idx<num_planes; f_idx++)
    {
        polygons_idx += polygons[polygons_idx] + 1;
    }

    if (polygons_idx > max_polygons_vec_size_) {
        return false;
    }

    for (int i=0; i<polygons_idx; ++i)
    {
        conv->polygons[i] = polygons[i];
    }

    conv->num_planes = num_planes;

    for (int p_idx=0; p_idx<num_points; p_idx++)
    {
        conv->points[p_idx] = fcl_2::Vec3f(points[p_idx].x(), points[p_idx].y(), points[p_idx].z());
    }

    conv->num_points = num_points;
    conv->fillEdges();
    calculateRadius();
    updateInternalData();

    return true;
}

bool Convex::updateConvex(int num_points, const std::vector<geometry_msgs::Point> &points, int num_planes, const std::vector<int> &polygons) {
    if (num_points > max_points_vec_size_) {
        return false;
    }

    fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
    int polygons_idx = 0;
    for (int f_idx=0; f_idx<num_planes; f_idx++)
    {
        polygons_idx += polygons[polygons_idx] + 1;
    }

    if (polygons_idx > max_polygons_vec_size_) {
        return false;
    }

    for (int i=0; i<polygons_idx; ++i)
    {
        conv->polygons[i] = polygons[i];
    }

    conv->num_planes = num_planes;

    for (int p_idx=0; p_idx<num_points; p_idx++)
    {
        conv->points[p_idx] = fcl_2::Vec3f(points[p_idx].x, points[p_idx].y, points[p_idx].z);
    }

    conv->num_points = num_points;
    conv->fillEdges();
    calculateRadius();
    updateInternalData();

    return true;
}

void Convex::clear()
{
    points_str_.clear();
    points_id_.clear();
}

void Convex::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
    const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(shape.get());

    marker_id_ = 0;
    if (marker_array.markers.size() > 0)
    {
        marker_id_ = marker_array.markers.back().id + 1;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "default";
    marker.id = marker_id_;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(500);
    marker.scale.x = 0.005;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
}

void Convex::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
    const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(shape.get());

    double qx, qy, qz, qw;
    fr.M.GetQuaternion(qx, qy, qz, qw);

    const fcl_2::Vec3f *points = ob->points;
    int num_planes = ob->num_planes;
    const int *polygons = ob->polygons;

    // edges
    marker_array.markers[marker_id_].header.stamp = ros::Time();
    marker_array.markers[marker_id_].pose.position.x = fr.p.x();
    marker_array.markers[marker_id_].pose.position.y = fr.p.y();
    marker_array.markers[marker_id_].pose.position.z = fr.p.z();
    marker_array.markers[marker_id_].pose.orientation.x = qx;
    marker_array.markers[marker_id_].pose.orientation.y = qy;
    marker_array.markers[marker_id_].pose.orientation.z = qz;
    marker_array.markers[marker_id_].pose.orientation.w = qw;
    int pt_size = marker_array.markers[marker_id_].points.size();
    int pt_idx = 0;
    int poly_idx = 0;
    for (int i=0; i<num_planes; i++)
    {
        int points_in_poly = polygons[poly_idx];
        for (int j=0; j<points_in_poly; j++)
        {
            geometry_msgs::Point pt1;
            pt1.x = points[polygons[poly_idx + 1 + j]][0];
            pt1.y = points[polygons[poly_idx + 1 + j]][1];
            pt1.z = points[polygons[poly_idx + 1 + j]][2];
            marker_array.markers[marker_id_].points[pt_idx++] = pt1;
            if (pt_idx >= pt_size)
            {
                break;
            }

            geometry_msgs::Point pt2;
            pt2.x = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][0];
            pt2.y = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][1];
            pt2.z = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][2];
            marker_array.markers[marker_id_].points[pt_idx++] = pt2;
            if (pt_idx >= pt_size)
            {
                break;
            }
        }
        if (pt_idx >= pt_size)
        {
            break;
        }
        poly_idx += points_in_poly+1;
    }

    for (; pt_idx<pt_size; pt_idx++)
    {
        marker_array.markers[marker_id_].points[pt_idx].x = 0;
        marker_array.markers[marker_id_].points[pt_idx].y = 0;
        marker_array.markers[marker_id_].points[pt_idx].z = 0;
    }
}

VisualGeometry::VisualGeometry(int type)
    : type_(type)
{}

VisualMesh::VisualMesh()
    : VisualGeometry(MESH)
{}

int VisualGeometry::getType() const {
    return type_;
}

Visual::Visual()
{}

void Collision::clear()
{
    // TODO
}

Link::Link() :
    kdl_segment_(NULL)
{
}

void Link::clear()
{
    // TODO
}

void Joint::clear()
{
    // TODO
}

const boost::shared_ptr< Link > CollisionModel::getLink(int id) const {
    if (id < 0 || id >= link_count_)
    {
        ROS_ERROR("CollisionModel::getLink: id out of range: 0 <= %d < %d", id, link_count_);
        return boost::shared_ptr< Link >();
    }
    return links_[id];
}

const boost::shared_ptr< Link > CollisionModel::getLink(const std::string &link_name) const {
    int idx = getLinkIndex(link_name);
    if (idx < 0) {
        return boost::shared_ptr< Link >();
    }
    return getLink(idx);
}

int CollisionModel::getLinkIndex(const std::string &name) const {
    std::map<std::string, int >::const_iterator idx_it = link_name_idx_map_.find(name);
    if (idx_it == link_name_idx_map_.end()) {
        return -1;
    }

    return idx_it->second;
}

const std::string &CollisionModel::getLinkName(int idx) const {
    return links_[idx]->name;
}

KDL::Vector initVectorFromString(const std::string &vector_str)
{
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    boost::split( pieces, vector_str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i){
        if (pieces[i] != "")
        {
            try
            {
                xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
            }
            catch (boost::bad_lexical_cast &e)
            {
                ROS_ERROR("Vector3 xyz element (%s) is not a valid float",pieces[i].c_str());
                return KDL::Vector();
            }
        }
    }

    if (xyz.size() != 3)
    {
        ROS_ERROR("Vector contains %i elements instead of 3 elements", (int)xyz.size()); 
        return KDL::Vector();
    }
    return KDL::Vector(xyz[0], xyz[1], xyz[2]);
};

CollisionModel::CollisionModel() :
    link_count_(0)
{
}

bool CollisionModel::parsePose(KDL::Frame &pose, TiXmlElement* xml)
{
    KDL::Vector pos;
    KDL::Vector rpy;
    if (xml)
    {
        const char* xyz_str = xml->Attribute("xyz");
        if (xyz_str != NULL)
        {
            try
            {
                pos = initVectorFromString(xyz_str);
            }
            catch (urdf::ParseError &e)
            {
                ROS_ERROR("%s", e.what());
                return false;
            }
        }
        const char* rpy_str = xml->Attribute("rpy");
        if (rpy_str != NULL)
        {
            try
            {
                rpy = initVectorFromString(rpy_str);
            }
            catch (urdf::ParseError &e)
            {
                ROS_ERROR("%s", e.what());
                return false;
            }
        }
    }
    KDL::Frame pose_ret( KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()), pos );
    pose = pose_ret;
    return true;
}

bool CollisionModel::parseCapsule(Capsule &s, TiXmlElement *c)
{
    s.clear();
    double radius, length;
    if (!c->Attribute("radius"))
    {
        ROS_ERROR("Capsule shape must have a radius attribute");
        return false;
    }
    try
    {
        radius = boost::lexical_cast<double>(c->Attribute("radius"));
    }
    catch (boost::bad_lexical_cast &e)
    {
        std::stringstream stm;
        stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
        ROS_ERROR("%s", stm.str().c_str());
        return false;
    }

    if (!c->Attribute("length"))
    {
        ROS_ERROR("Capsule shape must have a length attribute");
        return false;
    }
    try
    {
        length = boost::lexical_cast<double>(c->Attribute("length"));
    }
    catch (boost::bad_lexical_cast &e)
    {
        std::stringstream stm;
        stm << "length [" << c->Attribute("length") << "] is not a valid float: " << e.what();
        ROS_ERROR("%s", stm.str().c_str());
        return false;
    }

    s.setSize(radius, length);
    return true;
}

bool CollisionModel::parseSphere(Sphere &s, TiXmlElement *c)
{
    s.clear();
    double radius;
    if (!c->Attribute("radius"))
    {
        ROS_ERROR("Capsule shape must have a radius attribute");
        return false;
    }
    try
    {
        radius = boost::lexical_cast<double>(c->Attribute("radius"));
    }
    catch (boost::bad_lexical_cast &e)
    {
        std::stringstream stm;
        stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
        ROS_ERROR("%s", stm.str().c_str());
        return false;
    }

    s.setSize(radius);
    return true;
}

bool CollisionModel::parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c)
{
    if (c)
    {
        const char* xyz_str = c->Attribute("xyz");
        if (xyz_str != NULL)
        {
            try
            {
                p = initVectorFromString(xyz_str);
            }
            catch (urdf::ParseError &e)
            {
                ROS_ERROR("%s", e.what());
                return false;
            }
        }
        const char* frame_str = c->Attribute("frame");
        if (frame_str != NULL)
        {
            try
            {
                frame = frame_str;
            }
            catch (urdf::ParseError &e)
            {
                ROS_ERROR("%s", e.what());
                return false;
            }
        }
    }
    return true;
}

bool CollisionModel::parseConvex(Convex &s, TiXmlElement *c)
{
    s.clear();

    for (TiXmlElement* col_xml = c->FirstChildElement("point"); col_xml; col_xml = col_xml->NextSiblingElement("point"))
    {
        std::string frame;
        KDL::Vector point;
        if (parsePoint(frame, point, col_xml))
        {
            s.points_str_.push_back(std::pair<std::string, KDL::Vector>(frame, point));
        }
        else
        {
            ROS_ERROR("Could not parse point element for Convex");
            return false;
        }
    }

    return true;
}

boost::shared_ptr<Geometry> CollisionModel::parseGeometry(TiXmlElement *g)
{
    boost::shared_ptr<Geometry> geom;
    if (!g) return geom;
    TiXmlElement *shape_xml = g->FirstChildElement();
    if (!shape_xml)
    {
        ROS_ERROR("Geometry tag contains no child element.");
        return geom;
    }
    std::string type_name = shape_xml->ValueStr();
    if (type_name == "capsule")
    {
        Capsule *s = new Capsule();
        geom.reset(s);
        if (parseCapsule(*s, shape_xml))
        {
            return geom;
        }
    }
    else if (type_name == "sphere")
    {
        Sphere *s = new Sphere();
        geom.reset(s);
        if (parseSphere(*s, shape_xml))
        {
            return geom;
        }
    }
    else if (type_name == "convex")
    {
        Convex *s = new Convex();
        geom.reset(s);
        if (parseConvex(*s, shape_xml))
        {
            return geom;
        }
    }
    else
    {
        ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
        return geom;
    }
    return boost::shared_ptr<Geometry>();
}

bool CollisionModel::parseCollision(Collision &col, TiXmlElement* config)
{
    col.clear();
    // Origin
    TiXmlElement *o = config->FirstChildElement("origin");
    if (o) {
        if (!parsePose(col.origin, o))
            return false;
    }
    // Geometry
    TiXmlElement *geom = config->FirstChildElement("geometry");
    col.geometry = parseGeometry(geom);
    if (!col.geometry)
        return false;
    const char *name_char = config->Attribute("name");
    return true;
}

boost::shared_ptr<VisualGeometry > CollisionModel::parseVisualGeometry(TiXmlElement* config) {
    // mesh
    TiXmlElement *mesh = config->FirstChildElement("mesh");
    if (mesh) {
        boost::shared_ptr<VisualMesh > result(new VisualMesh());
        result->filename_ = mesh->Attribute("filename");
        return boost::dynamic_pointer_cast<VisualGeometry >(result);
    }
    return boost::shared_ptr<VisualGeometry >();
}

boost::shared_ptr<Visual > CollisionModel::parseVisual(TiXmlElement* config) {
    boost::shared_ptr<Visual > result(new Visual());

    // Origin
    TiXmlElement *o = config->FirstChildElement("origin");
    if (o) {
        if (!parsePose(result->origin_, o)) {
            return boost::shared_ptr<Visual >();
        }
    }
    // Geometry
    TiXmlElement *geom = config->FirstChildElement("geometry");
    result->geom_ = parseVisualGeometry(geom);
    if (!result->geom_) {
        return boost::shared_ptr<Visual >();
    }
    return result;
}

bool CollisionModel::parseLink(Link &link, TiXmlElement* config)
{
    link.clear();
    const char *name_char = config->Attribute("name");
    if (!name_char)
    {
        ROS_ERROR("No name given for the link.");
        return false;
    }
    link.name = std::string(name_char);
    // Multiple Collisions (optional)
    for (TiXmlElement* col_xml = config->FirstChildElement("self_collision_checking"); col_xml; col_xml = col_xml->NextSiblingElement("self_collision_checking"))
    {
        boost::shared_ptr<Collision> col(new Collision());
        col->parent_link_idx_ = link.index_;
        if (parseCollision(*col, col_xml))
        {
            link.collision_array.push_back(col);
        }
        else
        {
            ROS_ERROR("Could not parse collision element for Link [%s]", link.name.c_str());
            return false;
        }
    }

    // Multiple Visuals (optional)
    for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
    {
        boost::shared_ptr<Visual > vis = parseVisual(vis_xml);
        if (vis) {
            link.visual_array_.push_back(vis);
        }
        else {
            //ROS_INFO("Could not parse visual element for Link [%s]", link.name.c_str());
            //return false; // no error
        }
    }
    return true;
}

bool CollisionModel::parseLimit(Joint &joint, TiXmlElement* o)
{
// e.g. <limit effort="100" lower="-2.792444444" upper="2.792444444" velocity="100"/>

    const char *lower_char = o->Attribute("lower");
    const char *upper_char = o->Attribute("upper");
    if (!lower_char)
    {
        ROS_ERROR("No lower limit given for the joint.");
        return false;
    }
    if (!upper_char)
    {
        ROS_ERROR("No lower limit given for the joint.");
        return false;
    }

    try
    {
        joint.lower_limit_ = boost::lexical_cast<double>(lower_char);
        joint.upper_limit_ = boost::lexical_cast<double>(upper_char);
    }
    catch (boost::bad_lexical_cast &e)
    {
        ROS_ERROR("Joint limit (%s, %s) is not a valid float", lower_char, upper_char);
        return false;
    }

    return true;
}

bool CollisionModel::parseJoint(Joint &joint, TiXmlElement* o)
{
    joint.clear();
    const char *name_char = o->Attribute("name");
    if (!name_char)
    {
        ROS_ERROR("No name given for the joint.");
        return false;
    }
    joint.name_ = std::string(name_char);

    // joint limit is optional
    TiXmlElement *lim = o->FirstChildElement("limit");
    if (lim) {
        if (!parseLimit(joint, lim))
            return false;
    }
    return true;
}

bool CollisionModel::parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c)
{
    const char* link1_str = c->Attribute("link1");
    const char* link2_str = c->Attribute("link2");
    if (link1_str != NULL && link2_str != NULL)
    {
        link1 = link1_str;
        link2 = link2_str;
        return true;
    }
    ROS_ERROR("disable_collisions has wrong attributes");

    return false;
}

int CollisionModel::parseSRDF(const std::string &xml_string)
{
    int result_id = disabled_collisions.size();
    disabled_collisions.push_back( CollisionPairs() );
    CollisionPairs& dis_col = disabled_collisions[result_id];

    TiXmlDocument xml_doc;
    xml_doc.Parse(xml_string.c_str());
    if (xml_doc.Error())
    {
        ROS_ERROR("%s", xml_doc.ErrorDesc());
        xml_doc.ClearError();
        throw std::logic_error("Wrong xml document");
        //return -1;
    }

    TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        ROS_ERROR("Could not find the 'robot' element in the xml file");
        throw std::logic_error("Could not find the 'robot' element in the xml file");
        //return -1;
    }
    // Get robot name
    const char *name = robot_xml->Attribute("name");
    if (!name)
    {
        ROS_ERROR("No name given for the robot.");
        throw std::logic_error("No name given for the robot.");
        //return -1;
    }

    if (name_ != std::string(name))
    {
        ROS_ERROR("Name from SRDF: %s differ from %s", name, name_.c_str());
        throw std::logic_error("Name from SRDF differs from URDF");
        //return -1;
    }

    // Get all disable_collisions elements
    for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions");
            disable_collision_xml;
            disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
    {
        std::string link1, link2;
        try {
            parseDisableCollision(link1, link2, disable_collision_xml);
            int link1_id = getLinkIndex(link1);
            int link2_id = getLinkIndex(link2);
            if (link1_id == -1)
            {
                ROS_INFO("Ignoring collision pair: ('%s', '%s'), because link '%s' does not exist.",
                                                link1.c_str(), link2.c_str(), link1.c_str());
                continue;
            }
            if (link2_id == -1)
            {
                ROS_INFO("Ignoring collision pair: ('%s', '%s'), because link '%s' does not exist.",
                                                link1.c_str(), link2.c_str(), link2.c_str());
                continue;
            }
            for (int idx = 0; idx < dis_col.size(); idx++) {
                if ( (dis_col[idx].first == link1_id && dis_col[idx].second == link2_id) ||
                        (dis_col[idx].first == link2_id && dis_col[idx].second == link1_id) ) {
                    ROS_WARN("disabled collision pair is repeated:  %s  %s", link1.c_str(), link2.c_str());
                }
            }

            dis_col.push_back(std::pair<int, int>(link1_id, link2_id));
        }
        catch (urdf::ParseError &e) {
            ROS_ERROR("disable_collisions xml is not initialized correctly");
        throw std::logic_error("disable_collisions xml is not initialized correctly");
            //return -1;
        }
    }
    return result_id;
}

boost::shared_ptr<CollisionModel> CollisionModel::parseURDF(const std::string &xml_string)
{
    boost::shared_ptr<CollisionModel> model(new CollisionModel);
    model->root_index_ = -1;
    TiXmlDocument xml_doc;
    xml_doc.Parse(xml_string.c_str());
    if (xml_doc.Error())
    {
        ROS_ERROR("%s", xml_doc.ErrorDesc());
        xml_doc.ClearError();
        model.reset();
        return model;
    }
    TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        ROS_ERROR("Could not find the 'robot' element in the xml file");
        model.reset();
        return model;
    }
    // Get robot name
    const char *name = robot_xml->Attribute("name");
    if (!name)
    {
        ROS_ERROR("No name given for the robot.");
        model.reset();
        return model;
    }
    model->name_ = std::string(name);
    int link_index=0;
    // Get all Link elements
    for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"), link_index++)
    {
        boost::shared_ptr<Link> plink(new Link());
        model->links_.push_back(  plink );
        plink->index_ = link_index;
        try {
            parseLink(*plink, link_xml);
            model->link_name_idx_map_[plink->name] = plink->index_;
        }
        catch (urdf::ParseError &e) {
            ROS_ERROR("link xml is not initialized correctly");
            model.reset();
            return model;
        }
    }

    model->link_count_ = model->links_.size();

    for (int l_i = 0; l_i < model->link_count_; l_i++)
    {
        for (Link::VecPtrCollision::iterator c_it = model->links_[l_i]->collision_array.begin(); c_it != model->links_[l_i]->collision_array.end(); c_it++)
        {
            if ((*c_it)->geometry->getType() == Geometry::CONVEX)
            {
                Convex *conv = static_cast<Convex*>( (*c_it)->geometry.get() );
                conv->points_id_.clear();
                for (Convex::ConvexPointsStrVector::iterator p_it = conv->points_str_.begin(); p_it != conv->points_str_.end(); p_it++)
                {
                    int id = model->getLinkIndex(p_it->first);
                    if (id == -1)
                    {
                        ROS_ERROR("parseURDF: could not find link %s", p_it->first.c_str());
                        break;
                    }
                    conv->points_id_.push_back(std::pair<int, KDL::Vector>(id, p_it->second));
                }
            }
        }
    }

    // Get all Joint elements
    for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
    {
        try {
            Joint joint;
            parseJoint(joint, joint_xml);
            model->joints_.push_back(joint);
        }
        catch (urdf::ParseError &e) {
            ROS_ERROR("link xml is not initialized correctly");
            model.reset();
            return model;
        }
    }

    return model;
}

bool CollisionModel::convertSelfCollisionsInURDF(const std::string &xml_in, std::string &xml_out)
{
    TiXmlDocument xml_doc;
    xml_doc.Parse(xml_in.c_str());
    if (xml_doc.Error())
    {
        ROS_ERROR("%s", xml_doc.ErrorDesc());
        xml_doc.ClearError();
        return false;
    }
    TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
    if (!robot_xml)
    {
        ROS_ERROR("Could not find the 'robot' element in the xml file");
        return false;
    }
    int link_index=0;
    // Get all Link elements
    for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"), link_index++)
    {
        Link link;
        if (!parseLink(link, link_xml)) {
            ROS_ERROR("Could not parse link.");
            return false;
        }

        bool first_found = false;
        for (int i = 0; i < link.collision_array.size(); ++i) {
            boost::shared_ptr<Sphere > sp = boost::dynamic_pointer_cast<Sphere >( link.collision_array[i]->geometry );
            boost::shared_ptr<Capsule > cap = boost::dynamic_pointer_cast<Capsule >( link.collision_array[i]->geometry );
            if (sp || cap) {
                if (!first_found) {
                    first_found = true;
                    // remove all <collision> nodes
                    while (TiXmlNode* collision_xml = link_xml->LastChild("collision")) {
                        link_xml->RemoveChild(collision_xml);
                    }
                }
                if (sp) {
                    TiXmlElement collision("collision");
                    std::ostringstream xyz_val, rpy_val, radius_val;
                    xyz_val << link.collision_array[i]->origin.p.x() << " " << link.collision_array[i]->origin.p.y() << " " << link.collision_array[i]->origin.p.z();
                    double roll, pitch, yaw;
                    link.collision_array[i]->origin.M.GetRPY(roll, pitch, yaw);
                    rpy_val << roll << " " << pitch << " " << yaw;
                    TiXmlElement origin("origin");
                    origin.SetAttribute("xyz", xyz_val.str().c_str());
                    origin.SetAttribute("rpy", rpy_val.str().c_str());
                    collision.InsertEndChild(origin);

                    radius_val << sp->getRadius();
                    TiXmlElement geometry("geometry");
                    TiXmlElement sphere("sphere");
                    sphere.SetAttribute("radius", radius_val.str().c_str());
                    geometry.InsertEndChild(sphere);
                    collision.InsertEndChild(geometry);
                    link_xml->InsertEndChild(collision);
                }
                else if (cap) {
                    std::ostringstream xyz_val, rpy_val, radius_val, length_val;
                    double roll, pitch, yaw;
                    link.collision_array[i]->origin.M.GetRPY(roll, pitch, yaw);
                    rpy_val << roll << " " << pitch << " " << yaw;

                    TiXmlElement collision1("collision");
                    KDL::Frame fr1 = link.collision_array[i]->origin * KDL::Frame(KDL::Vector(0,0,cap->getLength()/2));
                    xyz_val << fr1.p.x() << " " << fr1.p.y() << " " << fr1.p.z();
                    TiXmlElement origin1("origin");
                    origin1.SetAttribute("xyz", xyz_val.str().c_str());
                    origin1.SetAttribute("rpy", rpy_val.str().c_str());
                    collision1.InsertEndChild(origin1);

                    length_val << cap->getLength();
                    radius_val << cap->getRadius();
                    TiXmlElement geometry1("geometry");
                    TiXmlElement sphere1("sphere");
                    sphere1.SetAttribute("radius", radius_val.str().c_str());
                    geometry1.InsertEndChild(sphere1);
                    collision1.InsertEndChild(geometry1);
                    link_xml->InsertEndChild(collision1);

                    TiXmlElement collision2("collision");
                    KDL::Frame fr2 = link.collision_array[i]->origin * KDL::Frame(KDL::Vector(0,0,-cap->getLength()/2));
                    xyz_val.str("");
                    xyz_val << fr2.p.x() << " " << fr2.p.y() << " " << fr2.p.z();
                    TiXmlElement origin2("origin");
                    origin2.SetAttribute("xyz", xyz_val.str().c_str());
                    origin2.SetAttribute("rpy", rpy_val.str().c_str());
                    collision2.InsertEndChild(origin2);

                    TiXmlElement geometry2("geometry");
                    TiXmlElement sphere2("sphere");
                    sphere2.SetAttribute("radius", radius_val.str().c_str());
                    geometry2.InsertEndChild(sphere2);
                    collision2.InsertEndChild(geometry2);
                    link_xml->InsertEndChild(collision2);

                    TiXmlElement collision3("collision");
                    KDL::Frame fr3 = link.collision_array[i]->origin;
                    xyz_val.str("");
                    xyz_val << fr3.p.x() << " " << fr2.p.y() << " " << fr2.p.z();
                    TiXmlElement origin3("origin");
                    origin3.SetAttribute("xyz", xyz_val.str().c_str());
                    origin3.SetAttribute("rpy", rpy_val.str().c_str());
                    collision3.InsertEndChild(origin3);

                    TiXmlElement geometry3("geometry");
                    TiXmlElement cylinder("cylinder");
                    cylinder.SetAttribute("radius", radius_val.str().c_str());
                    cylinder.SetAttribute("length", length_val.str().c_str());
                    geometry3.InsertEndChild(cylinder);
                    collision3.InsertEndChild(geometry3);
                    link_xml->InsertEndChild(collision3);
                }
            }
        }
    }

    std::ostringstream stream_out;
    stream_out << xml_doc;
    xml_out = stream_out.str();
    return true;
}

void CollisionModel::generateCollisionPairs()
{
    enabled_collisions.clear();
    for (int srdf_id = 0; srdf_id < disabled_collisions.size(); ++srdf_id) {
        enabled_collisions.push_back( CollisionPairs() );
        CollisionPairs& dis_col = disabled_collisions[srdf_id];
        CollisionPairs& en_col = enabled_collisions[srdf_id];
        //ROS_INFO("%ld", links_.size());

        for (int l_i = 0; l_i < link_count_; l_i++)
        {
            if (links_[l_i]->collision_array.size() == 0)
            {
                continue;
            }
            for (int l_j = 0; l_j < link_count_; l_j++)
            {
                if (links_[l_j]->collision_array.size() == 0)
                {
                    continue;
                }
                if (l_i == l_j)
                {
                    continue;
                }
                bool add = true;
                for (CollisionPairs::iterator dc_it = dis_col.begin(); dc_it != dis_col.end(); dc_it++)
                {
                    if (    (dc_it->first == l_i && dc_it->second == l_j) ||
                        (dc_it->second == l_i && dc_it->first == l_j) )
                    {
                        add = false;
                        break;
                    }
                }
                if (add)
                {
                    for (CollisionPairs::iterator ec_it = en_col.begin(); ec_it != en_col.end(); ec_it++)
                    {
                        if ((ec_it->first == l_i && ec_it->second == l_j) ||
                            (ec_it->second == l_i && ec_it->first == l_j) )
                        {
                            add = false;
                            break;
                        }
                    }

                    if (add)
                    {
                        en_col.push_back(std::pair<int, int>(l_i, l_j));
                    }
                }
            }
        }
    }
}

bool CollisionModel::checkRayCollision(const Geometry *geom, const KDL::Frame &tf, const KDL::Vector &ray_start, const KDL::Vector &ray_end) {
//    if (geom->getType() == Geometry::CAPSULE) {
        KDL::Vector nz = ray_end-ray_start;
        Capsule ray(0.04, nz.Norm());
        nz.Normalize();
        KDL::Vector nx,ny;
        if (fabs(nz.z()) < 0.7) {
            nx = KDL::Vector(0,0,1);
        }
        else {
            nx = KDL::Vector(0,1,0);
        }
        ny = nz*nx;
        nx = ny*nz;
        ny.Normalize();
        nx.Normalize();
        KDL::Frame tf2(KDL::Rotation(nx,ny,nz), (ray_end+ray_start)/2);
        KDL::Vector d1_out, d2_out, n1_out, n2_out;
        double distance;
        if (!getDistance(geom, tf, &ray, tf2, d1_out, d2_out, n1_out, n2_out, 0.0, distance) || distance < 0.0) {
            return true;
        }

//    }
//    else if (geom->getType() == Geometry::SPHERE) {
//    }

    return false;
}

bool CollisionModel::getDistance(const Geometry *geom1, const KDL::Frame &tf1, const Geometry *geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, KDL::Vector &n1_out, KDL::Vector &n2_out, double d0, double &distance)
{
    // check broadphase distance
    if ((tf1.p - tf2.p).Norm() > geom1->getBroadphaseRadius() + geom2->getBroadphaseRadius() + d0)
    {
        distance = d0 * 2.0;
        return true;
    }
    if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::CAPSULE)
    {
        const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());
        const fcl_2::Capsule *ob2 = static_cast<fcl_2::Capsule* >(geom2->shape.get());

        // capsules are shifted by length/2
        double x1,y1,z1,w1, x2, y2, z2, w2;
        KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
        KDL::Frame tf2_corrected = tf2 * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
        tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
             &distance, &p1, &p2, &n1, &n2);
        // the output for two capsules is in global coordinates
        d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
        d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
        n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
        n2_out = KDL::Vector(n2[0], n2[1], n2[2]);
        return result;
    }
    else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::CONVEX)
    {
//        std::cout << "getDistance: CAPSULE CONVEX" << std::endl;
        // TODO: check if it works
        const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());
        const fcl_2::Convex *ob2 = static_cast<fcl_2::Convex* >(geom2->shape.get());

        const Convex *conv2 = static_cast<const Convex* >(geom2);

        // capsules are shifted by length/2
        double x1,y1,z1,w1, x2, y2, z2, w2;
        KDL::Frame tf1_corrected = tf1;
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
        tf2.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
             &distance, &p1, &p2, &n1, &n2);

        if (result) {
            // the output for two capsules is in wtf coordinates
            d1_out = tf1_corrected*KDL::Vector(p1[0], p1[1], p1[2]);
            d2_out = tf1_corrected*((tf1_corrected.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
        }
        else {
            d1_out = tf1_corrected.p;
            d2_out = tf2.p;
            result = true;
            distance = 0.0;
        }
        n1_out = d2_out - d1_out;
        n1_out.Normalize();
        n2_out = d1_out - d2_out;
        n2_out.Normalize();

        return result;
    }
    else if (geom1->getType() == Geometry::CONVEX && geom2->getType() == Geometry::CAPSULE)
    {
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
    }
    else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::CONVEX)
    {
//        std::cout << "getDistance: SPHERE CONVEX" << std::endl;

        // TODO: check if it works
        const fcl_2::Sphere *ob1 = static_cast<fcl_2::Sphere* >(geom1->shape.get());
        const fcl_2::Convex *ob2 = static_cast<fcl_2::Convex* >(geom2->shape.get());

        const Convex *conv2 = static_cast<const Convex* >(geom2);

        // capsules are shifted by length/2
        double x1,y1,z1,w1, x2, y2, z2, w2;
        KDL::Frame tf1_corrected = tf1;// * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
        tf2.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
             &distance, &p1, &p2, &n1, &n2);
        if (result) {
            // the output for two capsules is in wtf coordinates
            d1_out = tf1_corrected*KDL::Vector(p1[0], p1[1], p1[2]);
            d2_out = tf1_corrected*((tf1_corrected.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
        }
        else {
            d1_out = tf1_corrected.p;
            d2_out = tf2.p;
            result = true;
            distance = 0.0;
        }
        n2_out = d1_out - d2_out;
        n2_out.Normalize();
        n1_out = d2_out - d1_out;
        n1_out.Normalize();
        return result;
    }
    else if (geom1->getType() == Geometry::CONVEX && geom2->getType() == Geometry::SPHERE)
    {
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
    }
    else if (geom1->getType() == Geometry::CONVEX && geom2->getType() == Geometry::CONVEX)
    {
//        std::cout << "getDistance: CONVEX CONVEX" << std::endl;
        // TODO: check if it works
        const fcl_2::Convex *ob1 = static_cast<fcl_2::Convex* >(geom1->shape.get());
        const fcl_2::Convex *ob2 = static_cast<fcl_2::Convex* >(geom2->shape.get());

        const Convex *conv1 = static_cast<const Convex* >(geom1);
        const Convex *conv2 = static_cast<const Convex* >(geom2);

        // calculate narrowphase distance
        if ((tf1 * conv1->center_ - tf2 * conv2->center_).Norm() > conv1->radius_ + conv2->radius_ + d0)
        {
            distance = d0 * 2.0;
            return true;
        }

        double x1,y1,z1,w1, x2, y2, z2, w2;
        tf1.M.GetQuaternion(x1,y1,z1,w1);
        tf2.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
             &distance, &p1, &p2, &n1, &n2);

        if (result) {
            // the output for two capsules is in wtf coordinates
            d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
            d2_out = tf1*((tf1.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
        }
        else {
            d1_out = tf1.p;
            d2_out = tf2.p;
            result = true;
            distance = 0.0;
        }

        n2_out = d1_out - d2_out;
        n2_out.Normalize();
        n1_out = d2_out - d1_out;
        n1_out.Normalize();
        return result;
    }
    else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::SPHERE)
    {
        const fcl_2::Sphere *ob1 = static_cast<fcl_2::Sphere* >(geom1->shape.get());
        const fcl_2::Sphere *ob2 = static_cast<fcl_2::Sphere* >(geom2->shape.get());

        double x1,y1,z1,w1, x2, y2, z2, w2;
        KDL::Frame tf1_corrected = tf1;
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
        KDL::Frame tf2_corrected = tf2;
        tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
             &distance, &p1, &p2, &n1, &n2);
        // the output for two spheres is in global coordinates
        d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
        d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
        n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
        n2_out = KDL::Vector(n2[0], n2[1], n2[2]);
        return result;
    }
    else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::SPHERE)
    {
        const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());
        const fcl_2::Sphere *ob2 = static_cast<fcl_2::Sphere* >(geom2->shape.get());

        // capsules are shifted by length/2
        double x1,y1,z1,w1, x2, y2, z2, w2;
        KDL::Frame tf1_corrected = tf1;
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
        KDL::Frame tf2_corrected = tf2;
        tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

        // output variables
        fcl_2::Vec3f p1;
        fcl_2::Vec3f p2;
        fcl_2::Vec3f n1;
        fcl_2::Vec3f n2;
        bool result = gjk_solver.shapeDistance(
            *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
            *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
             &distance, &p2, &p1, &n2, &n1);

        // the output is in objects coordinate frames
        d1_out = tf1_corrected * KDL::Vector(p1[0], p1[1], p1[2]);
        d2_out = tf2_corrected * KDL::Vector(p2[0], p2[1], p2[2]);
        n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
        n2_out = KDL::Vector(n2[0], n2[1], n2[2]);

        return result;
    }
    else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::CAPSULE)
    {
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
    }
    else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::OCTOMAP)
    {
        const Sphere *sp = static_cast<const Sphere*>(geom1);
        const Octomap *om = static_cast<const Octomap*>(geom2);
        const boost::shared_ptr<octomap::OcTree > &oc = om->getOctomap();

        const KDL::Frame &T_W_S = tf1;
        const KDL::Frame &T_W_O = tf2;

        KDL::Frame T_O_S = T_W_O.Inverse() * T_W_S;
        KDL::Vector p = T_O_S.p;
        double r = sp->getRadius() + oc->getResolution() + d0 * 2.0;
        octomath::Vector3 pmin(p.x() - r, p.y() - r, p.z() - r), pmax(p.x() + r, p.y() + r, p.z() + r);
        double min_dist = r;
        KDL::Vector min_pt;
        for (octomap::OcTree::leaf_bbx_iterator it = oc->begin_leafs_bbx(pmin, pmax); it != oc->end_leafs_bbx(); it++) {
            if(it->getOccupancy() <= oc->getOccupancyThres()) {
                continue;
            }
            KDL::Vector pt(it.getX(), it.getY(), it.getZ());
            double dist = (p - pt).Norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_pt = pt;
            }
        }

        if (min_dist >= sp->getRadius() + oc->getResolution() + d0) {
            distance = 2.0 * d0;
            return true;
        }
        KDL::Vector pt_sp_W = T_W_S.p;
        KDL::Vector pt_oc_W = T_W_O * min_pt;
        KDL::Vector n(pt_oc_W - pt_sp_W);
        n.Normalize();
        d1_out = pt_sp_W + n * sp->getRadius();
        d2_out = pt_oc_W - n * oc->getResolution();
        n1_out = n;
        n2_out = -n;
        distance = min_dist - sp->getRadius() - oc->getResolution();
        return true;
    }
    else if (geom1->getType() == Geometry::OCTOMAP && geom2->getType() == Geometry::SPHERE)
    {
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
    }
    else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::OCTOMAP)
    {
        const Capsule *ca = static_cast<const Capsule*>(geom1);
        const Octomap *om = static_cast<const Octomap*>(geom2);
        const boost::shared_ptr<octomap::OcTree > &oc = om->getOctomap();

        const KDL::Frame &T_W_C = tf1;
        const KDL::Frame &T_W_O = tf2;
        KDL::Frame T_O_C = T_W_O.Inverse() * T_W_C;

        const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());

        // capsules are shifted by length/2
        double x1,y1,z1,w1;
        KDL::Frame tf1_corrected = T_O_C;
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);

        double r = ca->getRadius() + oc->getResolution() + d0 * 2.0;

        // create fake sphere for all octomap leafs
//        boost::shared_ptr<fcl_2::ShapeBase> shape_sp( static_cast<fcl_2::ShapeBase*>(new fcl_2::Sphere(oc->getResolution())) );
        fcl_2::Sphere shape_sp(oc->getResolution());

        KDL::Vector e1_O = T_O_C * KDL::Vector(0, 0, ca->getLength());
        KDL::Vector e2_O = T_O_C * KDL::Vector(0, 0, -ca->getLength());
        octomath::Vector3 pmin(std::min(e1_O.x(), e2_O.x()) - r, std::min(e1_O.y(), e2_O.y()) - r, std::min(e1_O.z(), e2_O.z()) - r);
        octomath::Vector3 pmax(std::max(e1_O.x(), e2_O.x()) + r, std::max(e1_O.y(), e2_O.y()) + r, std::max(e1_O.z(), e2_O.z()) + r);
        double min_dist = r;
        // output variables
        fcl_2::Vec3f min_p1, min_p2, min_n1, min_n2;
        KDL::Frame min_tf2_corrected;
        KDL::Vector min_pt_ca;
        KDL::Vector min_pt_oc;

        for (octomap::OcTree::leaf_bbx_iterator it = oc->begin_leafs_bbx(pmin, pmax); it != oc->end_leafs_bbx(); it++) {
            KDL::Vector pt_O(it.getX(), it.getY(), it.getZ());
            KDL::Frame tf2_corrected(pt_O);
            double x2, y2, z2, w2;
            tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

            // output variables
            fcl_2::Vec3f p1, p2, n1, n2;
            double dist;
            bool result = gjk_solver.shapeDistance(
                shape_sp, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
                *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
                 &dist, &p2, &p1, &n2, &n1);

            if (dist < min_dist) {
                min_dist = dist;
                min_p1 = p1;
                min_p2 = p2;
                min_n1 = n1;
                min_n2 = n2;
                min_tf2_corrected = tf2_corrected;
            }
        }

        // the output is in objects coordinate frames
        d1_out = T_W_O * tf1_corrected * KDL::Vector(min_p1[0], min_p1[1], min_p1[2]);
        d2_out = T_W_O * min_tf2_corrected * KDL::Vector(min_p2[0], min_p2[1], min_p2[2]);
        n1_out = KDL::Frame(T_W_O.M) * KDL::Vector(min_n1[0], min_n1[1], min_n1[2]);
        n2_out = KDL::Frame(T_W_O.M) * KDL::Vector(min_n2[0], min_n2[1], min_n2[2]);
        distance = min_dist;

        return true;
    }
    else if (geom1->getType() == Geometry::OCTOMAP && geom2->getType() == Geometry::CAPSULE)
    {
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
    }
    else
    {
        ROS_ERROR("not supported distance measure");
    }
    return false;
}

bool CollisionModel::addLink(const std::string &name, const std::string &parent_name, const std::vector< boost::shared_ptr< Collision > > &col_array) {
    if (name.empty()) {
        return false;
    }

    for (VecPtrLink::const_iterator it = links_.begin(); it != links_.end(); it++) {
        if ((*it)->name == name) {
            return false;
        }
    }

    boost::shared_ptr< Link > plink(new Link());
    plink->name = name;
    plink->collision_array = col_array;
    plink->index_ = links_.size();
    plink->parent_index_ = getLinkIndex(parent_name);
    for (Link::VecPtrCollision::iterator it = plink->collision_array.begin(); it != plink->collision_array.end(); it++) {
        (*it)->parent_link_idx_ = plink->index_;
    }
    links_.push_back(plink);
    link_name_idx_map_[plink->name] = plink->index_;

    link_count_ = links_.size();

    return true;
}

bool CollisionModel::addCollisionToLink(const std::string &link_name, const boost::shared_ptr< Collision > &pcol, const KDL::Frame &T_L_C) {
    int lidx = getLinkIndex(link_name);
    if (lidx < 0) {
        std::cout << "ERROR: CollisionModel::addCollisionToLink: could not find link " << link_name << std::endl;
        return false;
    }

    links_[lidx]->collision_array.push_back( pcol );
    return true;
}

bool CollisionModel::removeCollisionFromLink(const std::string &link_name, const boost::shared_ptr< Collision > &pcol) {
    int lidx = getLinkIndex(link_name);
    if (lidx < 0) {
        std::cout << "ERROR: CollisionModel::removeCollisionFromLink: could not find link " << link_name << std::endl;
        return false;
    }

    bool after = false;
    for (int cidx = 0; cidx < links_[lidx]->collision_array.size(); cidx++) {
        if (after) {
            links_[lidx]->collision_array[cidx-1] = links_[lidx]->collision_array[cidx];
        }
        else if ( links_[lidx]->collision_array[cidx].get() == pcol.get() ) {
            after = true;
        }
    }
    if (!after) {
        std::cout << "ERROR: CollisionModel::removeCollisionFromLink: could not find collision object" << std::endl;
        return false;
    }
    links_[lidx]->collision_array.resize(links_[lidx]->collision_array.size() - 1);
    return true;
}

const CollisionModel::VecPtrLink &CollisionModel::getLinks() const {
    return links_;
}

int CollisionModel::getLinksCount() const {
    return link_count_;
}

const Link::VecPtrCollision &CollisionModel::getLinkCollisionArray(int idx) const {
    return links_[idx]->collision_array;
}

bool compareCollisionInfoDist(const CollisionInfo &i1, const CollisionInfo &i2) {
    if (i1.dist < i2.dist) {
        return true;
    }
    return false;
}

boost::shared_ptr< self_collision::Collision > createCollisionCapsule(double radius, double length, const KDL::Frame &origin) {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Capsule(radius, length));

        pcol->origin = origin;
        return pcol;
}

boost::shared_ptr< self_collision::Collision > createCollisionSphere(double radius, const KDL::Frame &origin) {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Sphere(radius));
        pcol->origin = origin;
        return pcol;
}

boost::shared_ptr< self_collision::Collision > createCollisionConvex(const std::vector<KDL::Vector > &vertices, const std::vector<int> &polygons, const KDL::Frame &origin, const std::string &visualisation_hint) {
    int num_planes = 0;
    int next_plane_idx = 0;
    for (int i=0; i<polygons.size(); i++) {
        if (i == next_plane_idx) {
            next_plane_idx += polygons[i] + 1;
            num_planes++;
        }
    }

    boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
    pcol->geometry.reset(new self_collision::Convex());
    boost::shared_ptr<self_collision::Convex > conv = boost::static_pointer_cast<self_collision::Convex >(pcol->geometry);
    conv->updateConvex(vertices.size(), vertices, num_planes, polygons);
    conv->visualisation_hint_ = visualisation_hint;
    pcol->origin = origin;
    return pcol;
}

boost::shared_ptr< self_collision::Collision > createCollisionOctomap(const boost::shared_ptr<octomap::OcTree > &om, const KDL::Frame &origin) {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Octomap(om));
        pcol->origin = origin;
        return pcol;
}

//*
void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model,
                    int srdf_id, const std::vector<KDL::Frame > &links_fk, double activation_dist,
                                    std::vector<self_collision::CollisionInfo> &link_collisions) {
        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it =
                                    col_model->enabled_collisions[srdf_id].begin();
                                        it != col_model->enabled_collisions[srdf_id].end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
                    KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;

                    // TODO: handle dist < 0
                    if (!self_collision::CollisionModel::getDistance((*col1)->geometry.get(), T_B_C1, (*col2)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, activation_dist, dist)) {
                        std::cout << "ERROR: getCollisionPairs: dist < 0" << std::endl;
                    }

                    if (dist < activation_dist) {
                        self_collision::CollisionInfo col_info;
                        col_info.link1_idx = link1_idx;
                        col_info.link2_idx = link2_idx;
                        col_info.dist = dist;
                        col_info.n1_B = n1_B;
                        col_info.n2_B = n2_B;
                        col_info.p1_B = p1_B;
                        col_info.p2_B = p2_B;
                        link_collisions.push_back(col_info);
                    }

                }
            }
        }
}

void getCollisionPairsNoAlloc(const boost::shared_ptr<self_collision::CollisionModel> &col_model,
                    int srdf_id, const std::vector<KDL::Frame > &links_fk, double activation_dist,
                                    std::vector<self_collision::CollisionInfo> &link_collisions) {
        const int N = link_collisions.size();
        if (N == 0) {
            std::cout << "ERROR: getCollisionPairsNoAlloc: link_collisions.size() == 0" << std::endl;
            return;
        }

        for (int i = 0; i < N; ++i) {
            link_collisions[i].link1_idx = -1;
        }

        int col_count = 0;

        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it =
                                    col_model->enabled_collisions[srdf_id].begin();
                                        it != col_model->enabled_collisions[srdf_id].end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
                    KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;

                    // TODO: handle dist < 0
                    if (!self_collision::CollisionModel::getDistance((*col1)->geometry.get(), T_B_C1, (*col2)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, activation_dist, dist)) {
                        std::cout << "ERROR: getCollisionPairs: dist < 0" << std::endl;
                    }

                    if (dist < activation_dist) {
                        self_collision::CollisionInfo col_info;
                        col_info.link1_idx = link1_idx;
                        col_info.link2_idx = link2_idx;
                        col_info.dist = dist;
                        col_info.n1_B = n1_B;
                        col_info.n2_B = n2_B;
                        col_info.p1_B = p1_B;
                        col_info.p2_B = p2_B;

                        for (int i = 0; i < N; ++i) {
                            if (link_collisions[i].link1_idx == -1) {
                                link_collisions[i] = col_info;
                                ++col_count;
                                break;
                            }
                            else if (link_collisions[i].dist > col_info.dist) {
                                int max_idx = ((col_count > N)?N:col_count);
                                for (int j = max_idx-1; j > i; --j) {
                                    link_collisions[j] = link_collisions[j-1];
                                }
                                link_collisions[i] = col_info;
                                ++col_count;
                                break;
                            }
                        }
                    }
                }
            }
        }
}

/*/
void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                            double activation_dist, std::vector<self_collision::CollisionInfo> &link_collisions) {
        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            double sum_depth = 0.0;
            bool added = false;
            self_collision::CollisionInfo col_info;
            col_info.link1_idx = link1_idx;
            col_info.link2_idx = link2_idx;
            col_info.dist = 0.0;
            col_info.n1_B = KDL::Vector();
            col_info.n2_B = KDL::Vector();
            col_info.p1_B = KDL::Vector();
            col_info.p2_B = KDL::Vector();
            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
                    KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;

                    // TODO: handle dist < 0
                    if (!self_collision::CollisionModel::getDistance((*col1)->geometry, T_B_C1, (*col2)->geometry, T_B_C2, p1_B, p2_B, n1_B, n2_B, activation_dist, dist)) {
                        std::cout << "ERROR: getCollisionPairs: dist < 0" << std::endl;
                    }

                    if (dist < activation_dist) {
                        added = true;
                        double depth = activation_dist - dist;
                        sum_depth += depth;
                        col_info.dist += dist * depth;
                        col_info.n1_B += n1_B * depth;
                        col_info.n2_B += n2_B * depth;
                        col_info.p1_B += p1_B * depth;
                        col_info.p2_B += p2_B * depth;
                    }
                }
            }
            if (added) {
                col_info.dist /= sum_depth;
                col_info.n1_B = col_info.n1_B / sum_depth;
                col_info.n2_B = col_info.n2_B / sum_depth;
                col_info.p1_B = col_info.p1_B / sum_depth;
                col_info.p2_B = col_info.p2_B / sum_depth;
                if (col_info.n1_B.Norm() > 0.0001) {
                    col_info.n1_B.Normalize();
                    col_info.n2_B.Normalize();
                    link_collisions.push_back(col_info);
                }
            }
        }
}
//*/
bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol, const KDL::Frame &T_B_L1, const std::vector<KDL::Frame > &links_fk, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::set<int> &excluded_link_idx) {
        // check collision with base and environment only
        std::vector<int > static_links_idx;
        static_links_idx.push_back( col_model->getLinkIndex("base") );
        static_links_idx.push_back( col_model->getLinkIndex("env_link") );

        for (std::vector<int >::const_iterator li_it = static_links_idx.begin(); li_it != static_links_idx.end(); li_it++) {
            int link2_idx = *li_it;
            if (excluded_link_idx.find(link2_idx) != excluded_link_idx.end()) {
                continue;
            }
            const KDL::Frame &T_B_L2 = links_fk[link2_idx];
            for (self_collision::Link::VecPtrCollision::const_iterator col2_it = col_model->getLinkCollisionArray(link2_idx).begin(); col2_it != col_model->getLinkCollisionArray(link2_idx).end(); col2_it++) {
                double dist = 0.0;
                KDL::Vector p1_B, p2_B, n1_B, n2_B;
                KDL::Frame T_B_C1 = T_B_L1 * pcol->origin;
                KDL::Frame T_B_C2 = T_B_L2 * (*col2_it)->origin;

                self_collision::CollisionModel::getDistance(pcol->geometry.get(), T_B_C1, (*col2_it)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, 0.01, dist);
                if (dist < 0.001) {
                    return true;
                }
            }
        }

        return false;
}

bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Collision > &pcol2, const KDL::Frame &T_B_L2) {
        double dist = 0.0;
        KDL::Vector p1_B, p2_B, n1_B, n2_B;
        KDL::Frame T_B_C1 = T_B_L1 * pcol1->origin;
        KDL::Frame T_B_C2 = T_B_L2 * pcol2->origin;

        self_collision::CollisionModel::getDistance(pcol1->geometry.get(), T_B_C1, pcol2->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, 0.01, dist);
        if (dist < 0.001) {
            return true;
        }

        return false;
}

bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Link > &link2, const KDL::Frame &T_B_L2) {
        double dist = 0.0;
        KDL::Vector p1_B, p2_B, n1_B, n2_B;
        KDL::Frame T_B_C1 = T_B_L1 * pcol1->origin;

        for (self_collision::Link::VecPtrCollision::const_iterator col2_it = link2->collision_array.begin(); col2_it != link2->collision_array.end(); col2_it++) {
            KDL::Frame T_B_C2 = T_B_L2 * (*col2_it)->origin;
            self_collision::CollisionModel::getDistance(pcol1->geometry.get(), T_B_C1, (*col2_it)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, 0.01, dist);
//            std::cout << "dist " << dist << std::endl;
            if (dist < 0.001) {
                return true;
            }
        }

        return false;
}

bool checkCollision(const boost::shared_ptr< self_collision::Link > &link1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Link > &link2, const KDL::Frame &T_B_L2, double *min_dist) {
        double dist = 0.0;
        if (min_dist != NULL) {
            *min_dist = -1.0;
        }

        KDL::Vector p1_B, p2_B, n1_B, n2_B;
        for (self_collision::Link::VecPtrCollision::const_iterator col1_it = link1->collision_array.begin(); col1_it != link1->collision_array.end(); col1_it++) {
            KDL::Frame T_B_C1 = T_B_L1 * (*col1_it)->origin;
            for (self_collision::Link::VecPtrCollision::const_iterator col2_it = link2->collision_array.begin(); col2_it != link2->collision_array.end(); col2_it++) {
                KDL::Frame T_B_C2 = T_B_L2 * (*col2_it)->origin;
                self_collision::CollisionModel::getDistance((*col1_it)->geometry.get(), T_B_C1, (*col2_it)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, 0.01, dist);
                if (min_dist != NULL) {
                    if ( (*min_dist) < 0 || (*min_dist) > dist) {
                        (*min_dist) = dist;
                    }
                }

                if (dist < 0.001) {
                    if (min_dist != NULL) {
                        *min_dist = 0.0;
                    }
                    return true;
                }
            }
        }
        return false;
}

bool checkCollision(const boost::shared_ptr<self_collision::CollisionModel> &col_model, int srdf_id,
        const std::vector<KDL::Frame > &links_fk, const std::set<int> &excluded_link_idx) {
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it =
                                    col_model->enabled_collisions[srdf_id].begin();
                                        it != col_model->enabled_collisions[srdf_id].end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;

            if (excluded_link_idx.find(link1_idx) != excluded_link_idx.end() || excluded_link_idx.find(link2_idx) != excluded_link_idx.end()) {
                continue;
            }
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
                    KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;
                    self_collision::CollisionModel::getDistance((*col1)->geometry.get(), T_B_C1, (*col2)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, 0.01, dist);
                    if (dist < 0.001) {
                        return true;
                    }
                }
            }
        }
        return false;
}

int removeNodesFromOctomap(boost::shared_ptr<octomap::OcTree > &oc, const Geometry* geom, const KDL::Frame &T_O_G) {
    int keys_removed = 0;

    const double dist_mult = 2.0;
    if (geom->getType() == Geometry::SPHERE) {
        const Sphere *sp = static_cast<const Sphere*>(geom);
        KDL::Vector p = T_O_G.p;
        double r = sp->getRadius() + dist_mult * oc->getResolution();

        KDL::Vector bb_min = (p - KDL::Vector(r,r,r));
        KDL::Vector bb_max = (p + KDL::Vector(r,r,r));

        octomap::OcTreeKey key = oc->coordToKey( bb_min.x(), bb_min.y(), bb_min.z() );
        int steps_i = std::max(1.0, (bb_max.x()-bb_min.x()) / oc->getResolution() );
        int steps_j = std::max(1.0, (bb_max.y()-bb_min.y()) / oc->getResolution() );
        int steps_k = std::max(1.0, (bb_max.z()-bb_min.z()) / oc->getResolution() );
        for (int i = 0; i < steps_i; ++i) {
            for (int j = 0; j < steps_j; ++j) {
                for (int k = 0; k < steps_k; ++k) {
                    octomap::OcTreeKey key_it(i+key[0],j+key[1],k+key[2]);
                    octomath::Vector3 coord = oc->keyToCoord(key_it);
                    octomap::OcTreeNode *node = oc->search(coord);
                    if (!node || !oc->isNodeOccupied(node)) {
                        continue;
                    }
                    KDL::Vector pt(coord.x(),coord.y(),coord.z());
                    double dist = (p - pt).Norm();
                    if (dist < r) {
                        ++keys_removed;
                        oc->updateNode(key_it, false, true);
                    }
                }
            }
        }
    }
    else if (geom->getType() == Geometry::CAPSULE) {
        const Capsule *ca = static_cast<const Capsule*>(geom);

        const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom->shape.get());

        // capsules are shifted by length/2
        double x1,y1,z1,w1;
        KDL::Frame tf1_corrected = T_O_G;
        tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);

        double r = ca->getRadius() + dist_mult * oc->getResolution();

        KDL::Vector cap1 = T_O_G * KDL::Vector(0,0,ca->getLength()/2);
        KDL::Vector cap2 = T_O_G * KDL::Vector(0,0,-ca->getLength()/2);

        // create fake sphere for all octomap leafs
        fcl_2::Sphere shape_sp(dist_mult*oc->getResolution());

        KDL::Vector bb_min = (KDL::Vector(std::min(cap1.x(), cap2.x()), std::min(cap1.y(), cap2.y()), std::min(cap1.z(), cap2.z())) - KDL::Vector(r,r,r));
        KDL::Vector bb_max = (KDL::Vector(std::max(cap1.x(), cap2.x()), std::max(cap1.y(), cap2.y()), std::max(cap1.z(), cap2.z())) + KDL::Vector(r,r,r));

        octomap::OcTreeKey key = oc->coordToKey( bb_min.x(), bb_min.y(), bb_min.z() );
        int steps_i = std::max(1.0, (bb_max.x()-bb_min.x()) / oc->getResolution() );
        int steps_j = std::max(1.0, (bb_max.y()-bb_min.y()) / oc->getResolution() );
        int steps_k = std::max(1.0, (bb_max.z()-bb_min.z()) / oc->getResolution() );
        for (int i = 0; i < steps_i; ++i) {
            for (int j = 0; j < steps_j; ++j) {
                for (int k = 0; k < steps_k; ++k) {
                    octomap::OcTreeKey key_it(i+key[0],j+key[1],k+key[2]);
                    octomath::Vector3 coord = oc->keyToCoord(key_it);
                    octomap::OcTreeNode *node = oc->search(coord);
                    if (!node || !oc->isNodeOccupied(node)) {
                        continue;
                    }
                    // output variables
                    fcl_2::Vec3f p1, p2, n1, n2;
                    double dist;
                    bool result = CollisionModel::gjk_solver.shapeDistance(
                        shape_sp, fcl_2::Transform3f(fcl_2::Quaternion3f(1,0,0,0), fcl_2::Vec3f(coord.x(),coord.y(),coord.z())),
                        *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
                         &dist, &p2, &p1, &n2, &n1);

                    if (dist < 0) {
                        ++keys_removed;
                        oc->updateNode(key_it, false, true);
                    }
                }
            }
        }
    }
    return keys_removed;
}

}    // namespace self_collision


