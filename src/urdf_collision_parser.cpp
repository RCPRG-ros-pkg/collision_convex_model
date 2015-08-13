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

#include "ros/ros.h"
#include "collision_convex_model/collision_convex_model.h"
#include "urdf/model.h"
#include <kdl/frames.hpp>
#include <tinyxml.h>
#include "narrowphase.h"

namespace self_collision
{

fcl_2::GJKSolver_indep CollisionModel::gjk_solver;

Geometry::Geometry(int type) :
	type_(type)
{
}

int Geometry::getType() const {
    return type_;
}

Capsule::Capsule() :
	Geometry(CAPSULE)
{
}

Capsule::Capsule(double radius, double length) :
	Geometry(CAPSULE)
{
    this->radius = radius;
    this->length = length;
    shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(radius, length)) );
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

/*	visualization_msgs::Marker marker4;
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

/*	marker_array.markers[marker_id_+3].header.stamp = ros::Time();
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
    this->radius = radius;
    shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Sphere(radius)) );
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

Convex::Convex() :
	Geometry(CONVEX)
{
	// allocate a lot of memory and initialize very simple convex hull mesh (4 points and 4 polygons)
	int num_points = 4;
	fcl_2::Vec3f* points = new fcl_2::Vec3f[5000];
	int num_planes = 4;
	int *polygons = new int[20000];
	points[0] = fcl_2::Vec3f(0.0, 0.0, 0.0);
	points[1] = fcl_2::Vec3f(0.1, 0.0, 0.0);
	points[2] = fcl_2::Vec3f(0.0, 0.1, 0.0);
	points[3] = fcl_2::Vec3f(0.0, 0.0, 0.1);
	polygons[0] = 3;	polygons[1] = 2;	polygons[2] = 1;	polygons[3] = 0;
	polygons[4] = 3;	polygons[5] = 3;	polygons[6] = 1;	polygons[7] = 0;
	polygons[8] = 3;	polygons[9] = 3;	polygons[10] = 2;	polygons[11] = 0;
	polygons[12] = 3;	polygons[13] = 3;	polygons[14] = 2;	polygons[15] = 1;
	shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)) );
}

Convex::~Convex()
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
	delete[] conv->polygons;
	conv->polygons = NULL;
	delete[] conv->points;
	conv->points = NULL;
}

void Convex::updateConvex(int num_points, const std::vector<geometry_msgs::Point> &points, int num_planes, const std::vector<int> &polygons)
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
	int polygons_idx = 0;
	for (int f_idx=0; f_idx<num_planes; f_idx++)
	{
		polygons_idx += polygons[polygons_idx] + 1;
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

boost::shared_ptr< const Link > CollisionModel::getLink(int id)
{
	if (id < 0 || id >= link_count_)
	{
		ROS_ERROR("CollisionModel::getLink: id out of range: 0 <= %d < %d", id, link_count_);
		return boost::shared_ptr< const Link >();
	}
	return links_[id];
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
	if (!c->Attribute("radius"))
	{
		ROS_ERROR("Capsule shape must have a radius attribute");
		return false;
	}
	try
	{
		s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
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
		s.length = boost::lexical_cast<double>(c->Attribute("length"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "length [" << c->Attribute("length") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}
	return true;
}

bool CollisionModel::parseSphere(Sphere &s, TiXmlElement *c)
{
	s.clear();
	if (!c->Attribute("radius"))
	{
		ROS_ERROR("Capsule shape must have a radius attribute");
		return false;
	}
	try
	{
		s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}

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
			
			s.points_str_.push_back(std::make_pair<std::string, KDL::Vector>(frame, point));
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
			s->shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(s->radius, s->length)) );
			return geom;
		}
	}
	else if (type_name == "sphere")
    {
		Sphere *s = new Sphere();
		geom.reset(s);
		if (parseSphere(*s, shape_xml))
		{
			s->shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Sphere(s->radius)) );
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
			col.reset();
			ROS_ERROR("Could not parse collision element for Link [%s]", link.name.c_str());
			return false;
		}
	}
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

void CollisionModel::parseSRDF(const std::string &xml_string)
{
	disabled_collisions.clear();

	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		ROS_ERROR("%s", xml_doc.ErrorDesc());
		xml_doc.ClearError();
		return;
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ROS_ERROR("Could not find the 'robot' element in the xml file");
		return;
	}
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ROS_ERROR("No name given for the robot.");
		return;
	}

	if (name_ != std::string(name))
	{
		ROS_ERROR("Name from SRDF: %s differ from %s", name, name_.c_str());
		return;
	}

	// Get all disable_collisions elements
	for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions"); disable_collision_xml; disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
	{
		std::string link1, link2;
		try {
			parseDisableCollision(link1, link2, disable_collision_xml);
			int link1_id = getLinkIndex(link1);
			int link2_id = getLinkIndex(link2);
			if (link1_id == -1)
			{
				ROS_ERROR("link '%s' does not exist.", link1.c_str());
				return;
			}
			if (link2_id == -1)
			{
				ROS_ERROR("link '%s' does not exist.", link2.c_str());
				return;
			}
			disabled_collisions.push_back(std::make_pair<int, int>(link1_id, link2_id));
		}
		catch (urdf::ParseError &e) {
			ROS_ERROR("disable_collisions xml is not initialized correctly");
			return;
		}
	}


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
					conv->points_id_.push_back( std::make_pair<int, KDL::Vector>(id, p_it->second) );
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

void CollisionModel::generateCollisionPairs()
{
	enabled_collisions.clear();
//	ROS_INFO("%ld", links_.size());

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
			for (CollisionPairs::iterator dc_it = disabled_collisions.begin(); dc_it != disabled_collisions.end(); dc_it++)
			{
				if (	(dc_it->first == l_i && dc_it->second == l_j) ||
					(dc_it->second == l_i && dc_it->first == l_j) )
				{
					add = false;
					break;
				}
			}
			if (add)
			{
				for (CollisionPairs::iterator ec_it = enabled_collisions.begin(); ec_it != enabled_collisions.end(); ec_it++)
				{
					if (	(ec_it->first == l_i && ec_it->second == l_j) ||
						(ec_it->second == l_i && ec_it->first == l_j) )
					{
						add = false;
						break;
					}
				}

				if (add)
				{
					enabled_collisions.push_back(std::make_pair<int, int>(l_i, l_j));
				}
			}
		}
	}
}

bool CollisionModel::getDistance(const boost::shared_ptr<Geometry > &geom1, const KDL::Frame &tf1, const boost::shared_ptr<Geometry > &geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, double d0, double &distance)
{
	if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CAPSULE");
        const boost::shared_ptr<fcl_2::Capsule >  ob1 = boost::static_pointer_cast<fcl_2::Capsule >(geom1->shape);
        const boost::shared_ptr<fcl_2::Capsule >  ob2 = boost::static_pointer_cast<fcl_2::Capsule >(geom2->shape);

		// calculate narrowphase distance
		if ((tf1.p - tf2.p).Norm() > ob1->radius + ob1->lz/2.0 + ob2->radius + ob2->lz/2.0 + d0)
		{
			distance = d0 * 2.0;
            return true;
		}

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2 * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		bool result = gjk_solver.shapeDistance(
			*ob1.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			*ob2.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in global coordinates
		d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
		return result;
	}
	else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CONVEX");
        const boost::shared_ptr<fcl_2::Capsule >  ob1 = boost::static_pointer_cast<fcl_2::Capsule >(geom1->shape);
        const boost::shared_ptr<fcl_2::Convex >  ob2 = boost::static_pointer_cast<fcl_2::Convex >(geom2->shape);

        const boost::shared_ptr<Convex > conv2 = boost::static_pointer_cast<Convex>(geom2);

		// calculate narrowphase distance
		if ((tf1.p - tf2 * conv2->center_).Norm() > ob1->radius + ob1->lz/2.0 + conv2->radius_ + d0)
		{
			distance = d0 * 2.0;
            return true;
		}


		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1;// * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		tf2.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		bool result = gjk_solver.shapeDistance(
			*ob1.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			*ob2.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in wtf coordinates
		d1_out = tf1_corrected*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1_corrected*((tf1_corrected.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return result;
	}
	else if (geom1->getType() == Geometry::CONVEX && geom2->getType() == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX, CAPSULE");
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, d0, distance);
	}
	else if (geom1->getType() == Geometry::CONVEX && geom2->getType() == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX,CONVEX");
        const boost::shared_ptr<fcl_2::Convex >  ob1 = boost::static_pointer_cast<fcl_2::Convex >(geom1->shape);
        const boost::shared_ptr<fcl_2::Convex >  ob2 = boost::static_pointer_cast<fcl_2::Convex >(geom2->shape);

        const boost::shared_ptr<Convex > conv1 = boost::static_pointer_cast<Convex>(geom1);
        const boost::shared_ptr<Convex > conv2 = boost::static_pointer_cast<Convex>(geom2);

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
		bool result = gjk_solver.shapeDistance(
			*ob1.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
			*ob2.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
			 &distance, &p1, &p2);

		// the output for two capsules is in wtf coordinates
		d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1*((tf1.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return result;
	}
	else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::SPHERE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: SPHERE,SPHERE");
        const boost::shared_ptr<fcl_2::Sphere >  ob1 = boost::static_pointer_cast<fcl_2::Sphere >(geom1->shape);
        const boost::shared_ptr<fcl_2::Sphere >  ob2 = boost::static_pointer_cast<fcl_2::Sphere >(geom2->shape);

		// calculate narrowphase distance
		if ((tf1.p - tf2.p).Norm() > ob1->radius + ob2->radius + d0)
		{
            distance = d0 * 2.0;
			return true;
		}

		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1;
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2;
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		bool result = gjk_solver.shapeDistance(
			*ob1.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			*ob2.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			 &distance, &p1, &p2);
		// the output for two spheres is in global coordinates
		d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
		return result;
	}
	else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::SPHERE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,SPHERE");
        const boost::shared_ptr<fcl_2::Capsule >  ob1 = boost::static_pointer_cast<fcl_2::Capsule >(geom1->shape);
        const boost::shared_ptr<fcl_2::Sphere >  ob2 = boost::static_pointer_cast<fcl_2::Sphere >(geom2->shape);

		// calculate narrowphase distance
		if ((tf1.p - tf2.p).Norm() > ob1->radius + ob1->lz/2.0 + ob2->radius + d0)
		{
            distance = d0 * 2.0;
			return true;
		}

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1;// * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2;
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		bool result = gjk_solver.shapeDistance(
			*ob2.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			*ob1.get(), fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			 &distance, &p2, &p1);

		// the output is in objects coordinate frames
		d1_out = tf1_corrected * KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf2_corrected * KDL::Vector(p2[0], p2[1], p2[2]);

		return result;
	}
	else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::CAPSULE)
	{
        return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, d0, distance);
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

const CollisionModel::VecPtrLink &CollisionModel::getLinks() const {
    return links_;
}

int CollisionModel::getLinksCount() const {
    return link_count_;
}

bool CollisionModel::getJointLimits(const std::string &joint_name, double &lower, double &upper) const {
    for (std::vector<Joint>::const_iterator j_it = joints_.begin(); j_it != joints_.end(); j_it++) {
        if (joint_name == j_it->name_) {
            lower = j_it->lower_limit_;
            upper = j_it->upper_limit_;
            return true;
        }
    }
    return false;
}

const Link::VecPtrCollision &CollisionModel::getLinkCollisionArray(int idx) const {
    return links_[idx]->collision_array;
}

boost::shared_ptr< self_collision::Collision > createCollisionCapsule(double radius, double length, const KDL::Frame &origin) {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Capsule(radius, length));
        boost::shared_ptr<self_collision::Capsule > cap = boost::static_pointer_cast<self_collision::Capsule >(pcol->geometry);
        pcol->origin = origin;
        return pcol;
}

boost::shared_ptr< self_collision::Collision > createCollisionSphere(double radius, const KDL::Frame &origin) {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Sphere(radius));
        boost::shared_ptr<self_collision::Sphere > sph = boost::static_pointer_cast<self_collision::Sphere >(pcol->geometry);
        pcol->origin = origin;
        return pcol;
}

void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                            double activation_dist, std::vector<self_collision::CollisionInfo> &link_collisions) {
        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
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
                    if (!self_collision::CollisionModel::getDistance((*col1)->geometry, T_B_C1, (*col2)->geometry, T_B_C2, p1_B, p2_B, activation_dist, dist)) {
                        std::cout << "ERROR: getCollisionPairs: dist < 0" << std::endl;
                    }

                    if (dist < activation_dist) {
                        self_collision::CollisionInfo col_info;
                        col_info.link1_idx = link1_idx;
                        col_info.link2_idx = link2_idx;
                        col_info.dist = dist;
                        n1_B = (p2_B - p1_B) / dist;
                        n2_B = -n1_B;
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

                if (!self_collision::CollisionModel::getDistance(pcol->geometry, T_B_C1, (*col2_it)->geometry, T_B_C2, p1_B, p2_B, 0.01, dist)) {
                    return true;
                }
            }
        }

        return false;
}

bool checkCollision(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk, double activation_dist, const std::set<int> &excluded_link_idx) {
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
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
                    if (!self_collision::CollisionModel::getDistance((*col1)->geometry, T_B_C1, (*col2)->geometry, T_B_C2, p1_B, p2_B, 0.01, dist)) {
                        return true;
                    }
                }
            }
        }
        return false;
}


}	// namespace self_collision


