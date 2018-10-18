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

#ifndef COLLISION_CONVEX_MODEL_H
#define COLLISION_CONVEX_MODEL_H

#include "ros/ros.h"
#include <octomap/octomap.h>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <math.h> 
#include <tinyxml.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"

namespace fcl_2
{
class ShapeBase;
class Capsule;
class Convex;
class GJKSolver_indep;
}

namespace self_collision
{

class Geometry
{
public:
	enum {UNDEFINED=0, CAPSULE, CONVEX, SPHERE, TRIANGLE, OCTOMAP};
	boost::shared_ptr<fcl_2::ShapeBase> shape;
	Geometry(int type);
	virtual void clear() = 0;
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array) = 0;
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr) = 0;
	int marker_id_;
    int getType() const;
    const std::string& getTypeStr() const;
    double getBroadphaseRadius() const;

    void setColor(double cr, double cg, double cb, double ca);
    void getColor(double &cr, double &cg, double &cb, double &ca) const;

    std::string visualisation_hint_;

protected:
    double cr_, cg_, cb_, ca_;
    double broadphase_radius_;
private:
	int type_;
};

class Octomap : public Geometry
{
public:
    Octomap();
    Octomap(const boost::shared_ptr<octomap::OcTree > &om);
    void setOctomap(const boost::shared_ptr<octomap::OcTree > &om);
    const boost::shared_ptr<octomap::OcTree >& getOctomap() const;

	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
protected:
    boost::shared_ptr<octomap::OcTree > om_;
};

class Capsule : public Geometry
{
public:
	Capsule();
	Capsule(double radius, double length);
    void setSize(double radius, double length);
    double getRadius() const;
    double getLength() const;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
	double radius;
	double length;
};

class Sphere : public Geometry
{
public:
	Sphere();
	Sphere(double radius);
    void setSize(double radius);
    double getRadius() const;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
	double radius;
};

class Triangle : public Geometry
{
public:
	Triangle();
	Triangle(const KDL::Vector &v1, const KDL::Vector &v2, const KDL::Vector &v3);
    void setPoints(const KDL::Vector &v1, const KDL::Vector &v2, const KDL::Vector &v3);
    void getPoints(KDL::Vector &v1, KDL::Vector &v2, KDL::Vector &v3) const;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
	KDL::Vector v1_, v2_, v3_;
	KDL::Vector v1_L_, v2_L_, v3_L_;
    KDL::Frame T_O_L_;
    KDL::Frame T_L_O_;

    KDL::Frame T_O_C1_, T_O_C2_, T_O_C3_;
    boost::shared_ptr<fcl_2::Capsule > c1_, c2_, c3_;
};

class Convex : public Geometry
{
public:
	Convex();
	~Convex();
	bool updateConvex(int num_points, const std::vector<geometry_msgs::Point> &points, int num_planes, const std::vector<int> &polygons);
	bool updateConvex(int num_points, const std::vector<KDL::Vector> &points, int num_planes, const std::vector<int> &polygons);

	typedef std::vector<std::pair<std::string, KDL::Vector> > ConvexPointsStrVector;
	typedef std::vector<std::pair<int, KDL::Vector> > ConvexPointsIdVector;
	ConvexPointsStrVector points_str_;
	ConvexPointsIdVector points_id_;
	KDL::Vector center_;
	double radius_;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);

    const std::vector<KDL::Vector >& getPoints() const;
    const std::vector<int >& getPolygons() const;

protected:
    void calculateRadius();
    void updateInternalData();

    const int max_points_vec_size_;
    const int max_polygons_vec_size_;

    std::vector<KDL::Vector > points_;
    std::vector<int > polygons_;
};

class Link;

class Collision
{
public:
	void clear();
	boost::shared_ptr< Geometry > geometry;
	KDL::Frame origin;
    int parent_link_idx_;
private:
};

class VisualGeometry
{
public:
	enum {UNDEFINED=0, MESH};
	VisualGeometry(int type);
    int getType() const;
private:
	int type_;
};

class VisualMesh : public VisualGeometry
{
public:
    VisualMesh();
    std::string filename_;
};


class Visual
{
public:
    Visual();
    KDL::Frame origin_;
    boost::shared_ptr<VisualGeometry > geom_;    
};

class Link
{
public:
    Link();
	void clear();
	std::string name;
	int index_;
	int parent_index_;
	const KDL::TreeElement *kdl_segment_;
	typedef std::vector< boost::shared_ptr< Collision > > VecPtrCollision;
	VecPtrCollision collision_array;
    std::vector<boost::shared_ptr<Visual > > visual_array_;
private:
};

class Joint
{
public:
	void clear();

    std::string name_;
    double lower_limit_, upper_limit_;
};

class CollisionModel
{
public:
	static boost::shared_ptr<CollisionModel> parseURDF(const std::string &xml_string);
    static bool convertSelfCollisionsInURDF(const std::string &xml_in, std::string &xml_out);
	void parseSRDF(const std::string &xml_string);

    bool addLink(const std::string &name, const std::string &parent_name, const std::vector< boost::shared_ptr< Collision > > &col_array);
	std::string name_;
    bool addCollisionToLink(const std::string &link_name, const boost::shared_ptr< Collision > &pcol, const KDL::Frame &T_L_C);
    bool removeCollisionFromLink(const std::string &link_name, const boost::shared_ptr< Collision > &pcol);

	typedef std::vector<std::pair<int, int> > CollisionPairs;
	CollisionPairs disabled_collisions;
	CollisionPairs enabled_collisions;

	const boost::shared_ptr< Link > getLink(int id) const;
	const boost::shared_ptr< Link > getLink(const std::string &link_name) const;
	int getLinkIndex(const std::string &name) const;
	const std::string &getLinkName(int idx) const;
	void generateCollisionPairs();
    static bool getDistance(const Geometry *geom1, const KDL::Frame &tf1, const Geometry *geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, KDL::Vector &n1_out, KDL::Vector &n2_out, double d0, double &distance);
    static bool checkRayCollision(const Geometry *geom, const KDL::Frame &tf, const KDL::Vector &ray_start, const KDL::Vector &ray_end);

    typedef std::vector< boost::shared_ptr< Link > > VecPtrLink;

    const VecPtrLink &getLinks() const;
    int getLinksCount() const;

    const Link::VecPtrCollision &getLinkCollisionArray(int idx) const;

	static fcl_2::GJKSolver_indep gjk_solver;

protected:
	CollisionModel();

	bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);

	static bool parsePose(KDL::Frame &pose, TiXmlElement* xml);
	static bool parseSphere(Sphere &s, TiXmlElement *c);
	static bool parseCapsule(Capsule &s, TiXmlElement *c);
	static bool parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c);
	static bool parseConvex(Convex &s, TiXmlElement *c);
	static boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g);
	static bool parseCollision(Collision &col, TiXmlElement* config);
	static bool parseLink(Link &link, TiXmlElement* config);
    static bool parseLimit(Joint &joint, TiXmlElement* o);
    static bool parseJoint(Joint &joint, TiXmlElement* o);
    static boost::shared_ptr<VisualGeometry > parseVisualGeometry(TiXmlElement* config);
    static boost::shared_ptr<Visual > parseVisual(TiXmlElement* config);

    VecPtrLink links_;
	int link_count_;
	int root_index_;

    std::vector<Joint> joints_;

    std::map<std::string, int > link_name_idx_map_;
};

class CollisionInfo {
public:
    CollisionInfo()
        : link1_idx(-1)
        , link2_idx(-1)
        , dist(0)
    {}

    int link1_idx;
    int link2_idx;
    KDL::Vector p1_B;
    KDL::Vector p2_B;
    double dist;
    KDL::Vector n1_B;
    KDL::Vector n2_B;
};

bool compareCollisionInfoDist(const CollisionInfo &i1, const CollisionInfo &i2);

boost::shared_ptr< self_collision::Collision > createCollisionCapsule(double radius, double length, const KDL::Frame &origin);
boost::shared_ptr< self_collision::Collision > createCollisionSphere(double radius, const KDL::Frame &origin);
boost::shared_ptr< self_collision::Collision > createCollisionConvex(const std::vector<KDL::Vector > &vertices, const std::vector<int> &polygons, const KDL::Frame &origin, const std::string &visualisation_hint="lines");
boost::shared_ptr< self_collision::Collision > createCollisionOctomap(const boost::shared_ptr<octomap::OcTree > &om, const KDL::Frame &origin);

void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                        double activation_dist, std::vector<self_collision::CollisionInfo> &link_collisions);

void getCollisionPairsNoAlloc(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                                double activation_dist, std::vector<self_collision::CollisionInfo> &link_collisions);

bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol, const KDL::Frame &T_B_L1, const std::vector<KDL::Frame > &links_fk,
                    const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::set<int> &excluded_link_idx);

bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Collision > &pcol2, const KDL::Frame &T_B_L2);

bool checkCollision(const boost::shared_ptr< self_collision::Collision > &pcol1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Link > &link2, const KDL::Frame &T_B_L2);

bool checkCollision(const boost::shared_ptr< self_collision::Link > &link1, const KDL::Frame &T_B_L1, const boost::shared_ptr< self_collision::Link > &link2, const KDL::Frame &T_B_L2, double *min_dist=NULL);

bool checkCollision(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                    const std::set<int> &excluded_link_idx);

int removeNodesFromOctomap(boost::shared_ptr<octomap::OcTree > &oc, const Geometry* geom, const KDL::Frame &T_O_G);

}	// namespace self_collision

#endif	// COLLISION_CONVEX_MODEL_H

