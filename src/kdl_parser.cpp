/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Wim Meeussen */

#include "kdl_parser.hpp"
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/frames_io.hpp>

#include <base/Logging.hpp>

using namespace std;
using namespace KDL;

namespace kdl_parser{

// construct vector
Vector toKdl(urdf::Vector3 v)
{
    return Vector(v.x, v.y, v.z);
}

// construct rotation
Rotation toKdl(urdf::Rotation r)
{
    return Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
Frame toKdl(urdf::Pose p)
{
    return Frame(toKdl(p.rotation), toKdl(p.position));
}

// construct joint
Joint toKdl(boost::shared_ptr<urdf::Joint> jnt)
{
    Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

    switch (jnt->type){
    case urdf::Joint::FIXED:{
        return KDL::Joint(jnt->name, KDL::Joint::None);
    }
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:{
        KDL::Vector axis = toKdl(jnt->axis);
        return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
    }
    case urdf::Joint::PRISMATIC:{
        KDL::Vector axis = toKdl(jnt->axis);
        return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::TransAxis);
    }
    default:{
        LOG_ERROR("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
        return Joint(jnt->name, Joint::None);
    }
    }
    return Joint();
}

// construct inertia
RigidBodyInertia toKdl(boost::shared_ptr<urdf::Inertial> i)
{
    Frame origin = toKdl(i->origin);
    // kdl specifies the inertia in the reference frame of the link, the urdf specifies the inertia in the inertia reference frame
    return origin.M * RigidBodyInertia(i->mass, origin.p, RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz));
}


// recursive function to walk through tree
bool addChildrenToTree(boost::shared_ptr<const urdf::Link> root, Tree& tree)
{
    std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
    LOG_DEBUG("DEBUG: Link %s had %i children", root->name.c_str(), (int)children.size());

    // constructs the optional inertia
    RigidBodyInertia inert(0);
    if (root->inertial)
        inert = toKdl(root->inertial);

    // constructs the kdl joint
    Joint jnt = toKdl(root->parent_joint);
    LOG_DEBUG("After adding joint");

    // construct the kdl segment
    Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

    // add segment to tree
    tree.addSegment(sgm, root->parent_joint->parent_link_name);

    // recurslively add all children
    for (size_t i=0; i<children.size(); i++){
        if (!addChildrenToTree(children[i], tree))
            return false;
    }
    return true;
}


bool treeFromFile(const string& path, Tree& tree)
{
    std::ifstream file(path.c_str());
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return treeFromString(str, tree);
}

#if 0
bool treeFromParam(const string& param, Tree& tree)
{
    urdf::Model robot_model;
    if (!robot_model.initParam(param)){
        ROS_ERROR("Could not generate robot model");
        return false;
    }
    return treeFromUrdfModel(robot_model, tree);
}
#endif

bool treeFromString(const string& xml, Tree& tree)
{
    boost::shared_ptr<urdf::ModelInterface> robot_model = urdf::parseURDF(xml);
    return treeFromUrdfModel(*robot_model, tree);
}

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, Tree& tree)
{
    tree = Tree(robot_model.getRoot()->name);

    // warn if root link has inertia. KDL does not support this
    if (robot_model.getRoot()->inertial)
        LOG_DEBUG("WARN: The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.", robot_model.getRoot()->name.c_str());

    //  add all children
    for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
        if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree))
            return false;

    return true;
}

/*
 * SDF to KDL
 * bellow there are the source codes to convert SDFs information to KDL
 */

typedef std::map<std::string, sdf::ElementPtr> JointsMap;
typedef std::map<std::string, sdf::ElementPtr> LinksMap;
typedef std::map<std::string, std::vector<std::string> > LinkNamesMap;

/**
 * convert <axis> element to KDL::Vector
 */
KDL::Vector sdf_axis_to_kdl(sdf::ElementPtr sdf)
{
    double x, y, z;
    std::string axis_value = sdf->GetElement("xyz")->Get<std::string>();
    sscanf(axis_value.c_str(), "%lf %lf %lf", &x, &y, &z);
    return KDL::Vector(x, y, z);
}

/**
 * convert <pose> element to KDL::Frame
 */
KDL::Frame sdf_pose_to_kdl(sdf::ElementPtr sdf)
{

    double x, y, z;
    double roll, pitch, yaw;
    std::string pose_value = sdf->Get<std::string>();

    sscanf(pose_value.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &roll, &pitch, &yaw);

    KDL::Rotation rotation = KDL::Rotation::RPY(roll, pitch, yaw);
    KDL::Vector position = KDL::Vector(x, y, z);

    return KDL::Frame(rotation, position);
}

/**
 * convert <inertial> element to KDL::RigidBodyInertia
 */
KDL::RigidBodyInertia sdf_inertial_to_kdl(sdf::ElementPtr sdf)
{

    KDL::Frame pose = sdf_pose_to_kdl(sdf->GetElement("pose"));
    double mass;

    if (sdf->HasElement("mass")){
        mass = sdf->GetElement("mass")->Get<double>();
        if (sdf->HasElement("inertia")){

            sdf::ElementPtr sdf_inertia = sdf->GetElement("inertia");

            char *tags[] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz" };

            double ixx, ixy, ixz, iyy, iyz, izz;
            double *ptr[] = {&ixx, &ixy, &ixz, &iyy, &iyz, &izz};

            for (int i = 0; i < 6; i++){
                (*ptr[i]) = sdf_inertia->GetElement(tags[i])->Get<double>();
            }

            return pose.M * RigidBodyInertia(mass, pose.p,
                                            RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz));
        }
    }

    return KDL::RigidBodyInertia();
}

/**
 * convert <joint> element to KDL::Joint
 */
KDL::Joint sdf_joint_to_kdl(sdf::ElementPtr sdf){

    std::string name = sdf->Get<std::string>("name");
    std::string type = sdf->Get<std::string>("type");

    if (sdf->HasElement("pose")){
         KDL::Frame pose = sdf_pose_to_kdl(sdf->GetElement("pose"));
         if (type == "revolute"){
             KDL::Vector axis = sdf_axis_to_kdl(sdf->GetElement("axis"));
             return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::RotAxis);
         }
         else if (type == "prismatic"){
             KDL::Vector axis = sdf_axis_to_kdl(sdf->GetElement("axis"));
             return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::TransAxis);
         }

    }

    return KDL::Joint(KDL::Joint::None);
}


/**
 * Fill KDL::Tree with SDF information
 */
void fillKdlTreeFromSDF(LinkNamesMap linkNames,
                        JointsMap joints,
                        LinksMap links,
                        std::string parent_name,
                        KDL::Tree& tree)
{

    LinkNamesMap::iterator parentItr = linkNames.find(parent_name);

    if (parentItr != linkNames.end()){
        for (std::vector<std::string>::iterator childItr = parentItr->second.begin();
             childItr != parentItr->second.end(); childItr++) {

            KDL::Joint joint;
            KDL::RigidBodyInertia I;

            LinksMap::iterator link_itr = links.find(*childItr);

            if (link_itr != links.end()){
                sdf::ElementPtr sdf_link = link_itr->second;

                if (sdf_link->HasElement("inertial")){
                    I = sdf_inertial_to_kdl(sdf_link->GetElement("inertial"));
                }
            }

            JointsMap::iterator  joint_itr = joints.find(*childItr);

            if (joint_itr != joints.end()){
                sdf::ElementPtr sdf_joint = joint_itr->second;
                joint = sdf_joint_to_kdl(sdf_joint);
            }


            KDL::Segment segment(*childItr, joint,
                                            KDL::Frame(),
                                            I);

            tree.addSegment(segment, parent_name);

            fillKdlTreeFromSDF(linkNames, joints, links, *childItr, tree);
        }
    }
}

/**
 * create a data structure to map <joint> elements
 * use link child as key to map refence the joints
 * if joint is a child then it has a joint
 */
JointsMap sdfLoadJoints(sdf::ElementPtr sdf_model)
{
    JointsMap joints;

    if (sdf_model->HasElement("joint")) {
        sdf::ElementPtr jointElem = sdf_model->GetElement("joint");

        while (jointElem) {
            std::string childLinkName = jointElem->GetElement("child")->Get<std::string>();
            joints.insert(std::make_pair<std::string, sdf::ElementPtr>(childLinkName, jointElem));
            jointElem = jointElem->GetNextElement("joint");
        }
    }

    return joints;
}

/**
 * create a data structure to map <link> elements
 * use link name as key to map the links
 */
LinksMap sdfLoadLinks(sdf::ElementPtr sdf_model)
{
    std::map<std::string, sdf::ElementPtr> links;

    if (sdf_model->HasElement("link")) {
        sdf::ElementPtr linkElem = sdf_model->GetElement("link");
        while (linkElem) {
            std::string linkName = linkElem->Get<std::string>("name");
            links.insert(std::make_pair<std::string, sdf::ElementPtr>(linkName, linkElem));
            linkElem = linkElem->GetNextElement("link");
        }
    }

    return links;
}

/**
 * create a structure to map parent and child
 * each parent has a list of children
 * this result is used to fill KDL::Tree
 */
LinkNamesMap sdfBuildLinkNames(LinksMap links, JointsMap joints, std::string rootName){

    LinkNamesMap linkNames;
    LinksMap::iterator linksItr;
    JointsMap::iterator jointItr;

    //list each link and build and associate each child to its parent
    for (linksItr = links.begin(); linksItr != links.end(); linksItr++){
        std::string child_name = linksItr->first;
        std::string parent_name = rootName;

        //check if child has a parent
        if ((jointItr = joints.find(child_name)) != joints.end()) {
            parent_name = jointItr->second->GetElement("parent")->Get<std::string>();
        }

        //insert child name in vector associated with parent name
        LinkNamesMap::iterator itr = linkNames.find(parent_name);
        if (itr == linkNames.end()){
            std::vector<std::string> childs;
            childs.push_back(child_name);
            linkNames.insert(std::make_pair<std::string, std::vector<std::string> >(parent_name, childs));
        }
        else {
            itr->second.push_back(child_name);
        }
    }

    return linkNames;

}

void treeFromSdfModel(const sdf::ElementPtr& sdf_model, KDL::Tree& out)
{
    // model is the root segment
    std::string modelName = sdf_model->Get<std::string>("name");

    // map parents and children links
    LinksMap links = sdfLoadLinks(sdf_model);

    // map links using link name
    JointsMap joints = sdfLoadJoints(sdf_model);

    // map joints using child link names
    LinkNamesMap linkNames = sdfBuildLinkNames(links, joints, modelName);

    //build KDL::Tree using SDF information
    Tree tree(modelName);
    fillKdlTreeFromSDF(linkNames, joints, links, tree.getRootSegment()->first, tree);
    out = tree;
}

}

