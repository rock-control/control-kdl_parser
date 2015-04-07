#include <base/Logging.hpp>
#include <kdl/tree.hpp>
#include <sdf/sdf.hh>


namespace kdl_parser {
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
KDL::Vector toKdl(sdf::Vector3 axis)
{
    return KDL::Vector(axis.x, axis.y, axis.z);
}

/**
 * convert <pose> element to KDL::Frame
 */
KDL::Frame toKdl(sdf::Pose pose)
{
    KDL::Rotation rotation = KDL::Rotation::Quaternion(pose.rot.x, pose.rot.y, pose.rot.x, pose.rot.w);
    KDL::Vector position = KDL::Vector(pose.pos.x, pose.pos.y, pose.pos.z);
    return KDL::Frame(rotation, position);
}

/**
 * convert <inertial> element to KDL::RigidBodyInertia
 */
KDL::RigidBodyInertia sdfInertiaToKdl(sdf::ElementPtr sdf)
{
    KDL::Frame pose = toKdl(sdf->GetElement("pose")->Get<sdf::Pose>());
    double mass;

    if (sdf->HasElement("mass")){
        mass = sdf->GetElement("mass")->Get<double>();
        if (sdf->HasElement("inertia")){

            sdf::ElementPtr sdf_inertia = sdf->GetElement("inertia");

            const char *tags[] = {"ixx", "ixy", "ixz", "iyy", "iyz", "izz" };

            double ixx, ixy, ixz, iyy, iyz, izz;
            double *ptr[] = {&ixx, &ixy, &ixz, &iyy, &iyz, &izz};

            for (int i = 0; i < 6; i++){
                (*ptr[i]) = sdf_inertia->GetElement(tags[i])->Get<double>();
            }

            return pose.M * KDL::RigidBodyInertia(mass, pose.p, KDL::RotationalInertia(ixx, iyy, izz, ixy, ixz, iyz));
        }
    }

    return KDL::RigidBodyInertia();
}

/**
 * convert <joint> element to KDL::Joint
 */
KDL::Joint sdfJointToKdl(sdf::ElementPtr sdf)
{
    std::string name = sdf->Get<std::string>("name");
    std::string type = sdf->Get<std::string>("type");

    if (sdf->HasElement("pose")){
         KDL::Frame pose = toKdl(sdf->GetElement("pose")->Get<sdf::Pose>());
         if (type == "revolute"){
             KDL::Vector axis = toKdl(sdf->GetElement("axis")->Get<sdf::Vector3>());
             return KDL::Joint(name, pose.p, pose.M * axis, KDL::Joint::RotAxis);
         }
         else if (type == "prismatic"){
             KDL::Vector axis = toKdl(sdf->GetElement("axis")->Get<sdf::Vector3>());
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
                    I = sdfInertiaToKdl(sdf_link->GetElement("inertial"));
                }
            }

            JointsMap::iterator  joint_itr = joints.find(*childItr);

            if (joint_itr != joints.end()){
                sdf::ElementPtr sdf_joint = joint_itr->second;
                joint = sdfJointToKdl(sdf_joint);
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
LinkNamesMap sdfBuildLinkNames(LinksMap links, JointsMap joints, std::string rootName)
{

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
    KDL::Tree tree(modelName);
    fillKdlTreeFromSDF(linkNames, joints, links, tree.getRootSegment()->first, tree);
    out = tree;
}

/**
 * load kdl tree from sdf xml string
 */
bool treeFromSdfString(const std::string& xml, KDL::Tree& tree)
{
    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf)){
        LOG_ERROR("unable to initialize sdf.");
        return false;
    }

    if (!sdf::readString(xml, sdf)){
        LOG_ERROR("unable to read xml string.");
        return false;
    }

    if (!sdf->root->HasElement("model")){
        LOG_ERROR("the <model> tag not exists");
        return false;
    }

    treeFromSdfModel(sdf->root->GetElement("model"), tree);

    return true;
}

/**
 * load kdl tree from sdf file
 */
bool treeFromSdfFile(const std::string& path, KDL::Tree& tree)
{
    std::ifstream file(path.c_str());
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return treeFromSdfString(str, tree);
}

}



