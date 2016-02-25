#include <object_moveit/ObjectMessageGenerator.h>

#include <iostream>

#define DEFAULT_COLLISION_OBJECT_TOPIC "collision_object"
#define DEFAULT_OBJECTS_TOPIC "world/objects"
#define DEFAULT_REQUEST_OBJECTS_TOPIC "world/request_object"
#define DEFAULT_GET_PLANNING_SCENE_SERVICE "/get_planning_scene"
#define DEFAULT_SET_PLANNING_SCENE_TOPIC "/planning_scene"
#define DEFAULT_USE_PLANNING_SCENE_DIFF true
    
#define DEFAULT_PUBLISH_COLLISION_RATE 30

using object_moveit::ObjectMessageGenerator;

ObjectMessageGenerator::ObjectMessageGenerator(ros::NodeHandle& _node_priv, ros::NodeHandle& _node_pub) :
    node_priv(_node_priv),
    node(_node_pub),
    acmManip(_node_priv){

    ros::NodeHandle _node("/object_moveit");
    _node.param<std::string>("world_objects_topic", OBJECTS_TOPIC, DEFAULT_OBJECTS_TOPIC);
    ROS_INFO("Got objects topic name: <%s>", OBJECTS_TOPIC.c_str());

    _node.param<std::string>("request_object_service", REQUEST_OBJECTS_TOPIC, DEFAULT_REQUEST_OBJECTS_TOPIC);
    ROS_INFO("Got object service topic name: <%s>", REQUEST_OBJECTS_TOPIC.c_str());
    
    _node.param<std::string>("collision_object_topic", COLLISION_OBJECT_TOPIC, DEFAULT_COLLISION_OBJECT_TOPIC);
    ROS_INFO("Got collision objects topic name: <%s>", COLLISION_OBJECT_TOPIC.c_str());
    
    GET_PLANNING_SCENE_SERVICE= DEFAULT_GET_PLANNING_SCENE_SERVICE;
    _node.param<std::string>("moveit_get_planning_scene_topic", GET_PLANNING_SCENE_SERVICE,GET_PLANNING_SCENE_SERVICE);
    ROS_INFO("Got moveit_get_planning_scene_topic: <%s>", GET_PLANNING_SCENE_SERVICE.c_str());
   

    SET_PLANNING_SCENE_TOPIC= DEFAULT_SET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_set_planning_scene_topic", SET_PLANNING_SCENE_TOPIC,SET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_set_planning_scene_topic: <%s>", SET_PLANNING_SCENE_TOPIC.c_str());


    _node.param<bool>("use_planning_scene_diff", USE_PLANNING_SCENE_DIFF, DEFAULT_USE_PLANNING_SCENE_DIFF);
    ROS_INFO("Got use_planning_scene_diff: <%i>", USE_PLANNING_SCENE_DIFF);

    std::stringstream def_coll_rate;
    def_coll_rate<<DEFAULT_PUBLISH_COLLISION_RATE;    
    std::string _PUBLISH_COLLISION_RATE=def_coll_rate.str();
    _node.param<std::string>("publish_collision_rate", _PUBLISH_COLLISION_RATE, _PUBLISH_COLLISION_RATE);
    PUBLISH_COLLISION_RATE=atof(_PUBLISH_COLLISION_RATE.c_str());

    std::string skip_string;        
    _node.param<std::string>("skip_objects", skip_string, "");
    ROS_INFO("Objects to skip: %s",skip_string.c_str());
    char * str=(char*)skip_string.c_str();
    char * pch = strtok(str," ,;");
    while (pch != NULL) {
        //ROS_INFO("%s\n",pch);
        skipObjects.insert(std::string(pch));
        pch = strtok (NULL, " ,;");
    }

    std::string allowed_coll_string;        
    _node.param<std::string>("allowed_collision_links", allowed_coll_string, "");
    ROS_INFO("Objects to allow collide: %s",allowed_coll_string.c_str());
    str=(char*)allowed_coll_string.c_str();
    pch = strtok(str," ,;");
    while (pch != NULL) {
        //ROS_INFO("%s\n",pch);
        allowedCollisionLinks.push_back(std::string(pch));
        pch = strtok (NULL, " ,;");
    }


    if (REQUEST_OBJECTS_TOPIC!="") object_info_client = node.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_TOPIC);
    planning_scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);

    object_sub = node.subscribe(OBJECTS_TOPIC, 100, &ObjectMessageGenerator::receiveObject,this);

    ros::SubscriberStatusCallback conn=boost::bind(&ObjectMessageGenerator::connectPub, this, _1);    
    ros::SubscriberStatusCallback disconn=boost::bind(&ObjectMessageGenerator::disconnectPub, this, _1);    
    if (!USE_PLANNING_SCENE_DIFF)
        collision_pub = node.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC, 100, conn, disconn); 
    else
        planning_scene_pub = node.advertise<moveit_msgs::PlanningScene>(SET_PLANNING_SCENE_TOPIC, 100, conn, disconn);
    ros::Rate rate(PUBLISH_COLLISION_RATE);
    publishCollisionsTimer=node_priv.createTimer(rate,&ObjectMessageGenerator::publishCollisionsEvent, this);

    initExistingObj=false;

}

ObjectMessageGenerator::~ObjectMessageGenerator() {
}

void ObjectMessageGenerator::connectPub(const ros::SingleSubscriberPublisher& p){
    //ROS_INFO("ObjectMessageGenerator: subscriber CONNECTING");

    mutex.lock();
    // get all collision objects currently in the scene
    addedObjects=getCurrentCollisionObjectNames();

    // also add the always allowed collision links for each object:
    for (std::set<std::string>::iterator it=addedObjects.begin();
            it!=addedObjects.end(); ++it)
    {
        acmManip.addAllowedMoveItCollision(*it,allowedCollisionLinks);
    }
    mutex.unlock();
    initExistingObj=true;
}

bool ObjectMessageGenerator::isConnected() const
{
    bool connected=false;
    if (!USE_PLANNING_SCENE_DIFF)
        connected = (collision_pub.getNumSubscribers() > 0);
    else
        connected = (planning_scene_pub.getNumSubscribers() > 0);
    return connected; 
}

void ObjectMessageGenerator::disconnectPub(const ros::SingleSubscriberPublisher& p){
    //ROS_INFO("ObjectMessageGenerator: a subscriber is DISCONNECTING");
    if (!isConnected())
    {   // lost connection to subscribers, so require new initialisation
        initExistingObj=false; 
    }
}

void ObjectMessageGenerator::publishCollisionsEvent(const ros::TimerEvent& e)
{
    if (!isConnected()) return;

    mutex.lock();    
    ObjToPublishMap::iterator it;
    for (it=objsToPublish.begin(); it!=objsToPublish.end(); ++it) {
        //ROS_INFO_STREAM("ObjectMessagGenerator: Publishing "<<it->second);
        if (!USE_PLANNING_SCENE_DIFF)
        {
            collision_pub.publish(it->second);
        }
        else
        {
            moveit_msgs::PlanningScene planning_scene;
            planning_scene.world.collision_objects.push_back(it->second);
            planning_scene.is_diff = true;
            planning_scene_pub.publish(planning_scene);
        }
    }
    objsToPublish.clear();
    mutex.unlock();    
}


void ObjectMessageGenerator::receiveObject(const ObjectMsg& msg){
    if (!isConnected())
    {   // lost connection to subscribers, so require new initialisation
        // ROS_INFO("ObjectMessageGenerator: No subscribers");
        initExistingObj=false;
        return;    
    }

    if (!initExistingObj) {
        ROS_WARN("ObjectMessageGenerator: No initialisation of objects yet.");
        return; //the existing objects haven't been initialised yet
    }

    // ROS_INFO_STREAM("ObjectMessageGenerator: Received object message to re-map to moveit collision objects.");//: "<<msg);
    
    std::string id=msg.name;
    if (skipObjects.find(id)!=skipObjects.end()) 
    {
        // this is an object to be skipped
        return; 
    }

    mutex.lock();    
    ObjToPublishMap::iterator existing=objsToPublish.find(id);
    if (existing!=objsToPublish.end())
    {   //we already have this object to publish, so only allow 
        //to overwrite the poses
        updatePose(msg,existing->second);
        mutex.unlock();    
        return;
    }

    moveit_msgs::CollisionObject obj;
    if (addedObjects.find(id)==addedObjects.end())
    {   // this is a new object
        if (msg.content==object_msgs::Object::POSE) {
            ROS_INFO("Object not added yet to the system, so retreiving object geometry... %s",__FILE__);    
            obj=getCollisionGeometry(id);
        } else {
            obj=transferContent(msg,false);
        }
        addedObjects.insert(id);
        acmManip.addAllowedMoveItCollision(id,allowedCollisionLinks);
    }
    else
    {   // We already have had geometry sent for this object. We may want to enforce a MOVE operation.
        // Only if geometry is actually specified in the message, we'll consider another ADD.
        if ((msg.content==object_msgs::Object::SHAPE) && !msg.primitives.empty()){
            obj=transferContent(msg,false);
        }else{
            if (msg.content==object_msgs::Object::SHAPE) 
                ROS_WARN("We already have had geometry sent for object '%s', so enforcing a MOVE operation.",msg.name.c_str());
            obj=transferContent(msg,true);
        }
    }

    /*if (obj.operation==moveit_msgs::CollisionObject::ADD) {
        ROS_INFO("ObjectMessageGenerator: Sending operation ADD for %s",obj.id.c_str()); //ROS_INFO_STREAM(obj);
    } else {
        ROS_INFO("ObjectMessageGenerator: Sending operation MOVE for %s",obj.id.c_str()); //ROS_INFO_STREAM(obj);
    }*/

    objsToPublish.insert(std::make_pair(id,obj));

    mutex.unlock();    
}

moveit_msgs::CollisionObject ObjectMessageGenerator::getCollisionGeometry(const std::string& name){
    ROS_INFO("Received information for new object %s, adding it by requesting mesh information...",name.c_str());
    /*while (!object_info_client.exists() && !object_info_client.waitForExistence(ros::Duration(1))) {
        ROS_INFO("ObjectMessageGenerator: Waiting for planning scene service (topic %s) to become available...",GET_PLANNING_SCENE_SERVICE.c_str());
    }*/
    object_msgs::ObjectInfo srv;
    srv.request.name=name;
    if (!object_info_client.call(srv)){
        ROS_ERROR("Could not add object %s because service request failed.",name.c_str());
        return moveit_msgs::CollisionObject();
    }
    ROS_INFO("Object added.");
    return transferContent(srv.response.object,false);
}

void ObjectMessageGenerator::updatePose(const ObjectMsg& newObj, moveit_msgs::CollisionObject& obj){
    if (obj.header.frame_id!=newObj.header.frame_id) ROS_WARN("messages not specified in same frame! %s %s",obj.header.frame_id.c_str(),newObj.header.frame_id.c_str());
    if (obj.id != newObj.name) ROS_ERROR("Not referring to same object to update the pose!");

    obj.header=newObj.header;
    obj.primitive_poses=newObj.primitive_poses;
    obj.mesh_poses=newObj.mesh_poses;
}


moveit_msgs::CollisionObject ObjectMessageGenerator::transferContent(const ObjectMsg& msg, bool skipGeometry){

    moveit_msgs::CollisionObject obj;

    obj.header=msg.header;

    obj.id=msg.name;
    //obj.type.key="Box";
    //obj.type.db="";

    if (!skipGeometry) obj.primitives=msg.primitives;
    obj.primitive_poses=msg.primitive_poses;
    
    if (!skipGeometry) obj.meshes=msg.meshes;
    obj.mesh_poses=msg.mesh_poses;

    if ((msg.content==ObjectMsg::POSE) || skipGeometry) {
        obj.operation=moveit_msgs::CollisionObject::MOVE;
    } else if (msg.content==ObjectMsg::SHAPE) {    
        //ROS_INFO("ObjectMessageGenerator: Sending operation ADD for %s",obj.id.c_str()); //ROS_INFO_STREAM(obj);
        obj.operation=moveit_msgs::CollisionObject::ADD;
        //obj.operation=moveit_msgs::CollisionObject::APPEND;
        //obj.operation=moveit_msgs::CollisionObject::REMOVE;
    }

    return obj;    

}


std::vector<moveit_msgs::CollisionObject> ObjectMessageGenerator::getCurrentCollisionObjects(bool only_names) {

    if (!planning_scene_client.exists()) return std::vector<moveit_msgs::CollisionObject>();

    /*while (!planning_scene_client.exists() && !planning_scene_client.waitForExistence(ros::Duration(1))) {
        ROS_INFO("Waiting for planning scene service (topic %s) to become available...",GET_PLANNING_SCENE_SERVICE.c_str());
    }*/

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components=
        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    if (!only_names) 
        srv.request.components.components= 
            srv.request.components.components |
            moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    
    if (!planning_scene_client.call(srv)) {
        ROS_ERROR("Can't obtain planning scene");
        return std::vector<moveit_msgs::CollisionObject>();
    }

    moveit_msgs::PlanningScene& scene=srv.response.scene;
    return scene.world.collision_objects;
}



std::set<std::string> ObjectMessageGenerator::getCurrentCollisionObjectNames() {
    std::vector<moveit_msgs::CollisionObject> obj=getCurrentCollisionObjects();
    std::set<std::string> ret;
    for (std::vector<moveit_msgs::CollisionObject>::iterator it=obj.begin(); it!=obj.end(); ++it ) {
        ret.insert(it->id);
    }
    return ret;
}

