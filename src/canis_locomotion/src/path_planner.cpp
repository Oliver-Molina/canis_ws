/*

void GaitPlanner::Path_CB(const PathQuat::ConstPtr& path) {
    this->path = (*path).poses; 
    std::queue<Gait> empty;
    std::vector<Gait> gait_path = calculatePath();
    for (Gait gait : gait_path) {
        empty.push(gait);
    }
    std::swap(gait_queue, empty);
    gait_pub.publish(gait_queue.front());
    gait_queue.pop();
}

nav_msgs::Odometry translate(nav_msgs::Odometry end, nav_msgs::Odometry start) {
    nav_msgs::Odometry translate_end = end;

    double x = translate_end.pose.pose.position.x - start.pose.pose.position.x;
    double y = translate_end.pose.pose.position.y - start.pose.pose.position.y;
    double z = translate_end.pose.pose.position.z - start.pose.pose.position.z;

    tf2::Quaternion quat_i, quat_o;
    tf2::convert(start.pose.pose.orientation, quat_i);
    tf2::convert(end.pose.pose.orientation, quat_o);


    tf2::Matrix3x3 m_i;
    m_i.setRotation(quat_i); 
    tf2::Vector3 out_pos_vec(x, y, z);
    out_pos_vec = m_i.inverse() * out_pos_vec;
    quat_o *= quat_i.inverse();

    translate_end.pose.pose.position.x = out_pos_vec.x();
    translate_end.pose.pose.position.y = out_pos_vec.y();
    translate_end.pose.pose.position.z = out_pos_vec.z();

    //translate_end.pose.pose.orientation = tf::createQuaternionFromMsg(quat_o);
    tf2::convert(quat_o, translate_end.pose.pose.orientation);
    return translate_end;

}

std::vector<Gait> GaitPlanner::pathCommand(nav_msgs::Odometry end, nav_msgs::Odometry start) {
    auto translated_end = translate(end, start);
    auto dist = translated_end.pose.pose.position.x;
    auto rad = translated_end.twist.twist.angular.z;
    if (dist == 0 && rad != 0) {
        return turn(rad);
    }
    else if (dist != 0 && rad == 0) {
        return walk(dist);
    }
    std::vector<Gait> empty_vec;
    return empty_vec;
}

std::vector<Gait> GaitPlanner::calculatePath() {
    std::vector<Gait> gait_path;
    for (int path_index = 1; path_index < path.size(); path_index++) {
        std::vector<Gait> temp = pathCommand(path[path_index], path[path_index - 1]);
        gait_path.insert(gait_path.end(), temp.begin(), temp.end());
    }
    
    return gait_path;
}


*/