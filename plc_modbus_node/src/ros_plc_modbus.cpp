//
// Created by Brad Bazemore on 10/29/15.
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include "plc_modbus_node/UInt16Array.h"
#include "plc_modbus_node/ByteArray.h"
#include <modbus/modbus.h>

struct plc_modbus_addr {
    std::vector<int> coils_addr;   // coil addresses
    std::vector<int> regs_write_addr;  // register addresses for writing to
    std::vector<int> regs_read_addr;   // register addresses that are read-only

    plc_modbus_addr(const std::vector<int>& coils_addr, const std::vector<int>& regs_write_addr, const std::vector<int>& regs_read_addr)
    : coils_addr(coils_addr), regs_write_addr(regs_write_addr), regs_read_addr(regs_read_addr) {}

    std::string to_string() {
        std::stringstream ss;
        ss << "([";
        for (int i = 0; i < coils_addr.size(); ++i)
            ss << coils_addr[i] << ",";
        ss << "], [";
        for (int i = 0; i < regs_write_addr.size(); ++i)
            ss << regs_write_addr[i] << ",";
        ss << "], [";
        for (int i = 0; i < regs_read_addr.size(); ++i)
            ss << regs_read_addr[i] << ",";
        ss << "])";
        return ss.str();
    }
};

class plc_modbus_manager {
public:
    plc_modbus_manager();

private:
    ros::NodeHandle node;

    ros::Publisher regs_read;
    ros::Subscriber regs_write;
    ros::Publisher coils_read;
    ros::Subscriber coils_write;

    std::vector<int> coils_addrs;   // coil addresses
    std::vector<int> regs_addrs;  // register addresses

    std_msgs::UInt16MultiArray regs_val;
    std_msgs::ByteMultiArray coils_val;

    std::map<std::string, plc_modbus_addr> plc_addresses;

    modbus_t *plc;

    std::string ip_address;
    int port;
    int spin_rate;

    void regs_callBack(const plc_modbus_node::UInt16Array::ConstPtr &regs_data);

    void coils_callBack(const plc_modbus_node::ByteArray::ConstPtr &coils_data);
};

plc_modbus_manager::plc_modbus_manager() {

    // parse parameters according to the types of components/senders that will communicate with the PLC
    std::string types_str;
    node.param<std::string>("plc_modbus_node/types", types_str, "forklift,roboteq,");
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    // tokenize string containing the list of components/senders
    while ((pos = types_str.find(delimiter)) != std::string::npos) {
        // tokenize
        token = types_str.substr(0, pos);
        std::cout << token << std::endl;

        // parse parameters for the arrays of addresses
        std::vector<int> coils_addr;
        std::vector<int> regs_write_addr;
        std::vector<int> regs_read_addr;
        // parse coils addresses
        if (!node.getParam("plc_modbus_node/" + token + "_coils_addr", coils_addr)) {
            ROS_WARN(token.c_str());
            ROS_WARN("No coil addrs given!");
        }
        // parse reg addresses (write)
        if (!node.getParam("plc_modbus_node/" + token + "_regs_write_addr", regs_write_addr)) {
            ROS_WARN(token.c_str());
            ROS_WARN("No reg write addrs given!");
        }
        // parse reg addresses (read-only)
        if (!node.getParam("plc_modbus_node/" + token + "_regs_read_addr", regs_read_addr)) {
            ROS_WARN(token.c_str());
            ROS_WARN("No reg read addrs given!");
        }
        // add to map
        plc_addresses.emplace(token, plc_modbus_addr(coils_addr, regs_write_addr, regs_read_addr));
        // add to lists
        coils_addrs.insert(coils_addrs.end(), coils_addr.begin(), coils_addr.end());
        regs_addrs.insert(regs_addrs.end(), regs_write_addr.begin(), regs_write_addr.end());
        regs_addrs.insert(regs_addrs.end(), regs_read_addr.begin(), regs_read_addr.end());

        // erase token to move on to next
        types_str.erase(0, pos + delimiter.length());
    }

    // declare pub/sub topics
    regs_read = node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_read", 100);
    regs_write = node.subscribe<plc_modbus_node::UInt16Array>("modbus/regs_write", 100,
                                                            &plc_modbus_manager::regs_callBack, this);
    coils_read = node.advertise<std_msgs::ByteMultiArray>("modbus/coils_read", 100);
    coils_write = node.subscribe<plc_modbus_node::ByteArray>("modbus/coils_write", 100,
                                                           &plc_modbus_manager::coils_callBack, this);

    // get parameters for modbus connection
    node.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.0.100");
    node.param("plc_modbus_node/port", port, 502);
    node.param("plc_modbus_node/spin_rate",spin_rate,30);

    ROS_INFO("NOTE: SKIPPED CONNECTING TO MODBUS DEVICE FOR SOFTWARE-ONLY TESTING; UNCOMMENT BELOW FOR ACTUAL USE");
    /*ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
    plc = modbus_new_tcp(ip_address.c_str(), port);
    if (plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }
    if (modbus_connect(plc) == -1) {
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(plc);
        return;
    } else {
        ROS_INFO("Connection to modbus device established");
    }*/

    // keep looping to prevent ros node from exiting
    // and to regularly publish the data in the PLC registers/coils
    ros::Rate loop_rate(spin_rate);
    while (ros::ok()) {
        // clear data to read reg/coil values again
        regs_val.data.clear();
        coils_val.data.clear();

        // read values from reg addresses, one value (uint16) at a time
        for (int i = 0; i < regs_addrs.size(); i++) {
            uint16_t temp[1] = {0};
            // read value at address
            if (modbus_read_registers(plc, regs_addrs.at(i), 1, temp) == -1) {  // error reading from register
                ROS_ERROR("Unable to read reg addr:%d", regs_addrs.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {    // read successfully
                regs_val.data.push_back(temp[0]);
            }
        }
        // publish if there is data available
        // NOTE: if there is error reading from one address but not others, it will be skipped over in this array
        if (regs_val.data.size() > 0) {
            regs_read.publish(regs_val);
        }

        // read values from coil addresses, one value (byte) at a time
        for (int i = 0; i < coils_addrs.size(); i++) {
            uint8_t temp[1] = {0};
            // read value at address
            if (modbus_read_bits(plc, coils_addrs.at(i), 1, temp) == -1) {  // error reading from coil
                ROS_ERROR("Unable to read coil addr:%d", coils_addrs.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {    // read successfuully
                coils_val.data.push_back(temp[0]);
            }
        }
        // publish if there is data available
        // NOTE: if there is error reading from one address but not others, it will be skipped over in this array
        if (coils_val.data.size() > 0) {
            coils_read.publish(coils_val);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    modbus_close(plc);
    modbus_free(plc);
    return;
}

void plc_modbus_manager::regs_callBack(const plc_modbus_node::UInt16Array::ConstPtr &regs_data) {

    // iterate through info on PLC addresses to find the one that corresponds to sender
    std::map<std::string, plc_modbus_addr>::iterator it;
    for (it = plc_addresses.begin(); it != plc_addresses.end(); ++it) {
        // find the addresses that corresponds to the node that sent it
        if (regs_data->name.compare(it->first)) {   // key found
                // check that length of regs addresses to write to is the same
            if (regs_data->data.size() != it->second.regs_write_addr.size()) {
                ROS_ERROR("%d registers to write but only %d given!", it->second.regs_write_addr.size(), regs_data->data.size());
                return;
            }
            // write to regs addresses, one value (uint16) at a time
            for (int i = 0; i < regs_data->data.size(); i++) {
                // get data value
                ROS_DEBUG("regs_out[%d]:%u", i, regs_data->data.at(i));
                uint16_t temp[1] = {regs_data->data.at(i)};
                // write value to reg address
                if (modbus_write_registers(plc, it->second.regs_write_addr.at(i), 1, temp) == -1) { // error writing to reg
                    ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", it->second.regs_write_addr.at(i), regs_data->data.at(i));
                    ROS_ERROR("%s", modbus_strerror(errno));
                } else {    // written successfully
                    ROS_INFO("Modbus register write at addr:%d with value:%u", it->second.regs_write_addr.at(i), regs_data->data.at(i));
                }
            }
            break;
        }
    }
}

void plc_modbus_manager::coils_callBack(const plc_modbus_node::ByteArray::ConstPtr &coils_data) {

    // iterate through info on PLC addresses to find the one that corresponds to sender
    std::map<std::string, plc_modbus_addr>::iterator it;
    for (it = plc_addresses.begin(); it != plc_addresses.end(); ++it) {
        // find the plc addresses that corresponds to the node that sent it
        if (coils_data->name.compare(it->first)) {   // key found
                // check that length of coils addresses to write to is the same
                if (coils_data->data.size() != it->second.coils_addr.size()) {
                ROS_ERROR("%d coils to write but %d given!", it->second.coils_addr.size(), coils_data->data.size());
                return;
            }
            // write to coils addresses, one value (byte) at a time
            for (int i = 0; i < coils_data->data.size(); i++) {
                // get data value
                ROS_DEBUG("regs_out[%d]:%u", i, coils_data->data.at(i));
                uint8_t temp[1] = {coils_data->data.at(i)};
                // write value to coil address
                if (modbus_write_bits(plc, it->second.coils_addr.at(i), 1, temp) == -1) {   // error writing to coil
                    ROS_ERROR("Modbus coil write failed at addr:%d with value:%u", it->second.coils_addr.at(i), coils_data->data.at(i));
                    ROS_ERROR("%s", modbus_strerror(errno));
                } else {    // written successfully
                    ROS_INFO("Modbus coil write at addr:%d with value:%u", it->second.coils_addr.at(i), coils_data->data.at(i));
                }
            }
            break;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_plc_modbus");
    plc_modbus_manager mm;
    return 0;
}
