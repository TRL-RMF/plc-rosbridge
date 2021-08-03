//
// Created by Brad Bazemore on 10/29/15.
// Modified for multiple nodes
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include "plc_modbus_node/UInt16Array.h"
#include "plc_modbus_node/MultiUInt16Array.h"
#include "plc_modbus_node/ByteArray.h"
#include "plc_modbus_node/MultiByteArray.h"

#include "modbus_wrapper.h"
//#include "test_modbus/test_modbus.h"
//#include <modbus/modbus.h>

// data type for coil/reg addresses for a component
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

    plc_modbus_node::MultiByteArray coils_pub_data;
    plc_modbus_node::MultiUInt16Array regs_pub_data;

    std::map<std::string, plc_modbus_addr> plc_addresses;

    bool debug_mode;

    modbus_wrapper *modbus;    // selects mock PLC or actual PLC communcation wrapper

    std::string ip_address;
    int port;
    int spin_rate;

    void regs_callBack(const plc_modbus_node::UInt16Array::ConstPtr &regs_data);

    void coils_callBack(const plc_modbus_node::ByteArray::ConstPtr &coils_data);
};

plc_modbus_manager::plc_modbus_manager() {

    // for mock testing only
    std::vector<int> regs_addrs;  // register addresses
    std::vector<int> coils_addrs;   // coil addresses

    // parse debug mode parameter
    node.param<bool>("plc_modbus_node/debug", debug_mode, true);
    ROS_INFO("Debug mode: %s", debug_mode ? "true" : "false");

    // parse name of addresses list parameter from the yaml file
    std::string rosparam_plcaddrs;  // name of the rosparam for plc addresses loaded from yaml file
    node.param<std::string>("plc_modbus_node/addr_param_name", rosparam_plcaddrs, "plc_modbus_node/plc_addrs");
    // parse the list of addresses of the components/senders that will communicate with the PLC
    XmlRpc::XmlRpcValue list;
    node.param(rosparam_plcaddrs, list, list);

    // tokenize string containing the list of components/senders
    for (int i = 0; i < list.size(); ++i) {

        // store lists of addresses
        std::vector<int> coils_addr;
        std::vector<int> regs_write_addr;
        std::vector<int> regs_read_addr;

        // parse parameters for the arrays of addresses
        XmlRpc::XmlRpcValue coils_addr_sublist = list[i]["coils_addr"];
        ROS_ASSERT(coils_addr_sublist.getType() == XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue regs_write_addr_sublist = list[i]["regs_write_addr"];
        ROS_ASSERT(regs_write_addr_sublist.getType() == XmlRpc::XmlRpcValue::TypeArray);
        XmlRpc::XmlRpcValue regs_read_addr_sublist = list[i]["regs_read_addr"];
        ROS_ASSERT(regs_read_addr_sublist.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        // add to lists of addresses
        for (int j = 0; j < coils_addr_sublist.size(); ++j) {
            coils_addr.push_back(coils_addr_sublist[j]);
        }
        for (int j = 0; j < regs_write_addr_sublist.size(); ++j) {
            regs_write_addr.push_back(regs_write_addr_sublist[j]);
        }
        for (int j = 0; j < regs_read_addr_sublist.size(); ++j) {
            regs_read_addr.push_back(regs_read_addr_sublist[j]);
        }

        // add to map
        plc_addresses.emplace(list[i]["name"], plc_modbus_addr(coils_addr, regs_write_addr, regs_read_addr));
        
        // add to lists
        if (debug_mode) {
            regs_addrs.insert(regs_addrs.end(), regs_write_addr.begin(), regs_write_addr.end());
            regs_addrs.insert(regs_addrs.end(), regs_read_addr.begin(), regs_read_addr.end());
            coils_addrs.insert(coils_addrs.end(), coils_addr.begin(), coils_addr.end());
        }
    }

    // declare pub/sub topics
    regs_read = node.advertise<plc_modbus_node::MultiUInt16Array>("modbus/regs_read", 100);
    regs_write = node.subscribe<plc_modbus_node::UInt16Array>("modbus/regs_write", 100,
                                                            &plc_modbus_manager::regs_callBack, this);
    coils_read = node.advertise<plc_modbus_node::MultiByteArray>("modbus/coils_read", 100);
    coils_write = node.subscribe<plc_modbus_node::ByteArray>("modbus/coils_write", 100,
                                                           &plc_modbus_manager::coils_callBack, this);

    // get parameters for ros
    node.param("plc_modbus_node/spin_rate",spin_rate, 30);

    if (debug_mode) {
        // Create mock PLC coils/registers address map
        ROS_INFO("NOTE: SKIPPED CONNECTING TO MODBUS DEVICE FOR SOFTWARE-ONLY TESTING; TURN OFF DEBUG FOR ACTUAL USE");
        modbus = new modbus_wrapper(regs_addrs, coils_addrs);
    }
    else {
        // get parameters for modbus connection
        node.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.0.100");
        node.param("plc_modbus_node/port", port, 502);

        // Create modbus/tcp connection to PLC
        ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
        modbus = new modbus_wrapper(ip_address.c_str(), port);
    }

    // keep looping to prevent ros node from exiting
    // and to regularly publish the data in the PLC registers/coils
    ros::Rate loop_rate(spin_rate);
    while (ros::ok()) {
        // clear data to read reg/coil values again
        coils_pub_data.arrays.clear();
        regs_pub_data.arrays.clear();

        std::map<std::string, plc_modbus_addr>::iterator it;
        for (it = plc_addresses.begin(); it != plc_addresses.end(); ++it) {
            plc_modbus_node::ByteArray coils_data;
            plc_modbus_node::UInt16Array regs_data;

            coils_data.name = it->first;
            // append coils data
            for (int i = 0; i < it->second.coils_addr.size(); i++) {
                uint8_t temp[1] = {0};
                // read value at address
                if (modbus->read_bits(it->second.coils_addr.at(i), 1, temp) == -1) {  // error reading from coil
                    ROS_ERROR("Unable to read coil addr:%d", it->second.coils_addr.at(i));
                    ROS_ERROR("%s", modbus->strerror());
                } else {    // read successfuully
                    coils_data.data.push_back(temp[0]);
                }
            }

            regs_data.name = it->first;
            // append reg write data
            for (int i = 0; i < it->second.regs_write_addr.size(); i++) {
                uint16_t temp[1] = {0};
                // read value at address
                if (modbus->read_registers(it->second.regs_write_addr.at(i), 1, temp) == -1) {  // error reading from register
                    ROS_ERROR("Unable to read reg addr:%d", it->second.regs_write_addr.at(i));
                    ROS_ERROR("%s", modbus->strerror());
                } else {    // read successfully
                    regs_data.data.push_back(temp[0]);
                }
            }
            // append reg read data
            for (int i = 0; i < it->second.regs_read_addr.size(); i++) {
                uint16_t temp[1] = {0};
                // read value at address
                if (modbus->read_registers(it->second.regs_read_addr.at(i), 1, temp) == -1) {  // error reading from register
                    ROS_ERROR("Unable to read reg addr:%d", it->second.regs_read_addr.at(i));
                    ROS_ERROR("%s", modbus->strerror());
                } else {    // read successfully
                    regs_data.data.push_back(temp[0]);
                }
            }
            // push
            coils_pub_data.arrays.push_back(coils_data);
            regs_pub_data.arrays.push_back(regs_data);
        }
        // publish
        if (coils_pub_data.arrays.size() > 0) {
            coils_read.publish(coils_pub_data);
        }
        if (regs_pub_data.arrays.size() > 0) {
            regs_read.publish(regs_pub_data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // terminate the connection
    modbus->close();
    return;
}

void plc_modbus_manager::regs_callBack(const plc_modbus_node::UInt16Array::ConstPtr &regs_data) {

    // iterate through info on PLC addresses to find the one that corresponds to sender
    std::map<std::string, plc_modbus_addr>::iterator it;
    for (it = plc_addresses.begin(); it != plc_addresses.end(); ++it) {
        // find the addresses that corresponds to the node that sent it
        if (regs_data->name.compare(it->first) == 0) {   // key found
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
                if (modbus->write_registers(it->second.regs_write_addr.at(i), 1, temp) == -1) { // error writing to reg
                    ROS_ERROR("Modbus reg write failed at addr:%d with value:%u", it->second.regs_write_addr.at(i), regs_data->data.at(i));
                    ROS_ERROR("%s", modbus->strerror());
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
        if (coils_data->name.compare(it->first) == 0) {   // key found
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
                if (modbus->write_bits(it->second.coils_addr.at(i), 1, temp) == -1) {   // error writing to coil
                    ROS_ERROR("Modbus coil write failed at addr:%d with value:%u", it->second.coils_addr.at(i), coils_data->data.at(i));
                    ROS_ERROR("%s", modbus->strerror());
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
