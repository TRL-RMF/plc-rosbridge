// Wrapper for modbus communication

#include <ros/ros.h>
#include "test_modbus/modbus_mock.h"
#include <modbus/modbus.h>

struct modbus_wrapper {
    modbus_mock mock;
    modbus_t *plc;

    bool debug_mode;

    // Constructor for mock PLC modbus
    modbus_wrapper(const std::vector<int>& regs_addrs, const std::vector<int>& coils_addrs) : debug_mode(true), mock(regs_addrs, coils_addrs), plc(0) {}

    // Constructor for actual PLC communication wrapper
    modbus_wrapper(const char *ip_addr, int port) : debug_mode(false), mock(), plc(0) {
        plc = modbus_new_tcp(ip_addr, port);
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
        }
    }

    // Read regs
    bool read_registers(int addr, int len, uint16_t *out) {
        if (debug_mode)
            return mock.modbus_read_registers(addr, len, out);
        else
            return modbus_read_registers(plc, addr, len, out);
    }
    // Read coils
    bool read_bits(int addr, int len, uint8_t *out) {
        if (debug_mode)
            return mock.modbus_read_bits(addr, len, out);
        else
            return modbus_read_bits(plc, addr, len, out);
    }

    // Write to regs
    bool write_registers(int addr, int len, uint16_t *in) {
        if (debug_mode)
            return mock.modbus_write_registers(addr, len, in);
        else
            return modbus_write_registers(plc, addr, len, in);
    }
    // Write to coils
    bool write_bits(int addr, int len, uint8_t *in) {
        if (debug_mode)
            return mock.modbus_write_bits(addr, len, in);
        else
            return modbus_write_bits(plc, addr, len, in);
    }

    // error message
    const char* strerror() {
        if (debug_mode)
            return "Invalid Address";
        else
            return modbus_strerror(errno);
    }

    void close() {
        // close the connection if any
        if (!debug_mode) {
            // close the actual connection
            modbus_close(plc);
            modbus_free(plc);
        }
    }
};