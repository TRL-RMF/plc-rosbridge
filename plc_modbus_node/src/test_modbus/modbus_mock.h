#ifndef TEST_MODBUS_H
#define TEST_MODBUS_H

#include <map>
#include <vector>

struct modbus_mock {
    std::map<int, uint16_t> regs;
    std::map<int, uint8_t> coils;

public:
    // Default constructor
    modbus_mock() {}

    modbus_mock(const std::vector<int>& regs_addrs, const std::vector<int>& coils_addrs) {
        // initialize regs addresses
        for (int i = 0; i < regs_addrs.size(); ++i) {
            regs[regs_addrs[i]] = 0;
        }
        // initialize coils addresses
        for (int i = 0; i < coils_addrs.size(); ++i) {
            coils[coils_addrs[i]] = 0;
        }
    }

    bool read_reg(int addr, uint16_t *out) {
        if (regs.find(addr) == regs.end()) {    // invalid address
            return -1;
        }
        *out = regs[addr];
        return 0;
    }
    bool read_coil(int addr, uint8_t *out) {
        if (coils.find(addr) == coils.end()) {    // invalid address
            return -1;
        }
        *out = coils[addr];
        return 0;
    }
    bool write_reg(int addr, uint16_t *in) {
        if (regs.find(addr) == regs.end()) {    // invalid address
            return -1;
        }
        regs[addr] = *in;
        return 0;
    }
    bool write_coil(int addr, uint8_t *in) {
        if (coils.find(addr) == coils.end()) {    // invalid address
            return -1;
        }
        coils[addr] = *in;
        return 0;
    }

    // Read regs
    bool modbus_read_registers(int addr, int len, uint16_t *out) {
        for (int i = 0; i < len; i++) {
            if (read_reg(addr+i, out+i) == -1)
                return -1;  // error
        }
        return 0;   // success
    }
    // Read coils
    bool modbus_read_bits(int addr, int len, uint8_t *out) {
        for (int i = 0; i < len; i++) {
            if (read_coil(addr+i, out+i) == -1)
                return -1;  // error
        }
        return 0;   // success
    }

    // Write to regs
    bool modbus_write_registers(int addr, int len, uint16_t *in) {
        for (int i = 0; i < len; i++) {
            if (write_reg(addr+i, in+i) == -1)
                return -1;  // error
        }
        return 0;   // success
    }
    // Write to coils
    bool modbus_write_bits(int addr, int len, uint8_t *in) {
        for (int i = 0; i < len; i++) {
            if (write_coil(addr+i, in+i) == -1)
                return -1;  // error
        }
        return 0;   // success
    }

    // error message
    char* modbus_strerror() {
        return "Invalid Address";
    }

};

#endif // TEST_MODBUS_H