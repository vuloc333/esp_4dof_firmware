#ifndef MODCOM_H
#define MODCOM_H

#include "ModbusReg.h"
#include "ModbusRTU.h"

#define SLAVE_ID 1

class Modcom {
    private:
        ModbusReg& _data;
        int _port;
        ModbusRTU _mb;
    public:
        Modcom(ModbusReg& data, int port);
        void begin();
        void poll();
};
#endif