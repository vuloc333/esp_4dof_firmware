#include "ModbusReg.h"
#include "Modbus.h"

#define SLAVE_ID 1

class ModCom {
    private:
        ModbusReg _data;

    public:
        ModCom();
        void begin();
        void poll();
        ModbusReg reg;
};