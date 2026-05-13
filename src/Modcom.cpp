#include "Modcom.h"
#include "ModbusRTU.h"

Modcom::Modcom(ModbusReg& data, int port) : _data(data), _port(port) {}

void Modcom::begin() {
    switch (_port) {
        case 0:
            _mb.begin(&Serial);
            break;
        case 1:
            _mb.begin(&Serial1);
            break;
        default:
            _mb.begin(&Serial2);
    }
    _mb.slave(SLAVE_ID);
    _mb.addCoil(0, 0, incoil);
    _mb.addHreg(0, 0, inreg);
    _mb.addCoil(mOffset, 0, outcoil);
    _mb.addHreg(mOffset, 0, outreg);
}  

void Modcom::poll() {
    //Write Data
    for (int i = 0; i < outreg; i++)  _mb.Hreg(i + mOffset, _data.outregs[i]);

    for (int i = 0; i < outcoil; i++)   _mb.Coil(i + mOffset, _data.outcoils[i]);

    _mb.task();

    //Read Data
    for (int i = 0; i < inreg; i++) _data.inregs[i] = _mb.Hreg(i);

    for (int i = 0; i < incoil; i++) _data.incoils[i] = _mb.Coil(i);
  
}
