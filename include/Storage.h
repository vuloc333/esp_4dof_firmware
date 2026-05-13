#ifndef STORAGE_H
#define STORAGE_H

#include "Preferences.h"
#include "ModbusReg.h"

class StorageManager
{
    private:
        Preferences _prefs;
        const char* _namespace = "Parameters";
    public:
        void save(int16_t* Parameter, size_t size) {
            _prefs.begin(_namespace, false);
            _prefs.putBytes("backup", Parameter, size * sizeof(int16_t));
            _prefs.end();
        }

        void load(int16_t* Parameter, size_t size) {
            _prefs.begin(_namespace, true);
            _prefs.getBytes("backup", Parameter, size * sizeof(int16_t));
            _prefs.end();
        }
};

#endif