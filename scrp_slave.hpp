#ifndef SCRP_SLAVE_H
#define SCRP_SLAVE_H
#include "mbed.h"

/*USBでPCにつなぐポートと、基板上でRasPiとつなぐポートを同時に開く。
 *RedePinの有り無しの選択、ポートを一つだけ開くことも可。
 *以下から選択。
 *ScrpSlave(PinName TX1,PinName RX1,uint32_t addr);//RedePinなし、１ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr);//RedePinあり、１ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr);//RedePinなし、２ポート
 *ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr);//RedePinあり、１ポート＋RedePinなし、１ポート
 *example not usb port
 *L432KC : TX = PA_9 , RX = PA_10 , REDE = PA_12 , addr = 0x0803e000
 *F446RE : TX = PC_12 , RX = PD_2 , RDDE = PH_1 , addr = 0x0807ffff
 *H743ZI : TX = ,RX = ,REDE = , addr = 0x081ee000
 *<obj>.addCMD(int cmd, bool (*proc)(int rx_data, int& tx_data))
 *でcmdで指定したコマンドを受信したときに呼び出される
 *bool型で引数が(int rx_data, int& tx_data)の関数を指定する。
 */
//ScrpSlave slave(SERIAL_TX,SERIAL_RX);
//ScrpSlave slave(PC_12,PD_2 ,PH_1 ,SERIAL_TX,SERIAL_RX,0x0807ffff);
//ScrpSlave slave(PA_9 ,PA_10,PA_12,SERIAL_TX,SERIAL_RX,0x0803e000);

inline int constrain(int x,int a,int b){
    return (x < a ? a : x > b ? b : x);
}

inline double constrain(double x,double a,double b){
    return (x < a ? a : x > b ? b : x);
}

class ScrpSlave{
public:
    ScrpSlave(PinName TX1,PinName RX1,uint32_t addr);//RedePinなし、１ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr);//RedePinあり、１ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr);//RedePinなし、２ポート
    ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr);//RedePinあり、１ポート＋RedePinなし、１ポート
    ~ScrpSlave();
    void setTimeout(int);
    void addCMD(uint8_t cmd, bool (*proc)(int rx_data,int& tx_data));
    int16_t send(uint8_t id,uint8_t cmd,int16_t tx_data);
    int16_t send2(uint8_t id,uint8_t cmd,int16_t tx_data);
private:
    DigitalOut *rede_;
    Serial *serial_[2];
    FlashIAP *flash_;
    uint8_t send_data_[2][8];
    uint8_t mode_;
    uint8_t my_id_;
    uint32_t address_;
    int timeout_;
    bool wait_data_[2];
    bool stx_flag_[2];
    bool id_ok_[2];
    uint8_t tmp_data_[2][5];
    uint8_t data_count_[2];
    int16_t rx_data_[2];
    bool (*procs_[256])(int rx_data, int& tx_data);
    int16_t sending(int,uint8_t,uint8_t,int16_t);
    void changeID(uint8_t);
    void init(PinName,PinName);
    void check(int port);
    void port1();
    void port2();
    void data_send1();
    void data_send2();
    void prime(int);
};

#endif /* SCRP_SLAVE_H */