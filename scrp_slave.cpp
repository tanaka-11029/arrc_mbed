#include "scrp_slave.hpp"

#define STX 0x41
#define DMY 0xff

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,uint32_t addr):address_(addr){
    mode_ = 0;
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr):address_(addr){
    mode_ = 1;
    rede_ = new DigitalOut(REDE1,0);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr):address_(addr){
    mode_ = 2;
    serial_[1] = new Serial(TX2,RX2,115200);
    serial_[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr):address_(addr){
    mode_ = 3;
    rede_ = new DigitalOut(REDE1,0);
    serial_[1] = new Serial(TX2,RX2,115200);
    serial_[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

void ScrpSlave::init(PinName TX,PinName RX){
    timeout_ = 500;
    for(int i = 0;i<2;i++){
        wait_data_[i] = false;
        stx_flag_[i] = false;
        id_ok_[i] = false;
    }
    serial_[0] = new Serial(TX,RX,115200);
    serial_[0]->attach(callback(this,&ScrpSlave::port1),Serial::RxIrq);
    flash_ = new FlashIAP;
    if(flash_->init()==0){
        if(flash_->read(&my_id_,address_,1) != 0){
            send(222,222,222);
            my_id_ = 10;
        }
    }else{
        send(111,111,111);
        my_id_ = 10;
    }
    for(int i = 1;i<256;++i){
        procs_[i] = NULL;
    }
}

void ScrpSlave::port1(){
    check(0);
}

void ScrpSlave::port2(){
    check(1);
}

void ScrpSlave::addCMD(uint8_t cmd, bool (*proc)(int rx_data, int& tx_data)){
    if(cmd == 0 || cmd == 254 || cmd == 253)return;
    procs_[cmd] = proc;
}

void ScrpSlave::setTimeout(int time){
    timeout_ = time;
}

void ScrpSlave::changeID(uint8_t id){
    flash_->erase(address_,flash_->get_sector_size(address_));
    flash_->program(&id,address_,1);
}

int16_t ScrpSlave::send(uint8_t id,uint8_t cmd,int16_t tx_data){
    return sending(0,id,cmd,tx_data);
}

int16_t ScrpSlave::send2(uint8_t id,uint8_t cmd,int16_t tx_data){
    if(mode_ < 2)return -1;
    return sending(1,id,cmd,tx_data);
}

int16_t ScrpSlave::sending(int port,uint8_t id,uint8_t cmd,int16_t tx_data){
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = id + cmd + tx_dataL + tx_dataH;
    Timer out;

    const uint8_t data[8] = {DMY, STX, id, cmd, tx_dataL, tx_dataH, tx_sum, DMY};  
    if(!serial_[port]->writeable()){
        return -1;
    }
    wait_data_[port] = true;//データ返信待ち
    if(mode_%2 == 1 && port == 0){
        rede_->write(1);
    }
    for(int i = 0;i < 8;i++){
        serial_[port]->putc(data[i]);
        while(!serial_[port]->writeable());
    }
    if(mode_%2 == 1 && port == 0){
        rede_->write(0);
    }
    
    out.reset();
    out.start();
    while(wait_data_[port]){
        if(out.read_ms() > timeout_){
           rx_data_[port] = -1;
           wait_data_[port] = false;
        }
    }
    out.stop();
    return rx_data_[port];
}

void ScrpSlave::check(int port){
    if(id_ok_[port]){
        tmp_data_[port][data_count_[port]] = serial_[port]->getc();
        data_count_[port]++;
        if(data_count_[port] > 4){
            stx_flag_[port] = false;//通信フラグクリア
            id_ok_[port] = false;
            
            uint8_t sum = 0;
            for(int i = 0;i<4;i++){
                sum += tmp_data_[port][i];
            }
            if(sum != tmp_data_[port][4]){
                return;
            }
            rx_data_[port] = (int16_t)(tmp_data_[port][2] + ((int16_t)tmp_data_[port][3] << 8));
            if(wait_data_[port]){//データ返信待ち時
                wait_data_[port] = false;
                return;
            }
            uint8_t rx_cmd = tmp_data_[port][1];
            bool broadcast = (tmp_data_[port][0] == 255);
            
            int tx_data = rx_data_[port];
            if(rx_cmd == 0){//通信テスト
            }else if(rx_cmd == 254){//id変更
                uint8_t new_id = rx_data_[port];
                my_id_ = new_id;
                changeID(new_id);
            }else if(rx_cmd == 253){//id確認
                tx_data = my_id_;
                rx_cmd = 250;
                broadcast = false;
            }else if(procs_[rx_cmd] == NULL || !procs_[rx_cmd](rx_data_[port],tx_data)){
                return;
            }
            if(broadcast){
                return;
            }
            uint8_t tx_dataL = tx_data;
            uint8_t tx_dataH = tx_data >> 8;
            uint8_t tx_sum = my_id_ + rx_cmd + tx_dataL + tx_dataH;
    
            const uint8_t data[8] = {DMY, STX, my_id_, rx_cmd, tx_dataL, tx_dataH, tx_sum, DMY};
            memcpy(this->send_data_[port],data,8);
            prime(port);
        }
    }else if(stx_flag_[port]){
        uint8_t get_data = serial_[port]->getc();
        if(get_data == my_id_ || get_data == 255){
            id_ok_[port] = true;
            tmp_data_[port][0] = get_data;
            data_count_[port]++;
        }else{
            stx_flag_[port] = false;
        }
    }else if(serial_[port]->getc() == STX){
        stx_flag_[port] = true;
        data_count_[port] = 0;
        id_ok_[port] = wait_data_[port];//データ返信待ち時はidチェック無し
    }
    return;
}

void ScrpSlave::data_send1(){
    while(serial_[0]->writeable()){
        if(data_count_[0] < 8){
            serial_[0]->putc(send_data_[0][data_count_[0]++]);
        }else{
            serial_[0]->attach(NULL, Serial::TxIrq);
            if(mode_%2 == 1){
                rede_->write(0);
            }
            break;
        }
    }
}

void ScrpSlave::data_send2(){
    while(serial_[1]->writeable()){
        if(data_count_[1] < 8){
            serial_[1]->putc(send_data_[1][data_count_[1]++]);
        }else{
            serial_[1]->attach(NULL, Serial::TxIrq);
            break;
        }
    }
}

void ScrpSlave::prime(int port){
    serial_[port]->attach(NULL, Serial::TxIrq);
    data_count_[port] = 0;
    if(port == 0){       
        if(mode_%2 == 1){
            rede_->write(1);
        }
        data_send1();
        serial_[0]->attach(callback(this, &ScrpSlave::data_send1), Serial::TxIrq);
    }else{
        data_send2();
        serial_[1]->attach(callback(this, &ScrpSlave::data_send2), Serial::TxIrq);
    }
}

ScrpSlave::~ScrpSlave(){
    delete serial_[0];
    delete flash_;
    if(mode_%2 == 1){
        delete rede_;
    }
    if(mode_ >= 2){
        delete serial_[1];
    }
}