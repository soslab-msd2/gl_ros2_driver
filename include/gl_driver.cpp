
#include "gl_driver.h"


#define PS1             0xC3
#define PS2             0x51
#define PS3             0xA1
#define PS4             0xF8
#define SM_SET          0
#define SM_GET          1
#define SM_STREAM       2
#define SM_ERROR        255
#define BI_PC2GL        0x21
#define BI_GL2PC        0x12
#define PE              0xC2

#define STATE_INIT      0
#define STATE_PS1       1
#define STATE_PS2       2
#define STATE_PS3       3
#define STATE_PS4       4
#define STATE_TL        5
#define STATE_PAYLOAD   6
#define STATE_CS        7

#define COMM_SERIAL     1
#define COMM_UDP        2


int recv_state = STATE_INIT;

uint8_t comm_type;
serial::Serial* serial_port;
int sockfd;
struct sockaddr_in gl_addr, pc_addr;

uint8_t read_cs;
uint8_t write_cs;

std::vector<uint8_t> send_packet;
std::vector<uint8_t> recv_packet;


std::vector<uint8_t> serial_num;

std::vector<std::vector<uint8_t>> lidar_data;
Gl::framedata_t frame_data_in;


//////////////////////////////////////////////////////////////
// Constructor and Deconstructor for GL Class
//////////////////////////////////////////////////////////////

Gl::Gl(std::string& gl_udp_ip, int gl_udp_port, int pc_udp_port)
{
    comm_type = COMM_UDP;

    if ( (sockfd=socket(AF_INET, SOCK_DGRAM, 0)) == -1 ) 
    { 
		perror("[ERROR] Socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 

	memset(&gl_addr, 0, sizeof(gl_addr)); 
	gl_addr.sin_family = AF_INET; 
	gl_addr.sin_port = htons(gl_udp_port); 
    const char * c = gl_udp_ip.c_str();
	gl_addr.sin_addr.s_addr = inet_addr(c);

    memset(&pc_addr, 0, sizeof(pc_addr)); 
	pc_addr.sin_family = AF_INET; 
	pc_addr.sin_port = htons(pc_udp_port); 
    pc_addr.sin_addr.s_addr = htonl( INADDR_ANY );

    if ( bind(sockfd,(struct sockaddr *)&pc_addr,sizeof(pc_addr)) == -1 ) 
    { 
		perror("[ERROR] Bind failed"); 
		exit(EXIT_FAILURE); 
	} 
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Socket START [" << sockfd << "]" << std::endl;

    th = std::thread(&Gl::ThreadCallBack,this);
}


Gl::Gl(std::string& gl_serial_name, uint32_t gl_serial_baudrate)
{
    comm_type = COMM_SERIAL;

    serial_port = new serial::Serial(gl_serial_name, gl_serial_baudrate, serial::Timeout::simpleTimeout(100));
    if(serial_port->isOpen()) std::cout << "GL Serial is opened." << std::endl;
    else std::cout << "[ERROR] GL Serial is not opened." << std::endl;

    th = std::thread(&Gl::ThreadCallBack,this);
}


Gl::~Gl()
{
    SetFrameDataEnable(false);
    thread_running = false;

    if(comm_type==COMM_SERIAL)
    {
        serial_port->close();
        delete serial_port;
    }
    else if(comm_type==COMM_UDP)
    {
        close(sockfd); 
    }

    std::cout << "Socket END" << std::endl;
}


//////////////////////////////////////////////////////////////
// Functions for Comm
//////////////////////////////////////////////////////////////

void read_cs_update(uint8_t data) { read_cs = read_cs^ (data&0xff); }
uint8_t read_cs_get() { return read_cs&0xff; }
void read_cs_clear() { read_cs = 0; }

void write_cs_update(uint8_t data) { write_cs = write_cs^ (data&0xff); }
uint8_t write_cs_get() { return write_cs&0xff; }
void write_cs_clear() { write_cs = 0; }

void SendPacket(std::vector<uint8_t>& send_packet)
{
    for(auto& i: send_packet) serial_port->write(&i, 1);
}

void write(uint8_t data)
{
    send_packet.push_back(data);
    write_cs_update(data);
}

void write_PS()
{   
    std::vector<uint8_t> PS = {PS1, PS2, PS3, PS4};

    for(auto& i: PS) write(i);
}

void WritePacket(uint8_t PI, uint8_t PL, uint8_t SM, uint8_t CAT0, uint8_t CAT1, const std::vector<uint8_t>& DTn)
{
    send_packet.clear();
    write_cs_clear();

    write_PS();

    uint16_t TL = DTn.size() + 14;
    uint8_t buff = TL&0xff;
    write(buff);
    buff = (TL>>8)&0xff;
    write(buff);

    write(PI);
    write(PL);
    write(SM);
    write(BI_PC2GL);
    write(CAT0);
    write(CAT1);

    for(auto i: DTn) write(i);

    write(PE);
    write(write_cs_get());

    if(comm_type==COMM_SERIAL) SendPacket(send_packet);
    else if(comm_type==COMM_UDP) sendto(sockfd, &send_packet[0], send_packet.size(), MSG_CONFIRM, (const struct sockaddr *) &gl_addr, sizeof(gl_addr));
}


void RecvPacketClear(void)
{
    read_cs_clear();
    recv_state = STATE_INIT;
    recv_packet.clear();
}

void FrameData(const std::vector<uint8_t>& recv_data, uint8_t PI, uint8_t PL, uint8_t SM)
{
    if(SM!=SM_STREAM || recv_data.size()==0) return;

    if(PI==0)
    {
        lidar_data.clear();
        lidar_data.push_back(recv_data);
    }
    else if(PI==lidar_data.size())
    {
        lidar_data.push_back(recv_data);
    }
    else
    {
        lidar_data.clear();
        return;
    }
    
    if(lidar_data.size()==PL)
    {
        if(lidar_data[0].size()<3)
        {
            lidar_data.clear();
            return;
        }

        std::vector<uint8_t> data;
        for(int i=0; i<lidar_data.size(); i++) std::copy(lidar_data[i].begin(), lidar_data[i].end(), std::back_inserter(data));

        uint16_t frame_data_size = data[0]&0xff;
        frame_data_size |= ((uint16_t)(data[1]&0xff))<<8;

        if(data.size()!=(frame_data_size*4+22))
        {
            lidar_data.clear();
            return;
        } 

        Gl::framedata_t frame_data;
        frame_data.distance.resize(frame_data_size);
        frame_data.pulse_width.resize(frame_data_size);
        frame_data.angle.resize(frame_data_size);
        for(size_t i=0; i<frame_data_size; i++)
        {
            uint16_t distance = data[i*4+2]&0xff;
            distance |= ((uint16_t)(data[i*4+3]&0xff))<<8;

            uint16_t pulse_width = data[i*4+4]&0xff;
            pulse_width |= ((uint16_t)(data[i*4+5]&0xff))<<8;

            frame_data.distance[i] = distance/1000.0;
            frame_data.pulse_width[i] = pulse_width;
            frame_data.angle[i] = i*180.0/(frame_data_size-1)*3.141592/180.0;
        }

        frame_data_in = frame_data;
        lidar_data.clear();
    }
}

void SerialNum(const std::vector<uint8_t>& recv_data, uint8_t PI, uint8_t PL, uint8_t SM)
{
    if(SM!=SM_GET || recv_data.size()==0) return;

    serial_num = recv_data;
}

void ParsingData(const std::vector<uint8_t>& recv_data, uint8_t PI, uint8_t PL, uint8_t SM, uint8_t BI, uint8_t CAT0, uint8_t CAT1)
{
    // std::cout << std::endl;
    // std::cout << "Recv Data" << std::endl;
    // std::cout << "PI = " << (int)PI << std::endl;
    // std::cout << "PL = " << (int)PL << std::endl;
    // std::cout << "SM = " << (int)SM << std::endl;
    // std::cout << "BI = " << (int)BI << std::endl;
    // std::cout << "CAT0 = " << (int)CAT0 << std::endl;
    // std::cout << "CAT1 = " << (int)CAT1 << std::endl;
    // std::cout << "DTL = " << recv_data.size() << std::endl;

    if(BI!=BI_GL2PC) return;

    if(CAT0==0x01 && CAT1==0x02) FrameData(recv_data, PI, PL, SM);
    else if(CAT0==0x02 && CAT1==0x0A) SerialNum(recv_data, PI, PL, SM);
}

void ParsingPayload(const std::vector<uint8_t>& recv_packet)
{
    std::vector<uint8_t> recv_data;

    uint16_t TL = recv_packet[0]&0xff;
    TL |= ((uint16_t)recv_packet[1])<<8;

    uint8_t PI = recv_packet[2]&0xff;
    uint8_t PL = recv_packet[3]&0xff;
    uint8_t SM = recv_packet[4]&0xff;
    uint8_t BI = recv_packet[5]&0xff;
    uint8_t CAT0 = recv_packet[6]&0xff;
    uint8_t CAT1 = recv_packet[7]&0xff;

    uint16_t DTL = TL - 14;

    recv_data.resize(DTL);
    for(int i=0; i<DTL; i++)
    {
        recv_data[i] = recv_packet[8+i];
    }

    ParsingData(recv_data, PI, PL, SM, BI, CAT0, CAT1);
}

void CheckPS(uint8_t data)
{
    if(recv_state==STATE_INIT && data==PS1)
    {
        RecvPacketClear();
        read_cs_update(data);
        recv_state = STATE_PS1;
        return;
    }
    else if(recv_state==STATE_PS1 && data==PS2)
    {
        read_cs_update(data);
        recv_state = STATE_PS2;
        return;
    }
    else if(recv_state==STATE_PS2 && data==PS3)
    {
        read_cs_update(data);
        recv_state = STATE_PS3;
        return;
    }
    else if(recv_state==STATE_PS3 && data==PS4)
    {
        read_cs_update(data);
        recv_state = STATE_PS4;
        return;
    }

    recv_state = STATE_INIT;
}

void AddPacketElement(uint8_t data)
{
    if(recv_state==STATE_INIT || recv_state==STATE_PS1 || recv_state==STATE_PS2 || recv_state==STATE_PS3)
    {
        CheckPS(data);
    }
    else if(recv_state==STATE_PS4)
    {
        recv_state = STATE_TL;
        recv_packet.push_back(data);
        read_cs_update(data);
    }
    else if(recv_state==STATE_TL)
    {
        recv_state = STATE_PAYLOAD;
        recv_packet.push_back(data);
        read_cs_update(data);
    }
    else if(recv_state==STATE_PAYLOAD)
    {
        if(recv_packet.size()>=8)
        {
            uint16_t recv_TL;
            recv_TL = recv_packet[0]&0xff;
            recv_TL |= ((uint16_t)recv_packet[1])<<8;

            if(recv_packet.size()==(recv_TL-6))
            {
                if(data==PE)
                {
                    recv_state = STATE_CS;
                    read_cs_update(data);
                }
                else recv_state = STATE_INIT;
            } 
            else
            {
                recv_packet.push_back(data);
                read_cs_update(data);
            }
        }
        else
        {
            recv_packet.push_back(data);
            read_cs_update(data);
        }
    }
    else if(recv_state==STATE_CS)
    {
        if(data==read_cs_get()) ParsingPayload(recv_packet);
        
        recv_state = STATE_INIT;
    }

    if(recv_state==STATE_INIT) 
    {
        std::vector<uint8_t> packet = recv_packet;
        RecvPacketClear();
        for(auto& v: packet) AddPacketElement(v);
    }
}


void Gl::ThreadCallBack(void) 
{
    RecvPacketClear();

    while(thread_running==true)
    {
        if(comm_type==COMM_SERIAL)
        {
            uint8_t data;
            while( thread_running==true && serial_port->read(&data,1)==1 ) AddPacketElement(data);
        }
        else if(comm_type==COMM_UDP)
        {
            std::vector<uint8_t> recv_packet(2000);
            int recv_len = recv(sockfd, &recv_packet[0], recv_packet.size(), MSG_WAITFORONE);

            for(int i=0; i<recv_len; i++) AddPacketElement(recv_packet[i]);
        }
    }
}


//////////////////////////////////////////////////////////////
// Read GL Conditions
//////////////////////////////////////////////////////////////

std::string Gl::GetSerialNum(void)
{
    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_GET;
    uint8_t CAT0 = 0x02;
    uint8_t CAT1 = 0x0A;
    std::vector<uint8_t> DTn = {1};

    serial_num.clear();
    for(size_t i=0; i<50; i++)
    {
        WritePacket(PI, PL, SM, CAT0, CAT1, DTn);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if(serial_num.size()>0)
        {
            std::string out_str(serial_num.begin(), serial_num.end());
            return out_str;
        }
    }

    std::string out_str = "[ERROR] Serial Number is not received.";
    return out_str;
}

void Gl::ReadFrameData(Gl::framedata_t& frame_data, bool filter_on)
{
    if(frame_data_in.distance.size()==0) return;

    frame_data = frame_data_in;
    frame_data_in.angle.clear();
    frame_data_in.distance.clear();
    frame_data_in.pulse_width.clear();
    
    if(filter_on==true)
    {
        for(int i=0; i<(int)frame_data.distance.size()-1; i++)
        {
            if(frame_data.distance[i]>0.0 && frame_data.distance[i+1]>0.0)
            {
                double diff = (frame_data.distance[i]-frame_data.distance[i+1])/2.0;
                if(diff>0.01*frame_data.distance[i]) frame_data.distance[i] = 0.0;
                if(diff<-0.01*frame_data.distance[i]) frame_data.distance[i] = 0.0;
            }
        }
    }
}

//////////////////////////////////////////////////////////////
// Set GL Conditions
//////////////////////////////////////////////////////////////

void Gl::SetFrameDataEnable(bool framedata_enable)
{
    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_SET;
    uint8_t CAT0 = 0x1;
    uint8_t CAT1 = 0x3;
    std::vector<uint8_t> DTn = {framedata_enable};

    WritePacket(PI, PL, SM, CAT0, CAT1, DTn);
}