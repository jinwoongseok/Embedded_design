#include <avr/io.h>
/* nRF24L01 Command */
#define RD_REG          0x00
#define WR_REG          0x20
#define RX_PAYLOAD_WD   0x60
#define RD_RX_PAYLOAD   0x61
#define WR_TX_PAYLOAD   0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define RD_RX_PL_WID    0x60
#define WR_ACK_PAYLOAD  0xA8
#define WR_TX_PL_NO_ACK 0xB0
#define NOP             0xFF

/* nRF24L01 Register Address */
#define CONFIG_REG      0x00
#define EN_AA_REG       0x01
#define EN_RXADDR_REG   0x02
#define SETUP_AW_REG    0x03
#define SETUP_RETR_REG  0x04
#define RF_CH_REG       0x05
#define RF_SETUP_REG    0x06
#define STATUS_REG      0x07
#define OBSERVE_TX_REG  0x08
#define RPD_REG         0x09
#define RX_ADDR_P0_REG  0x0A
#define RX_ADDR_P1_REG  0x0B
#define RX_ADDR_P2_REG  0x0C
#define RX_ADDR_P3_REG  0x0D
#define RX_ADDR_P4_REG  0x0E
#define RX_ADDR_P5_REG  0x0F
#define TX_ADDR_REG     0x10
#define RX_PWD_P0_REG   0x11
#define RX_PWD_P1_REG   0x12
#define RX_PWD_P2_REG   0x13
#define RX_PWD_P3_REG   0x14
#define RX_PWD_P4_REG   0x15
#define RX_PWD_P5_REG   0x16
#define FIFO_STATUS_REG 0x17
#define DYNPD_REG       0x1C
#define FEATURE_REG     0x1D

#define ADDR_WIDTH      5
#define PAYLOAD_WIDTH   32


/* nRF24L01 Config Register bit value */
#define MASK_RX_DR      0x40
#define MASK_TX_DS      0x20
#define MASK_MAX_RT     0x10
#define EN_CRC          0x08
#define CRC_2BYTES      0x04
#define POWER_UP        0x02
#define PRIM_RX         0x01

/* nRF24L01 Status Register bit value */
#define RX_DR           0x40
#define TX_DS           0x20
#define MAX_RT          0x10
#define TX_FULL         0x01

enum nrf_pipe
{
    NRF_PIPE0,              /**< Select pipe0 */
    NRF_PIPE1,              /**< Select pipe1 */
    NRF_PIPE2,              /**< Select pipe2 */
    NRF_PIPE3,              /**< Select pipe3 */
    NRF_PIPE4,              /**< Select pipe4 */
    NRF_PIPE5,              /**< Select pipe5 */
    NRF_TX,                 /**< Refer to TX address*/
    NRF_ALL = 0xFF          /**< Close or open all pipes*/
};

enum crc_mode 
{
    NRF_CRC_OFF,        /**< CRC check disabled */
    NRF_CRC_8BIT = 2,   /**< CRC check set to 8-bit */
    NRF_CRC_16BIT       /**< CRC check set to 16-bit */
};

enum address_width
{
    NRF_AW_3BYTES = 1,      /**< Set address width to 3 bytes */
    NRF_AW_4BYTES,          /**< Set address width to 4 bytes */
    NRF_AW_5BYTES           /**< Set address width to 5 bytes */
};

enum op_mode 
{
    NRF_PTX,            /**< Primary TX operation */
    NRF_PRX             /**< Primary RX operation */
};

enum pwr_mode 
{
    NRF_PWR_DOWN,       /**< Device power-down */
    NRF_PWR_UP          /**< Device power-up */
};

enum data_rate 
{
    NRF_1MBPS,          /**< Datarate set to 1 Mbps  */
    NRF_2MBPS           /**< Datarate set to 2 Mbps  */
};

enum output_pwr 
{
    NRF_18DBM,          /**< Output power set to -18dBm */
    NRF_12DBM,          /**< Output power set to -12dBm */
    NRF_6DBM,           /**< Output power set to -6dBm  */
    NRF_0DBM            /**< Output power set to 0dBm   */
};

void nrf24_init(void);
void nrf24_send(uint8_t *buf, uint8_t len);
int nrf24_receive(uint8_t *buf, uint8_t len);
void nrf24_flush_tx_fifo(void);
void nrf24_flush_rx_fifo(void);
void nrf24_dump_registers(void);

//---------SPI------------------
volatile unsigned char* PORTA_DIR_ADDR = 0x400;
volatile unsigned char* PORTA_OUT_ADDR = 0x404;
volatile unsigned char* PORTA_PIN5CTRL_ADDR=0x415;
volatile unsigned char* SPI_CTRLA_ADDR = 0x8c0;
volatile unsigned char* SPI_CTRLB_ADDR = 0x8c1;
volatile unsigned char* SPI_DATA_ADDR = 0x8c4;
volatile unsigned char* SPI_INTFLAGS_ADDR = 0x8c3;
//------------------------------

//------------------USART----------------------
#define USART_BAUD_RATE(BAUD_RATE) ((float)(16000000 * 64 / (16 *(float)BAUD_RATE)) + 0.5) //보드레이트 계산
#define DATA uint16_t
#define BYTE uint8_t
#define LOBYTE(data) ((BYTE)(((DATA)(data)) & 0xFF)) //-> 2Byte 데이터 중 하위 Byte 반환
#define HIBYTE(data) ((BYTE)((((DATA)(data)) >> 8) & 0xFF)) //-> 2Byte 데이터 중 상위 Byte 반환

volatile unsigned int *USART_BAUD=0x828; //USART1 base:0x820
volatile unsigned char *USART_CTRLC=0x827;
volatile unsigned char *USART_CTRLB=0x826;
volatile unsigned char *USART_STATUS=0x824;
volatile unsigned char *USART_TXDATAL=0x822;
volatile unsigned char *USART_RXDATAL=0x820;
volatile unsigned char *PORTC_DIR_Reg_Addr=0x440;
//----------------------------------
#define _BV(bits) (1<<(bits))

volatile unsigned char *PORTA_DIR_Reg_Addr=0x400;
volatile unsigned char *PORTA_OUT_Reg_Addr=0x404;

unsigned char  ax_H, ay_H, az_H, tmp_H, gx_H, gy_H, gz_H;
unsigned char  ax, ay, az, tmp, gx, gy, gz;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, test;

double angle_x = 0, angle_y = 0, deg_x, deg_y;
double dgy_x, dgy_y;

const double dt = 0.002;
const double  Com = 0.98;

double Q_angle_x, Q_gyro_x, R_measure_x;
double anglex, biasx;
double P_x[2][2], K_x[2];

double Q_angle_y, Q_gyro_y, R_measure_y;
double angley, biasy;
double P_y[2][2], K_y[2];

double getkalman_x(double acc, double gyro, double dt){
  anglex += dt * (gyro - biasx);

  P_x[0][0] += dt * (dt*P_x[1][1]-P_x[0][1] - P_x[1][0] + Q_angle_x);
  P_x[0][1] -= dt * P_x[1][1];
  P_x[1][0] -= dt * P_x[1][1];
  P_x[1][1] += Q_gyro_x * dt;

  double S = P_x[0][0] + R_measure_x;
  K_x[0] = P_x[0][0] / S;
  K_x[1] = P_x[1][0] / S;

  double y = acc - anglex;
  anglex += K_x[0] * y;
  biasx += K_x[1] * y;

  double P_temp[2] = {P_x[0][0], P_x[0][1]};
  P_x[0][0] -= K_x[0] * P_temp[0];
  P_x[0][1] -= K_x[0] * P_temp[1];
  P_x[1][0] -= K_x[1] * P_temp[0];
  P_x[1][1] -= K_x[1] * P_temp[1];  

  return anglex;
}

void kal_init_x(double angle, double gyro, double measure){
  Q_angle_x = angle;
  Q_gyro_x = gyro;
  R_measure_x = measure;

  anglex = 0; biasx = 0;

  P_x[0][0] = 0;  P_x[0][1] = 0;  P_x[1][0] = 0;  P_x[1][1] = 0;
}

double getkalman_y(double acc, double gyro, double dt){
  angley += dt * (gyro - biasy);

  P_y[0][0] += dt * (dt*P_y[1][1]-P_y[0][1] - P_y[1][0] + Q_angle_y);
  P_y[0][1] -= dt * P_y[1][1];
  P_y[1][0] -= dt * P_y[1][1];
  P_y[1][1] += Q_gyro_y * dt;

  double S = P_y[0][0] + R_measure_y;
  K_y[0] = P_y[0][0] / S;
  K_y[1] = P_y[1][0] / S;

  double y = acc - angley;
  angley += K_y[0] * y;
  biasy += K_y[1] * y;

  double P_temp[2] = {P_y[0][0], P_y[0][1]};
  P_y[0][0] -= K_y[0] * P_temp[0];
  P_y[0][1] -= K_y[0] * P_temp[1];
  P_y[1][0] -= K_y[1] * P_temp[0];
  P_y[1][1] -= K_y[1] * P_temp[1];  

  return angley;
}

void kal_init_y(double angle, double gyro, double measure){
  Q_angle_y = angle;
  Q_gyro_y = gyro;
  R_measure_y = measure;

  angley = 0; biasy = 0;

  P_y[0][0] = 0;  P_y[0][1] = 0;  P_y[1][0] = 0;  P_y[1][1] = 0;
}

void TWI_Init(char baud)
{
  TWI0.MBAUD = baud;
  TWI0.MCTRLB |= TWI_FLUSH_bm;
  TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
}

void TWI_Enable()
{
  TWI0.MCTRLA = 1<<TWI_SMEN_bp | 1 << TWI_ENABLE_bp;
  TWI0_MSTATUS |= TWI_BUSSTATE_IDLE_gc;
}

char TWI_Start(char addr)
{
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_BUSY_gc)
  {
    TWI0.MCTRLB &= ~(1<<TWI_ACKACT_bp);
    TWI0.MADDR = addr;
    
    if (addr & 1)
    {
      while (!(TWI0_MSTATUS & TWI_RIF_bm));
    }
    else
    {
      while (!(TWI0_MSTATUS & TWI_WIF_bm));
    }
    return TWI0.MSTATUS;
  }
  else
    return TWI0.MSTATUS;
}

char TWI_Read(char * c, char ACKorNACK)
{
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm)==TWI_BUSSTATE_OWNER_gc)
  {
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    
    if (ACKorNACK)
    TWI0.MCTRLB &= ~(1<<TWI_ACKACT_bp); //Send ACK
    else
    TWI0.MCTRLB |= 1<<TWI_ACKACT_bp;  //Send NACK
    
    *c = TWI0.MDATA;
    
    return TWI0.MSTATUS;
  }
  else
    return TWI0.MSTATUS;
}

char TWI_Write(char data)
{
  if ((TWI0.MSTATUS&TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc)
  {
    while (!((TWI0.MSTATUS & TWI_WIF_bm) | (TWI0_MSTATUS & TWI_RXACK_bm)));
    
    TWI0.MDATA = data;
    return TWI0.MSTATUS;
  }
  else
    return TWI0.MSTATUS;
}

void TWI_Stop()
{
  TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
}

void getdata(){
  TWI_Start(0xD0);
  TWI_Write(0x3B);
  TWI_Start(0xD1);  
  TWI_Read(&ax_H,1);
  TWI_Read(&ax,1);
  TWI_Read(&ay_H,1);
  TWI_Read(&ay,1);
  TWI_Read(&az_H,1);
  TWI_Read(&az,1);
  TWI_Read(&tmp_H,1);
  TWI_Read(&tmp,1);
  TWI_Read(&gx_H,1);
  TWI_Read(&gx,1);
  TWI_Read(&gy_H,1);
  TWI_Read(&gy,1);
  TWI_Read(&gz_H,1);
  TWI_Read(&gz,0);
  TWI_Stop();
}

void deci_print(int16_t val){
  if((val&(0x8000))==0x8000){
  while(!((*(USART_STATUS))&B00100000)); //음수
    *(USART_TXDATAL)=0x2d;
    val = (val ^ 0xFFFF) + 1;
  }
  else{
    while(!((*(USART_STATUS))&B00100000)); //양수
    *(USART_TXDATAL)=0x2b;
    val = val & 0x7FFF;
  }
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=val/10000 + 48;
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=(val%10000)/1000 + 48;
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=((val%10000)%1000)/100 + 48;
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=(((val%10000)%1000)%100)/10 + 48;
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=(((val%10000)%1000)%100)%10 + 48; 
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=0x20;
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=0x20;            
}
void serialsend(uint8_t data){
  while(!((*(USART_STATUS))&B00100000));
  *(USART_TXDATAL)=data;
}


void us_delay(unsigned int value){
  unsigned int i;
  for(i=0;i<value*2;i++){
    asm(" NOP ");
    asm(" NOP ");
    asm(" NOP ");
  }
}

void ms_delay(unsigned int value){
  unsigned int i;
  for(i=0;i<value;i++){
    us_delay(666);
  }
}

void USART1_init(void){
  *(PORTC_DIR_Reg_Addr)&=(~B00000010); //INPUT STATE
  *(PORTC_DIR_Reg_Addr)|=B00000001; //OUTPUT STATE
  
  *(USART_BAUD)=(uint16_t)USART_BAUD_RATE(9600); //보드레이트9600 설정 후 8비트씩 나누어 BAUD레지스터에 할당
  *(USART_CTRLB)|=B11000000; //TX, RX enable 설정
  *(USART_CTRLC)=B00000011;  //패리티 비트 disable, 스탑비트 1bit, 데이터 사이즈 8bit
}

void USART1_sendChar(uint8_t c){
  while(!(*(USART_STATUS) & B00100000)){ //송신 버퍼가 비어있지 않으면 계속 무한루프로 대기상태
    
  }
  /*if(c!=10){ //엔터키 입력은 제외하기 위함
    char BigAlphabet=c-0x20; //소문자를 대문자로 변환하기 위해 아스키코드 값을 이용
    *(USART_TXDATAL) = BigAlphabet; //송신 버퍼가 비었다면 TXDATAL 레지스터에 문자 할당
  }*/
  *(USART_TXDATAL)=c;
}

void USART1_sendVal(uint8_t c){
  while(!(*(USART_STATUS) & B00100000)){ //송신 버퍼가 비어있지 않으면 계속 무한루프로 대기상태
    
  }
  *(USART_TXDATAL)=c/16+0x30;
  while(!(*(USART_STATUS) & B00100000)){ //송신 버퍼가 비어있지 않으면 계속 무한루프로 대기상태
    
  }
  *(USART_TXDATAL)=c%16+0x30;
}

void USART1_sendString(unsigned char *str){ //문자열 송신
  for(size_t i=0;i<strlen(str);i++){
    USART1_sendChar(str[i]);
  }
}

void spi_init() {
   
   // MISO pin automatically overrides to input, MOSI pin output, SCK pin output
   *(PORTA_DIR_ADDR) = B11010010;
   //*(PORTA_PIN5CTRL_ADDR) |= B00001000;
   // Clock 2X speed, MSB first, Clock divided by 16, set as master, spi enable
   *(SPI_CTRLA_ADDR) |= B00100111;
}

uint8_t spi_rw(uint8_t data)
{
    *(SPI_DATA_ADDR) = data;
    while (!(*(SPI_INTFLAGS_ADDR) & B10000000));
    return *(SPI_DATA_ADDR);
}

uint8_t nrf24_read_reg(uint8_t reg, uint8_t *data, int len)
{
    uint8_t val;
    int i;
    us_delay(20);
    *(PORTA_OUT_ADDR) &= B01111111; //SS low
    us_delay(20);
    spi_rw(RD_REG+reg);
    if (data && len)
    {
        for (i=0; i<len; i++)
        {
            *data++ = spi_rw(NOP);
        }
    }
    us_delay(20);
    val = spi_rw(NOP);
    us_delay(20);
    *(PORTA_OUT_ADDR) |= B10000000; //SS high
    return val;
}

static uint8_t nrf24_write_reg(uint8_t reg, uint8_t *data, int len)
{
    volatile uint8_t val = 0;
    int     i;

    *(PORTA_OUT_ADDR) &= B01111111; //SS set low

    if (reg < WR_REG)
        reg += WR_REG;

    val = spi_rw(reg);

    if (data && len)
    {
        for (i=0; i<len; i++)
        {
            spi_rw(*data++);
        }
    }

    *(PORTA_OUT_ADDR) |= B10000000; //SS set high


    return val;
}

static void nrf24_write_payload(uint8_t *data, int len)
{
    uint8_t dummy = PAYLOAD_WIDTH - len;

    *(PORTA_OUT_ADDR) &= B01111111; //SS set low
    spi_rw(WR_TX_PAYLOAD);
    while (len--)
        spi_rw(*data++); //payload에 data를 그 크기만큼 집어 넣음
    while (dummy--)
        spi_rw(0); //그 후 0을 payload의 길이가 32byte가 될 때까지 집어 넣는다.
    *(PORTA_OUT_ADDR) |= B10000000; //SS set high
}

static void nrf24_close_pipe(enum nrf_pipe pipe)
{
    uint8_t aa = 0;
    uint8_t rxaddr = 0;

    if (pipe != NRF_ALL)
    {
        aa = nrf24_read_reg(EN_AA_REG, NULL, 0) & ~(1<<pipe);
        rxaddr = nrf24_read_reg(EN_RXADDR_REG, NULL, 0) & ~(1<<pipe);
    }

    nrf24_write_reg(EN_AA_REG, &aa, 1);
    nrf24_write_reg(EN_RXADDR_REG, &rxaddr, 1);
}

static void nrf24_open_pipe(enum nrf_pipe pipe)
{
    uint8_t aa = 0x3F;
    uint8_t rxaddr = 0x3F;

    if (pipe != NRF_ALL)
    {
        aa = nrf24_read_reg(EN_AA_REG, NULL, 0) | (1<<pipe);
        rxaddr = nrf24_read_reg(EN_RXADDR_REG, NULL, 0) | (1<<pipe);
    }

    nrf24_write_reg(EN_AA_REG, &aa, 1);
    nrf24_write_reg(EN_RXADDR_REG, &rxaddr, 1);
}

static void nrf24_crc_mode(enum crc_mode crc)
{
    uint8_t config = 0;

    config = (nrf24_read_reg(CONFIG_REG, NULL, 0) & ~0x0C) | (crc << 2);
    nrf24_write_reg(CONFIG_REG, &config, 1);
}

static void nrf24_auto_retr(uint8_t arc, uint16_t ard)
{
    uint8_t data = (((ard/250)-1) << 4) | arc;

    nrf24_write_reg(SETUP_RETR_REG, &data, 1);
}

static void nrf24_addr_width(enum address_width aw)
{
    nrf24_write_reg(SETUP_AW_REG, (uint8_t*)&aw, 1);
}

static void nrf24_set_addr(enum nrf_pipe pipe, uint8_t *addr)
{
    uint8_t aw = nrf24_read_reg(SETUP_AW_REG, NULL, 0) + 2;

    switch (pipe)
    {
        case NRF_PIPE0:
        case NRF_PIPE1:
        case NRF_TX: //TX_ADDR register transmit address set
            nrf24_write_reg(RX_ADDR_P0_REG + pipe, addr, aw);
            break;

        case NRF_PIPE2 ... NRF_PIPE5:
            nrf24_write_reg(RX_ADDR_P0_REG + pipe, addr, 1);
            break;

        default:
            break;
    }

}

static void nrf24_op_mode(enum op_mode mode)
{
    uint8_t config = nrf24_read_reg(CONFIG_REG, NULL, 0);

    if (mode == NRF_PTX)
    {
        config &= ~PRIM_RX;
    }
    else
    {
        config |= PRIM_RX;
    }

    nrf24_write_reg(CONFIG_REG, &config, 1);
}

static void nrf24_rf_channel(uint8_t rf_ch)
{
    nrf24_write_reg(RF_CH_REG, (uint8_t*)&rf_ch, 1);
}

static void nrf24_rf_data_rate(enum data_rate bps)
{
    uint8_t rf = nrf24_read_reg(RF_SETUP_REG, NULL, 0);

    if (bps == NRF_1MBPS)
    {
        rf &= ~(1<<3);
    }
    else
    {
        rf |= (1<<3);
    }

    nrf24_write_reg(RF_SETUP_REG, &rf, 1);
}

static void nrf24_power_mode(enum pwr_mode mode)
{
    uint8_t config = nrf24_read_reg(CONFIG_REG, NULL, 0);

    if (mode == NRF_PWR_DOWN)
    {
        config &= ~POWER_UP;
    }
    else
    {
        config |= POWER_UP;
    }

    nrf24_write_reg(CONFIG_REG, &config, 1);
}

static void nrf24_flush_tx(void)
{
    nrf24_write_reg(FLUSH_TX, NULL, 0);

}

static void nrf24_flush_rx(void)
{
    nrf24_write_reg(FLUSH_RX, NULL, 0);
}

static void nrf24_payload_width(enum nrf_pipe pipe, uint8_t width)
{
    nrf24_write_reg(RX_PWD_P0_REG + pipe, &width, 1);
}

static void nrf24_read_payload(uint8_t *data, int len, uint8_t width)
{
    uint8_t dummy = width - len;

    *(PORTA_OUT_ADDR) &= ~B10000000; //cs low
    spi_rw(RD_RX_PAYLOAD);
    while (len--)
        *data++ = spi_rw(NOP);
    while (dummy--)
        spi_rw(NOP);
    *(PORTA_OUT_ADDR) |= B10000000; //cs high
}

void nrf24_dump_registers()
{
    uint8_t addr[ADDR_WIDTH];
    int i;
    uint8_t temp;
    USART1_sendString("\nnRF24L01 Registers Value\n");

    USART1_sendString("    CONFIG : ");
    temp=nrf24_read_reg(CONFIG_REG, NULL, 0);
    USART1_sendVal(temp);
    USART1_sendString("\n");
    USART1_sendString("     EN_AA : ");
    USART1_sendVal(nrf24_read_reg(EN_AA_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" EN_RXADDR : ");
    USART1_sendVal(nrf24_read_reg(EN_RXADDR_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("  SETUP_AW : ");
    USART1_sendVal(nrf24_read_reg(SETUP_AW_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("SETUP_RETR : ");
    USART1_sendVal(nrf24_read_reg(SETUP_RETR_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("     RF_CH : ");
    USART1_sendVal(nrf24_read_reg(RF_CH_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("  RF_SETUP : ");
    USART1_sendVal(nrf24_read_reg(RF_SETUP_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("    STATUS : ");
    USART1_sendVal(nrf24_read_reg(STATUS_REG, NULL, 0));
    USART1_sendString("\n");

    nrf24_read_reg(RX_ADDR_P0_REG, addr, ADDR_WIDTH);
    USART1_sendString("RX_ADDR_P0 : ");
    for (i=0; i<ADDR_WIDTH; i++)
        USART1_sendVal(addr[i]);
    USART1_sendString("\n");

    nrf24_read_reg(RX_ADDR_P1_REG, addr, ADDR_WIDTH);
    USART1_sendString("RX_ADDR_P1 : ");
    for (i=0; i<ADDR_WIDTH; i++)
        USART1_sendVal(addr[i]);
    USART1_sendString("\n");

    USART1_sendString("RX_ADDR_P2 : ");
    USART1_sendVal(nrf24_read_reg(RX_ADDR_P2_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("RX_ADDR_P3 : ");
    USART1_sendVal(nrf24_read_reg(RX_ADDR_P3_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("RX_ADDR_P4 : ");
    USART1_sendVal(nrf24_read_reg(RX_ADDR_P4_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("RX_ADDR_P5 : ");
    USART1_sendVal(nrf24_read_reg(RX_ADDR_P5_REG, NULL, 0));
    USART1_sendString("\n");
  
    nrf24_read_reg(TX_ADDR_REG, addr, ADDR_WIDTH);
    USART1_sendString("   TX_ADDR : ");
    for (i=0; i<ADDR_WIDTH; i++)
        USART1_sendVal(addr[i]);
    USART1_sendString("\n");

    USART1_sendString(" RX_PWD_P0 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P0_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" RX_PWD_P1 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P1_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" RX_PWD_P2 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P2_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" RX_PWD_P3 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P3_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" RX_PWD_P4 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P4_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString(" RX_PWD_P5 : ");
    USART1_sendVal(nrf24_read_reg(RX_PWD_P5_REG, NULL, 0));
    USART1_sendString("\n");

    USART1_sendString("FIFO_STATUS: ");
    USART1_sendVal(nrf24_read_reg(FIFO_STATUS_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("     DYNPD : ");
    USART1_sendVal(nrf24_read_reg(DYNPD_REG, NULL, 0));
    USART1_sendString("\n");
    USART1_sendString("   FEATURE : ");
    USART1_sendVal(nrf24_read_reg(FEATURE_REG, NULL, 0));
    USART1_sendString("\n");
}

static uint8_t tranceiver_addr[ADDR_WIDTH] = {0x00, 0x00, 0x00, 0x00, 0x02};
void nrf24_init()
{
    *(PORTA_OUT_ADDR) &= B11111101; //ce low
    *(PORTA_OUT_ADDR) |= B10000000; //ss high
    ms_delay(20);

    nrf24_close_pipe(NRF_ALL);
    nrf24_open_pipe(NRF_PIPE0);
    nrf24_crc_mode(NRF_CRC_16BIT);
    nrf24_auto_retr(15, 500);

    nrf24_addr_width(NRF_AW_5BYTES);
    nrf24_set_addr(NRF_TX, tranceiver_addr);
    nrf24_set_addr(NRF_PIPE0, tranceiver_addr);

    nrf24_op_mode(NRF_PTX);
    //nrf24_payload_width(NRF_PIPE0, PAYLOAD_WIDTH);
    nrf24_rf_channel(76);
    nrf24_rf_data_rate(NRF_1MBPS);

    nrf24_flush_tx();
    nrf24_flush_rx();

    nrf24_power_mode(NRF_PWR_UP);

    ms_delay(20);
}

void nrf24_send(uint8_t *buf, uint8_t len)
{
    uint8_t status, clear;

    nrf24_write_payload(buf, len);

    *(PORTA_OUT_ADDR) |= B00000010; //ce high
    us_delay(20);
    *(PORTA_OUT_ADDR) &= B11111101; //ce low
    
    do
    {
        status = nrf24_read_reg(STATUS_REG, NULL, 0);
    } while(!(status & (TX_DS | MAX_RT)));

    if (status & TX_DS) //만약 송신이 제대로 되었다면
    {
        clear = TX_DS; //interrupt flag clear
        nrf24_write_reg(STATUS_REG, &clear, 1);
    }
    if (status & MAX_RT) //계속 ACK를 수신측에서 보내지 않는다면
    {
        clear = MAX_RT;
        nrf24_write_reg(STATUS_REG, &clear, 1); //clear
        nrf24_flush_tx();
        //USART1_sendString("Maximum number of Tx\n");
    }
}

int nrf24_receive(uint8_t *buf, uint8_t len)
{
    uint8_t status, clear;

    do
    {
        status = nrf24_read_reg(STATUS_REG, NULL, 0);
    } while(!(status & RX_DR));

    if (status & RX_DR)
    {
        //uint8_t pipe = (nrf24_read_reg(STATUS_REG, NULL, 0) >> 1) & 7;
        uint8_t width = nrf24_read_reg(RX_PAYLOAD_WD, NULL, 0);

        nrf24_read_payload(buf, len, width);
        clear = RX_DR;
        nrf24_write_reg(STATUS_REG, &clear, 1);

        return width;
    }

    return 0;
}

static uint8_t rx_buf[PAYLOAD_WIDTH];

void setup() {
  // put your setup code here, to run once:
  *(PORTA_DIR_Reg_Addr) |=B00000010;
  *(PORTA_OUT_Reg_Addr) |=B00000010;
  TWI_Init(0x0f);
  TWI_Enable(); 
  TWI_Start(0xD0);
  TWI_Write(0x6B);
  TWI_Write(0);
  TWI_Stop();
  
  spi_init();
  USART1_init();
  nrf24_init();
  nrf24_dump_registers();
  //*(PORTA_OUT_ADDR) |= B00000010; //ce high

  kal_init_x(0.0001, 0.0001, 0.0001);
  kal_init_y(0.0001, 0.0001, 0.0001);
}
volatile uint16_t ab;
void loop() {
  getdata();
  AcX=ax_H<<8|ax;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=ay_H<<8|ay;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=az_H<<8|az;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=tmp_H<<8|tmp;  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=gx_H<<8|gx;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=gy_H<<8|gy;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=gz_H<<8|gz;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  deg_x = atan2(AcX, AcZ) * 180 / PI ;
  dgy_x = GyY / 131. ;  //16-bit data to 250 deg/sec
  angle_x = (Com * (angle_x + (dgy_x * dt))) + ((1-Com) * deg_x) ; //complementary filter 

  double value_x = getkalman_x(deg_x, dgy_x,0.002);
  double sum_x = 0.1*value_x + 0.9*angle_x;
  
  deg_y = atan2(AcY, AcZ) * 180 / PI ;
  dgy_y = GyX / 131. ;  //16-bit data to 250 deg/sec
  angle_y = (Com * (angle_y + (dgy_y * dt))) + ((1-Com) * deg_y) ; //complementary filter

  double sum_y = angle_y;
  
  if(sum_x > 90) sum_x = 90;  else if(sum_x < -90) sum_x = -90;
  if(sum_y > 90) sum_y = 90;  else if(sum_y < -90) sum_y = -90;

  ab = (int8_t)sum_y << 8 | (int8_t)sum_x;
  
  nrf24_send((void*)&ab,2);
}
